#!/usr/bin/env python3
"""Tripod gait for the simulated ROSpider.

On the real robot, driver/controller walks the legs with the aarch64-only kinematics.so, which
cannot run on a PC. In simulation this node takes that role: it subscribes to the robot's command
topic /controller/cmd_vel, steps the legs through leg_controller, and forwards the body twist
(limited to what the stride allows) to Gazebo's VelocityControl, which moves the base. Feet in
stance move exactly opposite to the body, so they stay planted on the floor. Leg IK is solved
numerically on the URDF chains, so it follows the model's real geometry.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

LEGS = ('LF', 'LM', 'LR', 'RF', 'RM', 'RR')
TRIPODS = (('LF', 'RM', 'LR'), ('RF', 'LM', 'RR'))

RATE = 50.0              # control rate [Hz]
STRIDE = 0.05            # preferred foot travel per stance [m]
STRIDE_MAX = 0.07        # longest stance travel; faster commands are scaled down [m]
SWING_TIME = (0.3, 1.0)  # min/max duration of one swing (= one stance) [s]
STEP_HEIGHT = 0.03       # foot lift [m]
CMD_TIMEOUT = 0.5        # treat cmd_vel as zero after this long without a message [s]


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis, angle):
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def _body_velocity(vx, vy, wz, p):
    """Velocity of body point p (base_footprint frame) for body twist (vx, vy, wz)."""
    return np.array([vx - wz * p[1], vy + wz * p[0], 0.0])


class Leg:
    """Chain base_footprint -> end_<leg> with its three revolute joints."""

    def __init__(self, urdf, leg):
        joints = {j.find('child').get('link'): j for j in urdf.findall('joint')}
        chain, link = [], 'end_' + leg
        while link in joints:
            j = joints[link]
            o = j.find('origin')
            xyz = np.array([float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()])
            rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            axis, limit = None, None
            if j.get('type') == 'revolute':
                axis = np.array([float(v) for v in j.find('axis').get('xyz').split()])
                axis /= np.linalg.norm(axis)
                limit = j.find('limit')
            chain.append((xyz, _rpy(*rpy), axis, j.get('name'), limit))
            link = j.find('parent').get('link')
        self.chain = chain[::-1]
        revolute = [c for c in self.chain if c[2] is not None]
        self.names = [c[3] for c in revolute]
        self.lower = np.array([float(c[4].get('lower')) for c in revolute])
        self.upper = np.array([float(c[4].get('upper')) for c in revolute])
        self.q = np.zeros(len(revolute))
        self.neutral = self.foot(self.q)[0]  # all joints at 0 is the standing pose

    def foot(self, q):
        """Foot position and Jacobian d(foot)/dq, in base_footprint."""
        R, p, i = np.eye(3), np.zeros(3), 0
        axes, origins = [], []
        for xyz, rot, axis, _, _ in self.chain:
            p = p + R @ xyz
            R = R @ rot
            if axis is not None:
                axes.append(R @ axis)
                origins.append(p)
                R = R @ _axis_angle(axis, q[i])
                i += 1
        return p, np.column_stack([np.cross(a, p - o) for a, o in zip(axes, origins)])

    def solve(self, target):
        """Damped least-squares IK from the previous solution, retried from the standing pose if
        that misses (a warm start near the edge of reach can drift into the other knee branch)."""
        best = None
        for q in (self.q, np.zeros_like(self.q)):
            for _ in range(20):
                p, jac = self.foot(q)
                err = target - p
                if err @ err < 1e-8:
                    break
                step = jac.T @ np.linalg.solve(jac @ jac.T + 1e-5 * np.eye(3), err)
                q = np.clip(q + np.clip(step, -0.3, 0.3), self.lower, self.upper)
            miss = np.linalg.norm(target - self.foot(q)[0])
            if best is None or miss < best[0]:
                best = (miss, q)
            if miss < 1e-3:
                break
        self.q = best[1]
        return self.q


class SimGait(Node):

    def __init__(self):
        super().__init__('sim_gait')
        urdf = ET.fromstring(self.declare_parameter('robot_description', '').value)
        self.legs = {name: Leg(urdf, name) for name in LEGS}
        self.joint_names = [joint for name in LEGS for joint in self.legs[name].names]
        self.feet = {name: leg.neutral.copy() for name, leg in self.legs.items()}
        self.liftoff = dict(self.feet)
        self.phase = 0.0  # tripod 0 swings during [0, 0.5), tripod 1 during [0.5, 1)
        self.walking = False
        self.cmd, self.cmd_time, self.last = Twist(), None, None
        self.create_subscription(Twist, '/controller/cmd_vel', self.on_cmd, 10)
        self.base_pub = self.create_publisher(Twist, '/sim/base_cmd_vel', 10)
        self.leg_pub = self.create_publisher(JointTrajectory, '/leg_controller/joint_trajectory', 10)
        self.create_timer(1.0 / RATE, self.step)

    def on_cmd(self, msg):
        self.cmd, self.cmd_time = msg, self.get_clock().now()

    def step(self):
        now = self.get_clock().now()
        if self.last is None or now <= self.last:  # sim clock not running yet
            self.last = now
            return
        dt = min((now - self.last).nanoseconds * 1e-9, 0.1)
        self.last = now

        vx = vy = wz = 0.0
        if self.cmd_time is not None and (now - self.cmd_time).nanoseconds * 1e-9 < CMD_TIMEOUT:
            vx, vy, wz = self.cmd.linear.x, self.cmd.linear.y, self.cmd.angular.z

        # Pick the swing time for the preferred stride; above the longest stride at the fastest
        # step rate the legs cannot keep up, so the body twist is scaled down instead.
        swing_min, swing_max = SWING_TIME
        top = max(np.linalg.norm(_body_velocity(vx, vy, wz, leg.neutral)) for leg in self.legs.values())
        if top * swing_min > STRIDE_MAX:
            scale = STRIDE_MAX / (top * swing_min)
            vx, vy, wz, top = vx * scale, vy * scale, wz * scale, top * scale
        moving = top > 1e-4
        swing_time = min(max(STRIDE / top, swing_min), swing_max) if moving else swing_max

        body = Twist()
        body.linear.x, body.linear.y, body.angular.z = vx, vy, wz
        self.base_pub.publish(body)

        if not self.walking:
            if not moving:
                return
            self.walking, self.phase = True, 0.0
            for name in TRIPODS[0]:
                self.liftoff[name] = self.feet[name].copy()

        tripod = 0 if self.phase < 0.5 else 1
        end = 0.5 * (tripod + 1)
        self.phase = min(self.phase + dt / (2 * swing_time), end)
        u = (self.phase - 0.5 * tripod) * 2.0
        for name, leg in self.legs.items():
            if name in TRIPODS[tripod]:
                # Land ahead of neutral by half the coming stance travel, so stance is centred
                land = leg.neutral + _body_velocity(vx, vy, wz, leg.neutral) * (swing_time / 2)
                foot = self.liftoff[name] + (land - self.liftoff[name]) * (0.5 - 0.5 * math.cos(math.pi * u))
                # sin^2: zero vertical speed at lift-off and touch-down, so the foot sets down softly
                foot[2] = leg.neutral[2] + STEP_HEIGHT * math.sin(math.pi * u) ** 2
            else:
                foot = self.feet[name] - _body_velocity(vx, vy, wz, self.feet[name]) * dt
                foot[2] = leg.neutral[2]
            self.feet[name] = foot

        if self.phase >= end:  # swing done: stop once settled, otherwise the other tripod lifts
            self.phase %= 1.0
            if not moving and all(np.linalg.norm(self.feet[n] - self.legs[n].neutral) < 1e-3 for n in LEGS):
                self.walking = False
            else:
                for name in TRIPODS[1 - tripod]:
                    self.liftoff[name] = self.feet[name].copy()

        point = JointTrajectoryPoint()
        point.positions = [float(v) for name in LEGS for v in self.legs[name].solve(self.feet[name])]
        point.time_from_start = Duration(seconds=2.0 / RATE).to_msg()
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        msg.points = [point]
        self.leg_pub.publish(msg)


def main():
    rclpy.init()
    try:
        rclpy.spin(SimGait())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
