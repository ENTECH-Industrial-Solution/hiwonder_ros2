#!/usr/bin/env python3
"""Drive the exercise room's route like a participant with teleop: turn to face each waypoint,
walk to it, return to the start facing +x (the revisit gives V-SLAM its loop closure).

    python3 tools/drive_route.py            # both rooms (~300 s)
    python3 tools/drive_route.py --room-a   # room A only
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

ROUTE = [(0.5, 0.55), (1.5, 0.55), (1.5, -1.6), (2.5, -1.6), (3.6, -1.6), (3.6, -0.3), (3.5, 0.4),
         (3.5, 1.5), (3.5, 0.4), (2.9, -0.4), (2.4, -1.5), (1.5, -1.6), (1.5, 0.55), (0.5, 0.55),
         (0.5, -1.5), (-0.6, -1.5), (-0.6, 0.5), (0.3, 1.5), (0.0, 0.0)]
ROOM_A = [(0.5, -1.5), (-0.6, -1.5), (-0.6, 0.5), (0.3, 1.5), (0.0, 0.0)]
SPEED, TURN, TOLERANCE, LEG_TIMEOUT = 0.15, 0.4, 0.06, 60.0


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--room-a', action='store_true')
    route = ROOM_A if parser.parse_args().room_a else ROUTE
    rclpy.init()
    node = rclpy.create_node('drive_route')
    pose = {}

    def on_odom(msg):
        q = msg.pose.pose.orientation
        pose.update(x=msg.pose.pose.position.x, y=msg.pose.pose.position.y,
                    yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    node.create_subscription(Odometry, '/odom', on_odom, 10)
    pub = node.create_publisher(Twist, '/controller/cmd_vel', 1)
    wait_end = time.time() + 60.0
    while 'x' not in pose:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() > wait_end:
            raise SystemExit('no /odom for 60 s: is the simulation running?')

    def turn_to(heading):
        end = time.time() + 30.0
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            error = wrap(heading - pose['yaw'])
            if abs(error) < 0.05:
                break
            twist = Twist()
            twist.angular.z = max(-TURN, min(TURN, 1.5 * error))
            pub.publish(twist)
        pub.publish(Twist())

    start = time.time()
    start_pose = (pose['x'], pose['y'])
    for gx, gy in route:
        turn_to(math.atan2(gy - pose['y'], gx - pose['x']))
        end = time.time() + LEG_TIMEOUT
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            dx, dy = gx - pose['x'], gy - pose['y']
            distance = math.hypot(dx, dy)
            if distance < TOLERANCE:
                break
            error = wrap(math.atan2(dy, dx) - pose['yaw'])
            twist = Twist()
            twist.linear.x = min(SPEED, distance) * max(0.0, math.cos(error))
            twist.angular.z = max(-TURN, min(TURN, 1.5 * error))
            pub.publish(twist)
        else:
            print(f'leg to ({gx}, {gy}) timed out at ({pose["x"]:.2f}, {pose["y"]:.2f})')
            if (pose['x'], pose['y']) == start_pose:
                raise SystemExit('the robot has not moved: is the simulation paused or frozen?')
    turn_to(0.0)
    pub.publish(Twist())
    print(f'route done in {time.time() - start:.0f} s', flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
