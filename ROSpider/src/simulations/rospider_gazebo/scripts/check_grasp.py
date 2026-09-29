#!/usr/bin/env python3
"""ตรวจโจทย์ 3D Object Grasping: สั่งหุ่นไปหยิบบล็อกสีน้ำเงิน แล้วดูว่าวางลงตรงแผ่นเป้าหมายไหม

    ros2 run rospider_gazebo check_grasp.py

Orders the blue block from track_and_grab (~/pick blue: the same as its blue button), follows
pick_and_place's ~/state and the odometry (the sim's odometry is the world pose) until the block
has been put down or the robot gives no sign of getting further, then reads where the block came
to rest with `gz model -p` and scores the run with rospider_gazebo/grasp_check.py. Times are on
the sim clock.
"""

import math
import subprocess
import sys
import time

import rclpy
from interfaces.srv import SetString
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rospider_gazebo import grasp_check
from std_msgs.msg import String
from std_srvs.srv import SetBool

WAIT = 90.0
START_TOLERANCE_M = 0.1
#: No pick started this long after the order: the robot is not getting to the block.
PICK_LIMIT = 120.0
#: ... or it has stood still (no step, no turn) this long without starting one.
STILL_LIMIT = 20.0
#: The pick started but the block was not lifted, or lifted but not put down, this long after.
STEP_LIMIT = 60.0
#: The block settles on the floor for this long before it is read.
SETTLE = 2.0
#: No new /odom for this many wall seconds: the sim closed, crashed or is paused.
ODOM_SILENCE = 10.0
WALL_LIMIT = (PICK_LIMIT + 2 * STEP_LIMIT) * 5
FREE = ('IDLE', 'DONE')


class OdomLost(Exception):
    """Odometry stopped arriving."""


def block_pose():
    """The blue block's (x, y, z) from Gazebo, or None."""
    try:
        result = subprocess.run(['gz', 'model', '-m', grasp_check.BLOCK_MODEL, '-p'],
                                capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return grasp_check.parse_gz_pose(result.stdout)


class Checker:
    def __init__(self):
        self.node = rclpy.create_node(
            'check_grasp', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.pick = self.node.create_client(SetString, '/track_and_grab/pick')
        self.running = self.node.create_client(SetBool, '/track_and_grab/set_running')
        self.odom = None
        self.odom_wall = None
        self.states = []
        self.node.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.node.create_subscription(
            String, '/pick_and_place/state', lambda msg: self.states.append(msg.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, stamp)
        self.odom_wall = time.monotonic()

    def ready(self):
        end = time.monotonic() + WAIT
        while time.monotonic() < end and not self._up():
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return self._up()

    def _up(self):
        return (self.odom is not None and bool(self.states) and self.pick.service_is_ready()
                and self.running.service_is_ready())

    def call(self, client, request, timeout=5.0):
        future = client.call_async(request)
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return future.result() if future.done() else None

    def order(self):
        """Order the blue block: the response, or None. A lost reply is resent; if the first order
        got through after all, the job has started and the resent one is refused as busy, which
        counts as accepted."""
        for attempt in range(3):
            response = self.call(self.pick, SetString.Request(data=grasp_check.COLOR))
            if response is not None:
                if attempt and not response.success and response.message.startswith('busy'):
                    response.success = True
                return response
        return None

    def stop(self):
        request = SetBool.Request()
        request.data = False
        self.call(self.running, request, timeout=2.0)

    def _spin(self, wall_end):
        rclpy.spin_once(self.node, timeout_sec=0.1)
        now = time.monotonic()
        if now - self.odom_wall > ODOM_SILENCE or now > wall_end:
            raise OdomLost()

    def run(self):
        """The run's summary once it is over, or a message saying why there is none."""
        seen = len(self.states)
        response = self.order()
        if response is None:
            return 'track_and_grab ไม่ตอบคำสั่ง - ปิด-เปิด launch ใหม่แล้วตรวจอีกครั้ง'
        if not response.success:
            return f'track_and_grab ไม่รับคำสั่ง ({response.message}) - ปิด-เปิด launch ใหม่แล้วตรวจอีกครั้ง'
        wall_end = time.monotonic() + WALL_LIMIT
        begin = step = still = self.odom[3]
        anchor = self.odom
        states = []
        while True:
            self._spin(wall_end)
            new = self.states[seen:]
            seen = len(self.states)
            if new:
                states += new
                step = self.odom[3]
            now = self.odom[3]
            turned = abs(math.atan2(math.sin(self.odom[2] - anchor[2]),
                                    math.cos(self.odom[2] - anchor[2])))
            if math.dist(self.odom[:2], anchor[:2]) > 0.01 or turned > math.radians(3.0):
                anchor, still = self.odom, now
            if 'CARRY' in states and states[-1] in FREE:
                break                                   # put down
            if 'LOOK' not in states and (now - begin > PICK_LIMIT or now - still > STILL_LIMIT):
                break                                   # never got to picking
            if 'LOOK' in states and now - step > STEP_LIMIT:
                break                                   # stuck in the pick or the place
        self.stop()
        settle = self.odom[3]
        while self.odom[3] - settle < SETTLE:
            self._spin(wall_end)
        return grasp_check.summarise(self.odom[:3], states, block_pose())


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    checker = Checker()
    print('ตรวจโจทย์ 3D Object Grasping - ถ้าเพิ่งตรวจไปรอบหนึ่ง ให้ปิด-เปิด launch ใหม่ก่อน '
          'หุ่นและบล็อกจะได้กลับที่เดิม')
    try:
        if not checker.ready():
            print('ไม่พบ track_and_grab / pick_and_place - เปิด ros2 launch rospider_gazebo '
                  'grasp_challenge.launch.py แล้วรอให้หน้าต่าง track_and_grab ขึ้นก่อน')
            return 1
        x, y = checker.odom[:2]
        block = block_pose()
        if math.hypot(x, y) > START_TOLERANCE_M or block is None or \
                math.dist(block, grasp_check.BLOCK) > 0.02:
            print('หุ่นหรือบล็อกไม่ได้อยู่ที่จุดเริ่ม - ปิด-เปิด launch ใหม่ก่อนตรวจ')
            return 1
        print('สั่งหุ่นไปหยิบบล็อกสีน้ำเงินแล้ว (ใช้เวลาราว 1 นาที) ...', flush=True)
        try:
            summary = checker.run()
        except OdomLost:
            checker.stop()
            print('ไม่ได้รับตำแหน่งหุ่น (/odom) - Gazebo ปิด ค้าง หรือถูก pause อยู่ '
                  'ปิด-เปิด launch ใหม่แล้วตรวจอีกครั้ง')
            return 1
        if isinstance(summary, str):
            print(summary)
            return 1
        levels = grasp_check.evaluate(summary)
        print(grasp_check.format_report(levels))
        return 0 if all(level.status == 'pass' for level in levels) else 1
    except KeyboardInterrupt:
        try:
            checker.stop()
        except Exception:       # noqa: BLE001 -- shutting down anyway
            pass
        print('หยุดตรวจแล้ว (สั่งหุ่นหยุด)')
        return 130
    finally:
        checker.node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
