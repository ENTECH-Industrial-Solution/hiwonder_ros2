#!/usr/bin/env python3
"""ตรวจโจทย์ AprilTag Tracking: สั่งหุ่นเดินตามป้าย แล้วดูว่าไปหยุดที่ไหน

    ros2 run rospider_gazebo check_tag.py

Switches apriltag_track on (~/set_running), feeds the odometry (the sim's odometry is the world
pose) to rospider_gazebo/tag_check.py until the robot has walked and stood still, then switches
it off and scores the stop. Times are on the sim clock.
"""

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import SetBool
from rospider_gazebo import tag_check

START_TOLERANCE_M = 0.1
WAIT = 60.0
#: No new /odom for this many wall seconds: the sim closed, crashed or is paused.
ODOM_SILENCE = 10.0
WALL_LIMIT = tag_check.RUN_LIMIT * 5


class OdomLost(Exception):
    """Odometry stopped arriving."""


class Checker:
    def __init__(self):
        self.node = rclpy.create_node(
            'check_tag', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.running = self.node.create_client(SetBool, '/apriltag_track/set_running')
        self.odom = None
        self.odom_wall = None
        self.node.create_subscription(Odometry, '/odom', self._on_odom, 10)

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, stamp)
        self.odom_wall = time.monotonic()

    def ready(self):
        end = time.monotonic() + WAIT
        while time.monotonic() < end and (self.odom is None or not self.running.service_is_ready()):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return self.odom is not None and self.running.service_is_ready()

    def set_running(self, on, timeout=5.0):
        """The response, or None; resent when a reply is lost (the call is idempotent)."""
        request = SetBool.Request()
        request.data = on
        for _ in range(3 if on else 1):
            future = self.running.call_async(request)
            end = time.monotonic() + timeout
            while not future.done() and time.monotonic() < end:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            if future.done():
                return future.result()
        return None

    def run(self):
        if self.set_running(True) is None:
            return None
        wall_end = time.monotonic() + WALL_LIMIT
        tracker = tag_check.TagRun()
        last = None
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            now = time.monotonic()
            if now - self.odom_wall > ODOM_SILENCE or now > wall_end:
                raise OdomLost()
            if self.odom is last:
                continue
            last = self.odom
            tracker.feed(*last)
            if tracker.done(last[3]):
                break
        self.set_running(False)
        return tracker.summary()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    checker = Checker()
    print('ตรวจโจทย์ AprilTag - ถ้าเพิ่งตรวจไปรอบหนึ่ง ให้ปิด-เปิด launch ใหม่ก่อน '
          'หุ่นจะได้เริ่มที่จุดเกิด')
    try:
        if not checker.ready():
            print('ไม่พบ apriltag_track - เปิด ros2 launch rospider_gazebo apriltag_challenge.launch.py '
                  'แล้วรอให้หน้าต่าง image ขึ้นก่อน')
            return 1
        x, y = checker.odom[:2]
        if math.hypot(x, y) > START_TOLERANCE_M:
            print('หุ่นไม่ได้อยู่ที่จุดเกิด - ปิด-เปิด launch ใหม่ก่อนตรวจ')
            return 1
        print('สั่งหุ่นเริ่มเดินแล้ว ...', flush=True)
        try:
            summary = checker.run()
        except OdomLost:
            checker.set_running(False, timeout=2.0)
            print('ไม่ได้รับตำแหน่งหุ่น (/odom) - Gazebo ปิด ค้าง หรือถูก pause อยู่ '
                  'ปิด-เปิด launch ใหม่แล้วตรวจอีกครั้ง')
            return 1
        if summary is None:
            checker.set_running(False, timeout=2.0)     # a start request may have got through
            print('apriltag_track ไม่ตอบคำสั่ง - ปิด-เปิด launch ใหม่แล้วตรวจอีกครั้ง')
            return 1
        levels = tag_check.evaluate(summary)
        print(tag_check.format_report(levels))
        return 0 if all(level.status == 'pass' for level in levels) else 1
    except KeyboardInterrupt:
        try:
            if checker.running.service_is_ready():
                checker.set_running(False, timeout=2.0)
        except (ExternalShutdownException, RuntimeError, KeyboardInterrupt):
            pass
        print('หยุดตรวจแล้ว (สั่งหุ่นหยุด)')
        return 130
    finally:
        checker.node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
