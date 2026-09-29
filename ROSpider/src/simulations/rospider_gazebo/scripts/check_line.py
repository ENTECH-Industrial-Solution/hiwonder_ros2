#!/usr/bin/env python3
"""ตรวจโจทย์ Line Following: สั่งหุ่นเกาะเส้นหนึ่งรอบ แล้วบอกผลทีละด่าน

    ros2 run rospider_gazebo check_line.py

Picks the line's colour through line_following's own ~/set_target_color (the point where the
line runs at the spawn pose), lets it follow, and feeds the odometry to
rospider_gazebo/line_check.py until the lap is done, the robot strays or stands still, or
time runs out; then stops the robot. Times are on the sim clock.
"""

import math
import sys
import time

import rclpy
from interfaces.srv import SetPoint
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import SetBool
from rospider_gazebo import line_check
from rospider_gazebo.line_track import start_pose

#: Where the line runs in the picture at the spawn pose, as a fraction of it (measured).
LINE_PIXEL = (0.45, 0.82)
#: ColorPicker averages 10 frames; give it this many sim seconds before timing starts.
PICK_SECONDS = 3.0
START_TOLERANCE_M = 0.15
WAIT = 60.0
#: No new /odom for this many wall seconds: the sim closed, crashed or is paused.
ODOM_SILENCE = 10.0
#: Whatever the sim's speed, give up after this many wall seconds.
WALL_LIMIT = line_check.RUN_LIMIT * 5


class OdomLost(Exception):
    """Odometry stopped arriving."""


class Checker:
    def __init__(self):
        self.node = rclpy.create_node(
            'check_line', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.pick = self.node.create_client(SetPoint, '/line_following/set_target_color')
        self.running = self.node.create_client(SetBool, '/line_following/set_running')
        self.odom = None
        self.odom_wall = None
        self.node.create_subscription(Odometry, '/odom', self._on_odom, 10)

    def _on_odom(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, stamp)
        self.odom_wall = time.monotonic()

    def call(self, client, request, timeout=5.0, tries=3):
        """The response, or None. Both services are idempotent, so a request whose reply was
        lost (Jazzy's rmw sometimes drops it) is simply sent again."""
        for _ in range(tries):
            future = client.call_async(request)
            end = time.monotonic() + timeout
            while not future.done() and time.monotonic() < end:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            if future.done():
                return future.result()
        return None

    def ready(self):
        end = time.monotonic() + WAIT
        while time.monotonic() < end and (self.odom is None or not self.pick.service_is_ready()
                                          or not self.running.service_is_ready()):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return (self.odom is not None and self.pick.service_is_ready()
                and self.running.service_is_ready())

    def set_running(self, on, timeout=5.0):
        request = SetBool.Request()
        request.data = on
        return self.call(self.running, request, timeout, tries=1 if not on else 3)

    def spin(self, wall_end):
        rclpy.spin_once(self.node, timeout_sec=0.1)
        now = time.monotonic()
        if now - self.odom_wall > ODOM_SILENCE or now > wall_end:
            raise OdomLost()

    def run(self):
        request = SetPoint.Request()
        request.data.x, request.data.y = LINE_PIXEL
        if self.call(self.pick, request) is None or self.set_running(True) is None:
            return None
        wall_end = time.monotonic() + WALL_LIMIT
        begin = self.odom[2]
        while self.odom[2] - begin < PICK_SECONDS:
            self.spin(wall_end)
        tracker = line_check.LapTracker()
        last = None
        while True:
            self.spin(wall_end)
            if self.odom is last:
                continue
            last = self.odom
            x, y, t = last
            tracker.feed(x, y, t)
            if tracker.done(t):
                break
        self.set_running(False)
        return tracker.summary()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    checker = Checker()
    print('ตรวจโจทย์ Line Following - ถ้าเพิ่งตรวจไปรอบหนึ่ง ให้ปิด-เปิด launch ใหม่ก่อน '
          'หุ่นจะได้เริ่มบนเส้นที่จุดเดิม')
    try:
        if not checker.ready():
            print('ไม่พบ line_following - เปิด ros2 launch rospider_gazebo line_challenge.launch.py '
                  'แล้วรอให้หน้าต่าง image ขึ้นก่อน')
            return 1
        x0, y0, _ = start_pose()
        x, y, _ = checker.odom
        if math.hypot(x - x0, y - y0) > START_TOLERANCE_M:
            print('หุ่นไม่ได้อยู่ที่จุดเริ่มบนเส้น - ปิด-เปิด launch ใหม่ก่อนตรวจ')
            return 1
        print('เลือกสีเส้นให้แล้ว หุ่นกำลังวิ่ง ...', flush=True)
        try:
            summary = checker.run()
        except OdomLost:
            checker.set_running(False, timeout=2.0)
            print('ไม่ได้รับตำแหน่งหุ่น (/odom) - Gazebo ปิด ค้าง หรือถูก pause อยู่ ปิด-เปิด launch ใหม่แล้วตรวจอีกครั้ง')
            return 1
        if summary is None:
            checker.set_running(False, timeout=2.0)     # a start request may have got through
            print('line_following ไม่ตอบคำสั่ง - ปิด-เปิด launch ใหม่แล้วตรวจอีกครั้ง')
            return 1
        levels = line_check.evaluate(summary)
        print(line_check.format_report(levels))
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
