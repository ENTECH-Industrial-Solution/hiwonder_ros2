#!/usr/bin/env python3
"""ตรวจโจทย์ 3D Shape Recognition: ดูว่าหุ่นแยกวัตถุ เรียกชื่อรูปทรง และเลือกชิ้นที่โจทย์ต้องการได้ไหม

    ros2 run rospider_gazebo check_shape.py

Reads what object_classification reports on ~/objects (it recognises every camera frame; the scene
is static, so one report a couple of seconds after the arm has reached its look pose is the
answer) and scores it with rospider_gazebo/shape_check.py. Refuses when the robot is not at the
spawn: the expected boxes are pixels of that one view.
"""

import json
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.signals import SignalHandlerOptions
from rospider_gazebo import shape_check
from std_msgs.msg import String

WAIT = 60.0
#: Reports are taken this long after the first, so the arm has settled in its look pose.
SETTLE = 3.0
START_TOLERANCE_M = 0.05
START_TOLERANCE_DEG = 5.0


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('check_shape')
    reports = []
    odom = {}

    def on_odom(msg):
        q = msg.pose.pose.orientation
        odom.update(x=msg.pose.pose.position.x, y=msg.pose.pose.position.y,
                    yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    node.create_subscription(Odometry, '/odom', on_odom, 10)
    node.create_subscription(String, '/object_classification/objects',
                             lambda msg: reports.append((time.monotonic(), msg.data)), 10)
    print('ตรวจโจทย์ 3D Shape Recognition ...', flush=True)
    try:
        end = time.monotonic() + WAIT
        while time.monotonic() < end and not (reports and odom
                                               and time.monotonic() - reports[0][0] > SETTLE):
            rclpy.spin_once(node, timeout_sec=0.1)
        if not reports:
            print('ไม่พบ object_classification - เปิด ros2 launch rospider_gazebo '
                  'shape_challenge.launch.py แล้วรอให้หน้าต่าง depth ขึ้นก่อน')
            return 1
        if not odom:
            print('ไม่ได้รับตำแหน่งหุ่น (/odom) - Gazebo ปิด ค้าง หรือถูก pause อยู่')
            return 1
        if (math.hypot(odom['x'], odom['y']) > START_TOLERANCE_M
                or abs(math.degrees(odom['yaw'])) > START_TOLERANCE_DEG):
            print('หุ่นไม่ได้อยู่ที่จุดเริ่ม (กล้องเลยไม่ได้มองวัตถุจากมุมที่โจทย์กำหนด) '
                  '- ปิด-เปิด launch ใหม่ก่อนตรวจ')
            return 1
        report = json.loads(reports[-1][1])
        print(f'หุ่นใช้ค่า plane_distance {report["plane_distance"]:g}, roi {report["roi"]}, '
              f'shapes {report["shapes"]} (ถ้าไม่ตรงกับที่แก้ ให้ปิด-เปิด launch ใหม่)')
        levels = shape_check.evaluate(report)
        print(shape_check.format_report(levels))
        return 0 if all(level.status == 'pass' for level in levels) else 1
    except KeyboardInterrupt:
        print('หยุดตรวจแล้ว')
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
