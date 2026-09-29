#!/usr/bin/env python3
"""ตรวจโจทย์ Nav2: สั่งหุ่นวิ่งตามเส้นทาง แล้วบอกผลทีละด่าน

    ros2 run rospider_gazebo check_nav.py

Before each goal it asks the planner for a path, so a blocked doorway is
reported at once instead of after Nav2's recovery retries; then it sends the
goal and waits for the result. Times are on the sim clock, so a slow PC is
not penalised. It stops at the first failed goal. Scoring: rospider_gazebo/nav_check.py.
"""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rospider_gazebo import nav_check
from rospider_gazebo.nav_client import Nav2Client


def main():
    # rclpy's own SIGINT handler would shut the context down before our
    # KeyboardInterrupt handler could cancel the goal.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('check_nav', parameter_overrides=[Parameter('use_sim_time', value=True)])
    checker = Nav2Client(node)
    print('ตรวจโจทย์ Nav2 - ถ้าเพิ่งตรวจไปรอบหนึ่ง ให้ปิด-เปิด launch ใหม่ก่อน '
          'หุ่นจะได้เริ่มที่จุดเกิดและเวลาเทียบกันได้')
    try:
        if not checker.ready():
            print('ไม่พบ Nav2 - เปิด launch ของโจทย์ (nav_challenge หรือ vslam_nav_challenge) '
                  'แล้วรอให้ RViz ขึ้นแผนที่ก่อน')
            return 1
        if not checker.active():
            print('Nav2 ยังเปิดไม่ครบ - รอให้ RViz ขึ้นแผนที่ แล้วตรวจใหม่')
            return 1
        results = []
        for name, label, (x, y), timeout in nav_check.COURSE:
            print(f'{name} {label}: กำลังไป ({x:.1f}, {y:.1f}) ...', flush=True)
            spent = sum(r.seconds for r in results)
            result = checker.drive(name, x, y, timeout, nav_check.COURSE_LIMIT - spent)
            print(f'  -> {result.status} ({result.seconds:.0f} วินาที)', flush=True)
            results.append(result)
            if result.status != 'succeeded':
                break
        levels = nav_check.evaluate(results)
        print(nav_check.format_report(levels))
        return 0 if all(level.status == 'pass' for level in levels) else 1
    except KeyboardInterrupt:
        try:
            checker.cancel()
        except (ExternalShutdownException, RuntimeError):
            pass
        print('หยุดตรวจแล้ว (ยกเลิกเป้าหมายให้หุ่นหยุด)')
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
