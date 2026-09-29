#!/usr/bin/env python3
"""ตรวจโจทย์ Color Threshold: ดูภาพจากกล้องหนึ่งภาพ แล้วบอกผลทีละด่าน

    ros2 run rospider_gazebo check_color.py

Scores the bands the LAB_Tool window is using right now: color_detect publishes them on
/color_detect/settings (latched) at every slider move, saved or not. Without a running
color_detect it falls back to the files color_detect would load (config/color_challenge.yaml,
then the JSON LAB_Tool's Save writes). Takes one frame of worlds/color_challenge.sdf, refuses it
unless the answer-key bands pass on it (so a moved robot is not scored as a band mistake), and
scores it with rospider_gazebo/color_check.py.
"""

import argparse
import os
import sys
import time

import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rospider_gazebo import color_check

#: The first frames a new subscriber gets are black (the camera renders only while subscribed).
FRAMES = 8
WAIT = 30.0
SETTINGS_WAIT = 5.0
#: The path color_challenge.launch.py gives LAB_Tool's Save.
TUNED = '~/.ros/color_challenge_tuned.json'


def listen():
    """(frame or None, live settings JSON text or None)."""
    rclpy.init()
    node = rclpy.create_node('check_color')
    frames, live = [], []
    node.create_subscription(Image, '/depth_cam/rgb/image_raw', frames.append, 5)
    node.create_subscription(String, '/color_detect/settings', lambda m: live.append(m.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    start = time.monotonic()
    try:
        # Discovering color_detect's latched settings takes a moment; wait for them a while
        # after the frames are in (they are absent when no window is open).
        while time.monotonic() - start < WAIT and (
                len(frames) < FRAMES or (not live and time.monotonic() - start < SETTINGS_WAIT)):
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    frame = CvBridge().imgmsg_to_cv2(frames[-1], 'bgr8') if len(frames) >= FRAMES else None
    return frame, (live[-1] if live else None)


def default_params():
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('rospider_gazebo'), 'config',
                        'color_challenge.yaml')
    return os.path.realpath(path)       # under --symlink-install: the source copy


def settings_from_files(params, tuned):
    settings = color_check.load_settings(params, tuned)
    if os.path.isfile(os.path.expanduser(tuned)):
        note = f'ใช้ช่วงสีจาก {params} ทับด้วย {os.path.expanduser(tuned)} (ที่กด Save ไว้)'
    else:
        note = (f'ใช้ช่วงสีจาก {params} (หน้าต่าง LAB_Tool ไม่ได้เปิดอยู่ และยังไม่เคยกด Save '
                '- ค่าที่เลื่อน slider แต่ไม่ได้ Save จะไม่ถูกตรวจ)')
    return settings, note


def main(argv=None):
    parser = argparse.ArgumentParser(description='ตรวจโจทย์ Color Threshold')
    parser.add_argument('--params', default=None,
                        help='ไฟล์ช่วงสี ถ้าไม่มีหน้าต่าง LAB_Tool เปิดอยู่ (ค่าเริ่มต้น config/color_challenge.yaml)')
    parser.add_argument('--tuned', default=TUNED, help='ไฟล์ที่ LAB_Tool กด Save')
    args = parser.parse_args(argv)
    try:
        frame, live = listen()
    except KeyboardInterrupt:
        print('หยุดตรวจแล้ว')
        return 130
    try:
        if live is not None and args.params is None:
            settings = color_check.settings_from_json(live)
            note = ('ใช้ช่วงสีที่หน้าต่าง LAB_Tool ใช้อยู่ตอนนี้ (อย่าลืมกด Save '
                    'ไม่งั้นปิด launch แล้วค่าจะหาย)')
        else:
            settings, note = settings_from_files(args.params or default_params(), args.tuned)
    except (OSError, KeyError, TypeError, ValueError, AttributeError, yaml.YAMLError) as err:
        print(f'อ่านช่วงสีไม่ได้: {err}')
        return 1
    print(note)
    if frame is None:
        print('ไม่ได้ภาพจากกล้อง - เปิด ros2 launch rospider_gazebo color_challenge.launch.py ก่อน '
              'แล้วรอให้ Gazebo ขึ้น')
        return 1
    problem = color_check.frame_problem(frame) or color_check.scene_problem(frame)
    if problem:
        print(problem)
        return 1
    levels = color_check.evaluate(color_check.find_blobs(frame, settings))
    print(color_check.format_report(levels, color_check.ignored_colors(settings)))
    return 0 if all(level.status == 'pass' for level in levels) else 1


if __name__ == '__main__':
    sys.exit(main())
