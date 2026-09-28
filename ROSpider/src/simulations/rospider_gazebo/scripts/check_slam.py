#!/usr/bin/env python3
"""ตรวจแผนที่ของโจทย์ SLAM Mapping ว่าผ่านกี่ด่าน

    ros2 run rospider_gazebo check_slam.py room1              # ROSpider/maps/room1.yaml
    ros2 run rospider_gazebo check_slam.py ~/somewhere/m.yaml

Compares the map with maps/slam_challenge.yaml (the reference saved with
config/slam_challenge_solved.yaml); see rospider_gazebo/slam_check.py.
"""

import argparse
import os
import sys

from ament_index_python.packages import get_package_share_directory
from rospider_gazebo import slam_check
from rospider_gazebo import maps


def main():
    parser = argparse.ArgumentParser(description='ตรวจแผนที่ของโจทย์ SLAM Mapping')
    parser.add_argument('map', help='ชื่อแผนที่ใน ROSpider/maps (เช่น room1) '
                                    'หรือ path ของไฟล์ .yaml')
    parser.add_argument('--reference', default=os.path.join(
        get_package_share_directory('rospider_gazebo'), 'maps', 'slam_challenge.yaml'),
        help='แผนที่อ้างอิง (สำหรับวิทยากร)')
    args = parser.parse_args()

    reference = slam_check.load_map(args.reference)
    workspace_maps = maps.workspace_maps_dir()
    path = maps.find_map(args.map, '.yaml', workspace_maps, os.getcwd())
    learner = None
    if path is None:
        print(f"ไม่พบแผนที่ '{args.map}' (หาใน {workspace_maps} และในโฟลเดอร์ปัจจุบัน)")
    else:
        print(f'ตรวจแผนที่ {path}')
        try:
            learner = slam_check.load_map(path)
        except slam_check.MapLoadError as err:
            print(f'อ่านแผนที่ไม่ได้: {err}')
    levels = slam_check.evaluate(learner, reference, missing=path is None)
    print(slam_check.format_report(levels))
    return 0 if all(level.status == 'pass' for level in levels) else 1


if __name__ == '__main__':
    sys.exit(main())
