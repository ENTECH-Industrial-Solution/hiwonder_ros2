#!/usr/bin/env python3
"""ตรวจแผนที่ของโจทย์ V-SLAM ว่าผ่านกี่ด่าน

    ros2 run rospider_gazebo check_vslam.py myvslam          # ROSpider/maps/vslam/myvslam.db
    ros2 run rospider_gazebo check_vslam.py ~/somewhere/m.db

Copies the database to a temp folder (rtabmap-export must not touch the participant's file),
exports its 2D grid with rtabmap-export, reads its graph, and scores both against
maps/vslam_challenge.yaml; see rospider_gazebo/vslam_check.py.
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile

from rospider_gazebo import maps, slam_check, vslam_check


def reference_path():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('rospider_gazebo'), 'maps',
                        'vslam_challenge.yaml')


def export_grid(db_path, work_dir, run=subprocess.run):
    """rtabmap-export's 2D grid of `db_path` as a map yaml in `work_dir`, or None (reason printed)."""
    try:
        run(['rtabmap-export', '--map', '--output', 'grid', '--output_dir', work_dir, db_path],
            check=True, capture_output=True, text=True, timeout=300)
    except FileNotFoundError:
        print('ไม่พบคำสั่ง rtabmap-export - ติดตั้ง ros-jazzy-rtabmap ก่อน')
        return None
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as err:
        print(f'rtabmap-export สร้างแผนที่จากไฟล์นี้ไม่ได้ ({maps.error_tail(err) or err})')
        return None
    path = os.path.join(work_dir, 'grid.yaml')
    if not os.path.isfile(path):
        print('rtabmap-export ไม่ได้สร้างแผนที่ 2D ออกมา (ในไฟล์อาจยังไม่มีข้อมูลแผนที่)')
        return None
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description='ตรวจแผนที่ของโจทย์ V-SLAM')
    parser.add_argument('map', help='ชื่อแผนที่ใน ROSpider/maps/vslam (เช่น myvslam) '
                                    'หรือ path ของไฟล์ .db')
    parser.add_argument('--reference', default=None, help='แผนที่อ้างอิง (สำหรับวิทยากร)')
    args = parser.parse_args(argv)

    if os.sep in args.map or args.map.startswith('~'):
        workspace_maps = os.path.dirname(os.path.abspath(os.path.expanduser(args.map)))
    else:
        workspace_maps = maps.workspace_maps_dir('vslam')
    path = maps.find_map(args.map, '.db', workspace_maps, os.getcwd())
    reference_file = args.reference or reference_path()
    if path is not None and vslam_check.db_in_use(path):
        print(f'แผนที่ {path} ยังเปิดอยู่ - ปิด launch ของโจทย์ก่อน (กด Ctrl+C แล้วรอจนปิดเสร็จ) '
              'แผนที่จะถูกบันทึกตอนปิด')
        return 1
    reference = slam_check.load_map(reference_file)
    learner, closures = None, None
    if path is None:
        print(f"ไม่พบแผนที่ '{args.map}' (หาใน {workspace_maps} และในโฟลเดอร์ปัจจุบัน)")
    else:
        print(f'ตรวจแผนที่ {path}')
        with tempfile.TemporaryDirectory(prefix='check_vslam_') as work:
            copy = os.path.join(work, 'map.db')
            shutil.copyfile(path, copy)
            grid = export_grid(copy, work)
            if grid is None:
                return 1
            try:
                learner = slam_check.load_map(grid)
                poses, links = vslam_check.read_graph(copy)
                closures = vslam_check.closure_errors(poses, links)
            except (slam_check.MapLoadError, vslam_check.sqlite3.Error) as err:
                print(f'อ่านแผนที่ไม่ได้: {err}')
                return 1
    levels = vslam_check.evaluate(learner, reference, closures, missing=path is None)
    print(vslam_check.format_report(levels))
    return 0 if all(level.status == 'pass' for level in levels) else 1


if __name__ == '__main__':
    sys.exit(main())
