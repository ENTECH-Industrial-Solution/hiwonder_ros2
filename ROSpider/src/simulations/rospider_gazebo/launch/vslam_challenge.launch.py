"""`ros2 launch rospider_gazebo vslam_challenge.launch.py map:=myvslam` -- the V-SLAM mapping exercise.

The maze room (worlds/slam_challenge.sdf) with vslam.launch.py (camera only), run on
config/vslam_challenge.yaml (three values deliberately wrong), edited in the source tree and
merged over config/vslam.yaml by rospider_gazebo/vslam_params.py. Drive through both rooms
and back to the start, close the launch (Ctrl+C: RTAB-Map writes the database), then
`ros2 run rospider_gazebo check_vslam.py <map>`.

  params  another participant file (default: config/vslam_challenge.yaml); instructors pass
          config/vslam_challenge_solved.yaml's path
  map     the new map's name, kept as <workspace>/maps/vslam/<map>.db (overwritten)
  gui, rviz  as vslam.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from rospider_gazebo import challenge_params, vslam_params


def launch_setup(context):
    pkg = get_package_share_directory('rospider_gazebo')
    template = os.path.join(pkg, 'config', 'vslam_challenge.yaml')
    user = os.path.expanduser(LaunchConfiguration('params').perform(context)) or template
    created = challenge_params.ensure_user_file(template, user)
    user = os.path.realpath(user)       # under --symlink-install: the source copy to edit
    try:
        merged, arm_pose = vslam_params.mapping_params_file(
            os.path.join(pkg, 'config', 'vslam.yaml'), user)
    except challenge_params.ChallengeConfigError as err:
        raise RuntimeError(f'ไฟล์โจทย์ใช้ไม่ได้: {err}') from None
    note = ('สร้างไฟล์โจทย์ใหม่ที่' if created else 'ใช้ไฟล์โจทย์') + f' {user}'
    return [
        LogInfo(msg=f'[vslam_challenge] {note} -- แก้ไฟล์นี้แล้ว launch ใหม่; '
                    'ขับให้ทั่วแล้วกลับจุดเริ่ม จากนั้นปิด launch (Ctrl+C) ก่อนตรวจ'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'vslam.launch.py')),
            launch_arguments={
                'world': 'slam_challenge',
                'map': LaunchConfiguration('map'),
                'vslam_params': merged,
                'arm_pose': arm_pose,
                'localization': 'false',
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
            }.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value='',
                              description="the participant's values "
                                          '(default: config/vslam_challenge.yaml)'),
        DeclareLaunchArgument('map', default_value='myvslam'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
