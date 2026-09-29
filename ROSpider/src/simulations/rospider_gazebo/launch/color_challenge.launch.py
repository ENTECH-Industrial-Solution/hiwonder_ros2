"""`ros2 launch rospider_gazebo color_challenge.launch.py` -- the Color Threshold exercise (docs 6.1).

The scene worlds/color_challenge.sdf (coloured cubes, an orange look-alike, a blue cube in shadow)
and the LAB_Tool window (lab_tool.launch.py: color_detect with tune:=true) opened on
config/color_challenge.yaml, whose red, green and blue bands are deliberately wrong. Save in the
window writes ~/.ros/color_challenge_tuned.json, which overlays the file. Score with
`ros2 run rospider_gazebo check_color.py`. See
docs/superpowers/specs/2026-09-29-color-threshold-challenge-design.md.

  params  another participant file (default: config/color_challenge.yaml); a path that does not
          exist yet gets a copy of the starting file
  gui     as gazebo.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from rospider_gazebo import challenge_params

TUNED = '~/.ros/color_challenge_tuned.json'


def launch_setup(context):
    pkg = get_package_share_directory('rospider_gazebo')
    template = os.path.join(pkg, 'config', 'color_challenge.yaml')
    user = os.path.expanduser(LaunchConfiguration('params').perform(context)) or template
    created = challenge_params.ensure_user_file(template, user)
    user = os.path.realpath(user)       # under --symlink-install: the source copy to edit
    note = ('สร้างไฟล์โจทย์ใหม่ที่' if created else 'ใช้ไฟล์โจทย์') + f' {user}'
    tuned = os.path.expanduser(TUNED)
    if os.path.isfile(tuned):
        note += f' (ค่าที่กด Save ไว้ใน {tuned} จะทับค่าในไฟล์ - ลบไฟล์นั้นถ้าอยากเริ่มใหม่)'
    return [
        LogInfo(msg=f'[color_challenge] {note}; ตรวจด้วย ros2 run rospider_gazebo check_color.py'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': 'color_challenge', 'gui': LaunchConfiguration('gui')}.items()),
        # The window needs camera frames: start it once the sim is up.
        TimerAction(period=8.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'lab_tool.launch.py')),
            launch_arguments={'config': user, 'tuned_path': TUNED}.items())]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value='',
                              description="the participant's LAB bands (default: config/color_challenge.yaml)"),
        DeclareLaunchArgument('gui', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
