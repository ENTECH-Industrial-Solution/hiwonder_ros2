"""`ros2 launch rospider_gazebo vslam_nav_challenge.launch.py map:=myvslam` -- navigating on V-SLAM.

Loads the participant's own map from the V-SLAM mapping exercise (<workspace>/maps/vslam/<map>.db;
vslam.launch.py localizes on a temp copy, so the file is never changed) and runs Nav2 with
Regulated Pure Pursuit on config/vslam_nav2_params.yaml, with the participant's values from
config/vslam_nav_challenge.yaml (deliberately wrong ones included) merged by
rospider_gazebo/vslam_params.py. Score with `ros2 run rospider_gazebo check_nav.py` (the Nav2
exercise's course). The robot must start where the map was started (the spawn point):
relocalization from elsewhere was not reliable.

Measured on the answer-key map (maps/vslam/vslam_answer.db, check_nav.py): starting file
(robot_radius 0.3, max_vel_x 0.05) G1 in 41 s, then no path through the doorway: level 2;
robot_radius 0.15: 41 + 178 s, then over the 300 s limit during G3: level 3; answer key
24 + 67 + 127 = 218 s and 24 + 68 + 112 = 203 s: pass. camera_view is a distractor: at the
floor it only stopped the robot at 0.15 m/s (probes), not at the starting 0.05 (41-42 s to G1).

  params  another participant file (default: config/vslam_nav_challenge.yaml)
  map     the V-SLAM map's name or a path to a .db
  gui, rviz  as vslam.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from rospider_gazebo import challenge_params, maps, vslam_params


def launch_setup(context):
    pkg = get_package_share_directory('rospider_gazebo')
    template = os.path.join(pkg, 'config', 'vslam_nav_challenge.yaml')
    user = os.path.expanduser(LaunchConfiguration('params').perform(context)) or template
    created = challenge_params.ensure_user_file(template, user)
    user = os.path.realpath(user)       # under --symlink-install: the source copy to edit
    value = LaunchConfiguration('map').perform(context)
    try:
        database = vslam_params.require_map(value, maps.workspace_maps_dir('vslam'), os.getcwd())
        merged, arm_pose = vslam_params.nav_params_file(
            os.path.join(pkg, 'config', 'vslam_nav2_params.yaml'), user)
    except challenge_params.ChallengeConfigError as err:
        raise RuntimeError(str(err)) from None
    note = ('สร้างไฟล์โจทย์ใหม่ที่' if created else 'ใช้ไฟล์โจทย์') + f' {user}'
    return [
        LogInfo(msg=f'[vslam_nav_challenge] {note} -- แก้ไฟล์นี้แล้ว launch ใหม่; แผนที่ {database}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'vslam.launch.py')),
            launch_arguments={
                'world': 'slam_challenge',
                'map': database,
                'localization': 'true',
                'params_file': merged,
                'arm_pose': arm_pose,
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
            }.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value='',
                              description="the participant's values "
                                          '(default: config/vslam_nav_challenge.yaml)'),
        DeclareLaunchArgument('map', default_value='myvslam'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
