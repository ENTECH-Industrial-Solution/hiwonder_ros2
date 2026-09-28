"""`ros2 launch rospider_gazebo slam_challenge.launch.py` -- the SLAM Mapping exercise.

The two-room world (worlds/slam_challenge.sdf) with slam.launch.py, run on
config/slam_challenge.yaml (three values deliberately wrong). The
participant edits that file in the source tree and relaunches; with
`colcon build --symlink-install` the installed copy is a symlink to it, so
no rebuild is needed. `git checkout -- <file>` starts the exercise over.
Every run merges it over Hiwonder's slam/config/slam.yaml
(rospider_gazebo/challenge_params.py). Score the saved map with
`ros2 run rospider_gazebo check_slam.py <name>`.

  params  another participant file (default: config/slam_challenge.yaml);
          a path that does not exist yet gets a copy of the starting file.
          Instructors pass config/slam_challenge_solved.yaml's path.
  gui, rviz  as slam.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from rospider_gazebo import challenge_params

#: Set over Hiwonder's slam.yaml, under the participant's file. The sim's
#: odometry has no drift, so loop closure can only add false corrections:
#: measured on the exercise route, it shifted the map 0.19 m along the
#: room-A partition; without it the whole route stayed within 0.034 m.
FIXED = {'do_loop_closing': False}


def launch_setup(context):
    pkg = get_package_share_directory('rospider_gazebo')
    template = os.path.join(pkg, 'config', 'slam_challenge.yaml')
    user = os.path.expanduser(LaunchConfiguration('params').perform(context)) or template
    created = challenge_params.ensure_user_file(template, user)
    # Show the file to edit: under --symlink-install that is the source copy.
    user = os.path.realpath(user)
    base = os.path.join(get_package_share_directory('slam'), 'config', 'slam.yaml')
    try:
        merged = challenge_params.merged_params_file(base, user, FIXED)
    except challenge_params.ChallengeConfigError as err:
        raise RuntimeError(f'ไฟล์โจทย์ใช้ไม่ได้: {err}') from None

    note = ('สร้างไฟล์โจทย์ใหม่ที่' if created else 'ใช้ไฟล์โจทย์') + f' {user}'
    if f'{os.sep}install{os.sep}' in user:
        note += (' (ไฟล์นี้อยู่ใน install: แก้ที่ src แล้วต้อง colcon build ใหม่ '
                 'หรือ build ด้วย --symlink-install)')
    return [
        LogInfo(msg=f'[slam_challenge] {note} -- แก้ไฟล์นี้แล้ว launch ใหม่'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'slam.launch.py')),
            launch_arguments={
                'world': os.path.join(pkg, 'worlds', 'slam_challenge.sdf'),
                'params_file': merged,
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
            }.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value='',
                              description="the participant's slam_toolbox values "
                                          '(default: config/slam_challenge.yaml)'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
