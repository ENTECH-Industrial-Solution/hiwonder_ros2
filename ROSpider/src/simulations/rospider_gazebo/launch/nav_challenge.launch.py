"""`ros2 launch rospider_gazebo nav_challenge.launch.py` -- the Nav2 exercise.

The SLAM exercise's room (worlds/slam_challenge.sdf) and its reference map
(maps/slam_challenge.yaml) with navigation.launch.py, on
config/nav2_params.yaml with the participant's six values from
config/nav_challenge.yaml written into every place they belong
(rospider_gazebo/nav_params.py). The participant edits that file in the
source tree and relaunches (needs `colcon build --symlink-install`);
`git checkout -- <file>` starts over. Score the run with
`ros2 run rospider_gazebo check_nav.py`.

  params  another participant file (default: config/nav_challenge.yaml);
          a path that does not exist yet gets a copy of the starting file.
          Instructors pass config/nav_challenge_solved.yaml's path.
  map     the participant's own map instead of the reference (a name in
          ROSpider/maps or ./maps, or a path)
  gui, rviz  as navigation.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from rospider_gazebo import challenge_params, nav_params


def launch_setup(context):
    pkg = get_package_share_directory('rospider_gazebo')
    template = os.path.join(pkg, 'config', 'nav_challenge.yaml')
    user = os.path.expanduser(LaunchConfiguration('params').perform(context)) or template
    created = challenge_params.ensure_user_file(template, user)
    user = os.path.realpath(user)       # under --symlink-install: the source copy to edit
    try:
        merged = nav_params.merged_params_file(os.path.join(pkg, 'config', 'nav2_params.yaml'), user)
    except challenge_params.ChallengeConfigError as err:
        raise RuntimeError(f'ไฟล์โจทย์ใช้ไม่ได้: {err}') from None
    map_value = (LaunchConfiguration('map').perform(context)
                 or os.path.join(pkg, 'maps', 'slam_challenge.yaml'))

    note = ('สร้างไฟล์โจทย์ใหม่ที่' if created else 'ใช้ไฟล์โจทย์') + f' {user}'
    if f'{os.sep}install{os.sep}' in user:
        note += (' (ไฟล์นี้อยู่ใน install: แก้ที่ src แล้วต้อง colcon build ใหม่ '
                 'หรือ build ด้วย --symlink-install)')
    return [
        LogInfo(msg=f'[nav_challenge] {note} -- แก้ไฟล์นี้แล้ว launch ใหม่'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'world': 'slam_challenge',
                'map': map_value,
                'params_file': merged,
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
            }.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value='',
                              description="the participant's Nav2 values "
                                          '(default: config/nav_challenge.yaml)'),
        DeclareLaunchArgument('map', default_value='',
                              description='your own map instead of the reference'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
