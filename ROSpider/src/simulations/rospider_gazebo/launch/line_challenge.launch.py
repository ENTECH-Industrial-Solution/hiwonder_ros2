"""`ros2 launch rospider_gazebo line_challenge.launch.py` -- the Line Following exercise (docs 6.9).

line_following.launch.py (the loop on the floor, the robot on it, the `image` window) run with the
values in config/line_challenge.yaml (deliberately wrong ones included), which the participant edits
in the source tree and relaunches. Score with `ros2 run rospider_gazebo check_line.py`: it picks the
line's colour and lets the robot run one lap. See
docs/superpowers/specs/2026-09-29-line-following-challenge-design.md.

  params  another participant file (default: config/line_challenge.yaml); a path that does not
          exist yet gets a copy of the starting file
  gui     as gazebo.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from rospider_gazebo import challenge_params

#: The keys the participant may set, with their types: line_following's own defaults
#: (upstream's constants).
DEFAULTS = {'threshold': 0.5, 'kp': 1.1, 'speed': 0.05, 'max_turn': 0.35}


def launch_setup(context):
    pkg = get_package_share_directory('rospider_gazebo')
    template = os.path.join(pkg, 'config', 'line_challenge.yaml')
    user = os.path.expanduser(LaunchConfiguration('params').perform(context)) or template
    created = challenge_params.ensure_user_file(template, user)
    user = os.path.realpath(user)       # under --symlink-install: the source copy to edit
    try:
        values = challenge_params.merge(DEFAULTS, challenge_params.load_params(user), user)
    except challenge_params.ChallengeConfigError as err:
        raise RuntimeError(f'ไฟล์โจทย์ใช้ไม่ได้: {err}') from None
    note = ('สร้างไฟล์โจทย์ใหม่ที่' if created else 'ใช้ไฟล์โจทย์') + f' {user}'
    arguments = {key: str(value) for key, value in values.items()}
    # start false: a click in the window only picks a colour; the robot waits for
    # check_line.py's ~/set_running, so it starts on the spawn point every time.
    arguments.update({'start': 'false', 'gui': LaunchConfiguration('gui')})
    return [
        LogInfo(msg=f'[line_challenge] {note} -- แก้ไฟล์นี้แล้ว launch ใหม่; '
                    'ตรวจด้วย ros2 run rospider_gazebo check_line.py'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'line_following.launch.py')),
            launch_arguments=arguments.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value='',
                              description="the participant's values (default: config/line_challenge.yaml)"),
        DeclareLaunchArgument('gui', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
