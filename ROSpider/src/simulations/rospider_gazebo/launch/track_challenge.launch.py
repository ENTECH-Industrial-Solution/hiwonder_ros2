"""`ros2 launch rospider_gazebo track_challenge.launch.py` -- the Color Tracking exercise (docs 6.6).

worlds/track_challenge.sdf (a red ball to walk to, an orange one to leave alone) and the
object_tracking window run with config/track_challenge.yaml's values (three deliberately wrong),
which the participant edits in the source tree and relaunches. The tracker starts idle
(start: false: a click only picks a colour); `ros2 run rospider_gazebo check_track.py` picks the
red ball's colour, starts the walk and scores where the robot stops. See
docs/superpowers/specs/2026-09-29-color-tracking-challenge-design.md.

  params  another participant file (default: config/track_challenge.yaml); a path that does not
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

#: The keys the participant may set, with their types: object_tracking's own defaults
#: (config/vision_demos.yaml, upstream's constants).
DEFAULTS = {'threshold': 0.5, 'stop_y': 300, 'max_speed': 0.05, 'max_turn': 0.1}


def launch_setup(context):
    pkg = get_package_share_directory('rospider_gazebo')
    template = os.path.join(pkg, 'config', 'track_challenge.yaml')
    user = os.path.expanduser(LaunchConfiguration('params').perform(context)) or template
    created = challenge_params.ensure_user_file(template, user)
    user = os.path.realpath(user)       # under --symlink-install: the source copy to edit
    try:
        values = challenge_params.merge(DEFAULTS, challenge_params.load_params(user), user)
    except challenge_params.ChallengeConfigError as err:
        raise RuntimeError(f'ไฟล์โจทย์ใช้ไม่ได้: {err}') from None
    note = ('สร้างไฟล์โจทย์ใหม่ที่' if created else 'ใช้ไฟล์โจทย์') + f' {user}'
    arguments = {key: str(value) for key, value in values.items()}
    arguments['start'] = 'false'
    return [
        LogInfo(msg=f'[track_challenge] {note} -- แก้ไฟล์นี้แล้ว launch ใหม่; '
                    'ตรวจด้วย ros2 run rospider_gazebo check_track.py'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': 'track_challenge', 'gui': LaunchConfiguration('gui')}.items()),
        # The tracker's look pose is lost if it goes out before the controllers are up.
        TimerAction(period=8.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'object_tracking.launch.py')),
            launch_arguments=arguments.items())]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value='',
                              description="the participant's values (default: config/track_challenge.yaml)"),
        DeclareLaunchArgument('gui', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
