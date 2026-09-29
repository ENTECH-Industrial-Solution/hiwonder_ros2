"""`ros2 launch rospider_gazebo shape_challenge.launch.py` -- the 3D Shape Recognition exercise (docs 8.8).

worlds/shape_challenge.sdf (a sphere, a cuboid and a cylinder on the floor under the camera) and the
object_classification window run with config/shape_challenge.yaml's values (all three deliberately
wrong), which the participant edits in the source tree and relaunches. Score with
`ros2 run rospider_gazebo check_shape.py`. See
docs/superpowers/specs/2026-09-29-shape-recognition-challenge-design.md.

  params  another participant file (default: config/shape_challenge.yaml); a path that does not
          exist yet gets a copy of the starting file
  gui     as gazebo.launch.py
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from rospider_gazebo import challenge_params, shape_check

#: The keys the participant may set, with their types: object_classification's own defaults
#: (config/vision_demos.yaml).
DEFAULTS = {'plane_distance': 350.0, 'roi': [50, 350, 150, 500],
            'shapes': ['sphere', 'cuboid', 'cylinder']}


def launch_setup(context):
    pkg = get_package_share_directory('rospider_gazebo')
    template = os.path.join(pkg, 'config', 'shape_challenge.yaml')
    user = os.path.expanduser(LaunchConfiguration('params').perform(context)) or template
    created = challenge_params.ensure_user_file(template, user)
    user = os.path.realpath(user)       # under --symlink-install: the source copy to edit
    try:
        values = challenge_params.merge(DEFAULTS, challenge_params.load_params(user), user)
        shape_check.validate(values, user)
    except challenge_params.ChallengeConfigError as err:
        raise RuntimeError(f'ไฟล์โจทย์ใช้ไม่ได้: {err}') from None
    note = ('สร้างไฟล์โจทย์ใหม่ที่' if created else 'ใช้ไฟล์โจทย์') + f' {user}'
    # roi and shapes are lists, which a launch argument cannot carry: hand them over in a file.
    with tempfile.NamedTemporaryFile('w', prefix='shape_challenge_', suffix='.yaml',
                                     delete=False) as handle:
        yaml.safe_dump({'object_classification': {'ros__parameters': values}}, handle)
    return [
        LogInfo(msg=f'[shape_challenge] {note} -- แก้ไฟล์นี้แล้ว launch ใหม่; '
                    'ตรวจด้วย ros2 run rospider_gazebo check_shape.py'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': 'shape_challenge', 'gui': LaunchConfiguration('gui')}.items()),
        # The window's look pose is lost if it goes out before the controllers are up.
        TimerAction(period=8.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'vision_demo.launch.py')),
            launch_arguments={'demo': 'object_classification', 'debug': 'false',
                              'params_file': handle.name}.items())]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value='',
                              description="the participant's values (default: config/shape_challenge.yaml)"),
        DeclareLaunchArgument('gui', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
