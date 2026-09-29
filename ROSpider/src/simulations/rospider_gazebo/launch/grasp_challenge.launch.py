"""`ros2 launch rospider_gazebo grasp_challenge.launch.py` -- the 3D Object Grasping exercise (docs 8.7).

track_and_grab.launch.py's spread layout (the simulator, the blocks on their pedestals, the colour
detector, pick_and_place and the track_and_grab window) run with config/grasp_challenge.yaml's
values (all three deliberately wrong), which the participant edits in the source tree and
relaunches, plus a yellow pad on the floor where the blue block must be put down. Score with
`ros2 run rospider_gazebo check_grasp.py`: it orders the blue block and reads where it ends up. See
docs/superpowers/specs/2026-09-29-grasp-challenge-design.md.

  params  another participant file (default: config/grasp_challenge.yaml); a path that does not
          exist yet gets a copy of the starting file
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rospider_gazebo import challenge_params, grasp_check

#: The keys the participant may set, with their types: track_and_grab's own defaults
#: (config/vision_demos.yaml).
DEFAULTS = {'walk': False, 'target_x': 0.235, 'place_point': '0.02 0.16 0.035'}

#: The target pad: visual only, so neither the block nor the robot's feet catch on it.
PAD_SDF = f'''<sdf version="1.9"><model name="grasp_pad"><static>true</static>
<link name="link"><visual name="visual">
<geometry><box><size>{grasp_check.PAD_SIZE} {grasp_check.PAD_SIZE} 0.002</size></box></geometry>
<material><ambient>0.95 0.85 0.1 1</ambient><diffuse>0.95 0.85 0.1 1</diffuse></material>
</visual></link></model></sdf>'''


def launch_setup(context):
    pkg = get_package_share_directory('rospider_gazebo')
    template = os.path.join(pkg, 'config', 'grasp_challenge.yaml')
    user = os.path.expanduser(LaunchConfiguration('params').perform(context)) or template
    created = challenge_params.ensure_user_file(template, user)
    user = os.path.realpath(user)       # under --symlink-install: the source copy to edit
    try:
        values = challenge_params.merge(DEFAULTS, challenge_params.load_params(user), user)
        grasp_check.parse_point(values['place_point'], user)
    except challenge_params.ChallengeConfigError as err:
        raise RuntimeError(f'ไฟล์โจทย์ใช้ไม่ได้: {err}') from None
    note = ('สร้างไฟล์โจทย์ใหม่ที่' if created else 'ใช้ไฟล์โจทย์') + f' {user}'
    arguments = {key: str(value).lower() if isinstance(value, bool) else str(value)
                 for key, value in values.items()}
    arguments['layout'] = 'spread'
    return [
        LogInfo(msg=f'[grasp_challenge] {note} -- แก้ไฟล์นี้แล้ว launch ใหม่; '
                    'ตรวจด้วย ros2 run rospider_gazebo check_grasp.py'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'track_and_grab.launch.py')),
            launch_arguments=arguments.items()),
        TimerAction(period=5.0, actions=[Node(
            package='ros_gz_sim', executable='create', name='spawn_grasp_pad', output='screen',
            arguments=['-string', PAD_SDF, '-name', 'grasp_pad',
                       '-x', str(grasp_check.PAD[0]), '-y', str(grasp_check.PAD[1]), '-z', '0.001'])]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value='',
                              description="the participant's values (default: config/grasp_challenge.yaml)"),
        OpaqueFunction(function=launch_setup),
    ])
