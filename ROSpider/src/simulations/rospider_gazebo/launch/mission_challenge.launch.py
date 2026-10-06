"""`ros2 launch rospider_gazebo mission_challenge.launch.py` -- the final mission.

The maze room with two tag stations and a yellow pad (worlds/mission_challenge.sdf), Nav2 on the
reference map with the participant's own Nav2 exercise file (config/nav_challenge.yaml: the answer
keys are not in the repo, so the mission builds on the Nav2 exercise), and every node the mission's steps drive:
color_detect, apriltag_detect, apriltag_track (off until a go_to_tag step), pick_and_place and the
track_and_grab window. The participant's config/mission_challenge.yaml is read by
`ros2 run rospider_gazebo check_mission.py` at every run, not here; relaunch only to put the robot
and the blocks back. See docs/superpowers/specs/2026-09-29-mission-challenge-design.md.

  map        your own map from the SLAM exercise (a name in ROSpider/maps, or a path);
             default: the reference map maps/slam_challenge
  nav_params the Nav2 exercise file to use (default: config/nav_challenge.yaml);
             instructors pass their nav_challenge_solved.yaml's path
  gui, rviz  as navigation.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rospider_gazebo import challenge_params, mission_check, nav_params


def _demo(pkg, arguments):
    """vision_demo.launch.py in a scope of its own: launch arguments leak between includes."""
    return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'vision_demo.launch.py')),
        launch_arguments=arguments.items())])


def launch_setup(context):
    pkg = get_package_share_directory('rospider_gazebo')
    nav_file = os.path.realpath(os.path.expanduser(LaunchConfiguration('nav_params').perform(context))
                                or os.path.join(pkg, 'config', 'nav_challenge.yaml'))
    try:
        merged = nav_params.merged_params_file(os.path.join(pkg, 'config', 'nav2_params.yaml'), nav_file)
    except (OSError, challenge_params.ChallengeConfigError) as err:
        raise RuntimeError(f'ไฟล์ค่า Nav2 ใช้ไม่ได้: {err}') from None
    # The participant's own SLAM map, or the reference (navigation.launch.py resolves a name).
    map_value = (LaunchConfiguration('map').perform(context)
                 or os.path.join(pkg, 'maps', 'slam_challenge.yaml'))
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'navigation.launch.py')),
        launch_arguments={'world': 'mission_challenge',
                          'map': map_value,
                          'params_file': merged,
                          'gui': LaunchConfiguration('gui'),
                          'rviz': LaunchConfiguration('rviz')}.items())
    blocks = []
    for color in mission_check.BLOCK_OF_TAG.values():
        x, y, z = mission_check.block_spawn(color)
        blocks.append(Node(
            package='ros_gz_sim', executable='create', name=f'spawn_{color}', output='screen',
            arguments=['-file', os.path.join(pkg, 'models', f'pick_cube_{color}', 'model.sdf'),
                       '-name', f'pick_cube_{color}', '-x', str(x), '-y', str(y), '-z', str(z)]))
    # Gazebo welds each block to the hand the moment it spawns. pick_and_place releases the welds at
    # start-up, but only until the arm's joints show up in /joint_states: with Nav2 loading, the joints
    # were there before the blocks' weld subscribers existed, and the look pose then swung both blocks
    # 3 m through the air (measured). So release them from here too, for 5 s after the spawns.
    detach = ' '.join(f'gz topic -t /grasp/{c}/detach -m gz.msgs.Empty -p " ";'
                      for c in mission_check.BLOCK_OF_TAG.values())
    release = ExecuteProcess(cmd=['bash', '-c', f'for i in $(seq 10); do {detach} sleep 0.5; done'],
                             output='log')
    sim_time = {'use_sim_time': True}
    detectors = [
        Node(package='rospider_gazebo', executable='color_detect.py', name='color_detect',
             output='screen',
             parameters=[os.path.join(pkg, 'config', 'color_detect.yaml'), sim_time]),
        Node(package='rospider_gazebo', executable='apriltag_detect.py', name='apriltag_detect',
             output='screen',
             parameters=[os.path.join(pkg, 'config', 'apriltag.yaml'), sim_time]),
        # The blocks weld to the hand the instant they spawn; pick_and_place's start-up
        # detaches must still be firing then, so it starts in the same timer, after them.
        Node(package='rospider_gazebo', executable='pick_and_place.py', name='pick_and_place',
             output='screen',
             parameters=[os.path.join(pkg, 'config', 'pick_place.yaml'),
                         dict(sim_time, auto_start=False)]),
    ]
    windows = [
        _demo(pkg, {'demo': 'apriltag_track', 'detector': 'none', 'start': 'false', 'show': 'false',
                    'target_tag': str(mission_check.TARGET_TAG),
                    'stop_distance': str(mission_check.STOP_DISTANCE),
                    'speed_limit': '0.05', 'turn_limit': '0.2'}),
        _demo(pkg, {'demo': 'track_and_grab', 'detector': 'none', 'walk': 'false',
                    'auto_place': 'false'}),
    ]
    return [LogInfo(msg=f'[mission_challenge] ค่า Nav2 จาก {nav_file} (ไฟล์โจทย์ Nav2)'),
            navigation,
            TimerAction(period=5.0, actions=blocks + detectors),
            TimerAction(period=7.0, actions=[release]),
            TimerAction(period=10.0, actions=windows)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='',
                              description='your own map from the SLAM exercise (default: the reference)'),
        DeclareLaunchArgument('nav_params', default_value='',
                              description='Nav2 exercise file (default: config/nav_challenge.yaml)'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
