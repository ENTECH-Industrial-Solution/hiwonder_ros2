"""`ros2 launch rospider_gazebo track_and_grab.launch.py` -- the sim's answer to
Hiwonder's `ros2 launch example track_and_grab.launch.py` (8.7 3D Vision:
Object Grasping).

Upstream's launch starts the camera, the controller and the node; this one
starts pick_place.launch.py (the simulator, the colour detector and
pick_and_place, held back with auto_start:=false so it waits for the
tracker) and then the track_and_grab window. Arguments for the window
(color:=, start:=, walk:=, auto_place:=, show:=, ...) pass through;
pick_place.launch.py's own (world:=, gui:=, ...) too.

layout:=spread (the default) lays the blocks out for the window's two
modes: green on the pick pedestal in front, where "pick here" reaches, and
red and blue each on a pedestal of its own 0.8 m off to either side,
where only "walk to it" gets them. layout:=near is pick_place.launch.py's
scene, all three in front.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

#: The spread layout: model file -> spawned name -> [x, y, z] in the world
#: frame, the robot at the origin facing +x. The pedestal top is 0.08 m, so
#: a block spawns with its centre at 0.105. Green keeps pick_place.yaml's
#: pedestal spot; red and blue sit square off to either side, so the walk
#: to them starts sideways and back, away from green's pedestal -- the
#: robot stands 1 cm from it and cannot turn in place there.
SPREAD = (
    ('pick_pedestal', 'pick_pedestal', (0.200, 0.0, 0.040)),
    ('pick_cube_green', 'pick_cube_green', (0.235, 0.0, 0.105)),
    ('pick_pedestal', 'pedestal_red', (0.0, 0.800, 0.040)),
    ('pick_cube_red', 'pick_cube_red', (0.0, 0.800, 0.105)),
    ('pick_pedestal', 'pedestal_blue', (0.0, -0.800, 0.040)),
    ('pick_cube_blue', 'pick_cube_blue', (0.0, -0.800, 0.105)),
)


def generate_launch_description():
    pkg = get_package_share_directory('rospider_gazebo')
    launch_dir = os.path.join(pkg, 'launch')
    spread = PythonExpression(["'", LaunchConfiguration('layout'), "' == 'spread'"])
    near = PythonExpression(["'", LaunchConfiguration('layout'), "' == 'near'"])
    scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'pick_place.launch.py')),
        launch_arguments={'auto_start': 'false', 'scene': near}.items())
    spawns = [
        Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{name}',
            output='screen',
            arguments=['-file', os.path.join(pkg, 'models', model, 'model.sdf'),
                       '-name', name,
                       '-x', str(pose[0]), '-y', str(pose[1]), '-z', str(pose[2])],
            condition=IfCondition(spread),
        )
        for model, name, pose in SPREAD
    ]
    window = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'vision_demo.launch.py')),
        # detector:=none, explicitly: pick_place.launch.py's `detector`
        # argument (color) would otherwise leak into the include and start
        # a second color_detect on the same topics.
        launch_arguments={'demo': 'track_and_grab', 'detector': 'none'}.items())
    return LaunchDescription([
        DeclareLaunchArgument('layout', default_value='spread', choices=['spread', 'near'],
                              description='spread: green in reach, red and blue off to '
                                          'the sides; near: all three in front'),
        scene,
        # The blocks land once the robot and its controllers exist, as
        # pick_place.launch.py times its own; the window waits for the
        # spawn burst, as the other demos do when they follow
        # gazebo.launch.py from a second terminal.
        TimerAction(period=5.0, actions=spawns),
        TimerAction(period=8.0, actions=[window]),
    ])
