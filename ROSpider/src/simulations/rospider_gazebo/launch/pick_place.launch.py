import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Objects are spawned at run time rather than written into rospider_room.sdf:
# that world backs maps/rospider_room.* and the Nav2 demo's AMCL initial pose.
# Poses are in the world frame with the robot at the origin, so they are also
# base_footprint coordinates. See spec section 5.1.
SCENE = (
    ('pick_pedestal', (0.200, 0.0, 0.040)),
    ('pick_cube_red', (0.235, 0.07, 0.105)),
    ('pick_cube_green', (0.235, 0.0, 0.105)),
    ('pick_cube_blue', (0.235, -0.07, 0.105)),
    ('drop_marker', (0.160, 0.0, 0.082)),
)


def generate_launch_description():
    pkg = get_package_share_directory('rospider_gazebo')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'arm_pose': 'init',
        }.items(),
    )

    spawns = [
        Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{name}',
            output='screen',
            arguments=[
                '-file', os.path.join(pkg, 'models', name, 'model.sdf'),
                '-name', name,
                '-x', str(pose[0]), '-y', str(pose[1]), '-z', str(pose[2]),
            ],
        )
        for name, pose in SCENE
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg, 'worlds', 'rospider_room.sdf')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        gazebo,
        # The robot and its controllers need to exist before the cubes land on
        # the pedestal, or they drop through a world that is still loading.
        TimerAction(period=5.0, actions=spawns),
    ])
