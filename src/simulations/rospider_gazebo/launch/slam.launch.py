import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('rospider_gazebo')
    slam_pkg = get_package_share_directory('slam')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world'), 'gui': LaunchConfiguration('gui')}.items(),
    )

    # Jazzy's slam_toolbox is a lifecycle node, which Hiwonder's Humble slam_base.launch.py never
    # activates; use slam_toolbox's own launch with Hiwonder's parameters (slam/config/slam.yaml).
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_sync_launch.py')),
        launch_arguments={
            'slam_params_file': os.path.join(slam_pkg, 'config', 'slam.yaml'),
            'use_sim_time': 'true',
            'use_lifecycle_manager': 'true',
        }.items(),
    )

    # The launch file's own autostart sometimes loses the configure response and never activates
    # the node; the Nav2 lifecycle manager waits for the services and retries.
    slam_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{'node_names': ['slam_toolbox'], 'autostart': True, 'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(slam_pkg, 'rviz', 'slam.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'rospider_room.sdf')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        gazebo,
        # RViz starts with the rest so its TF buffer does not begin before the sim clock runs
        TimerAction(period=5.0, actions=[slam, slam_lifecycle_manager, rviz]),
    ])
