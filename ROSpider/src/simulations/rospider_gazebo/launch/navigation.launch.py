import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rospider_gazebo.maps import resolve_map


def launch_setup(context):
    pkg = get_package_share_directory('rospider_gazebo')
    map_yaml = resolve_map(LaunchConfiguration('map').perform(context), '.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world'), 'gui': LaunchConfiguration('gui')}.items(),
    )

    # Stock Jazzy nav2_bringup: Hiwonder's navigation package targets Humble (plugin names, TEB,
    # controller params without use_sim_time). ROSpider's tuning lives in config/nav2_params.yaml.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('navigation'), 'rviz', 'navigation.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [
        LogInfo(msg=f'Navigation map: {map_yaml}'),
        gazebo,
        # RViz starts with Nav2 so its TF buffer does not begin before the sim clock runs
        TimerAction(period=5.0, actions=[nav2, rviz]),
    ]


def generate_launch_description():
    pkg = get_package_share_directory('rospider_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'rospider_room.sdf')),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'rospider_room.yaml'),
                              description='2D map: a name in <workspace>/maps (<name>.yaml) or a path to a map .yaml'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
