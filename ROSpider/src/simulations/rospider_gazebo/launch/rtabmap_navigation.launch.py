import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from rospider_gazebo.maps import resolve_map


def launch_setup(context):
    pkg = get_package_share_directory('rospider_gazebo')
    nav_pkg = get_package_share_directory('navigation')
    database = resolve_map(LaunchConfiguration('map').perform(context), '.db')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'arm_pose': 'horizontal',
        }.items(),
    )

    # Hiwonder's RTAB-Map localization setup (Mem/IncrementalMemory false): loads the map, adds no new
    # nodes, relocalises with the camera and publishes /map and map->odom.
    rtabmap = GroupAction([
        SetParameter(name='database_path', value=database),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_pkg, 'launch', 'include', 'rtabmap.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
    ])

    # Nav2 without map_server/AMCL: RTAB-Map provides both the map and the localization
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(nav_pkg, 'rviz', 'rtabmap.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [
        LogInfo(msg=f'RTAB-Map map (loading): {database}'),
        gazebo,
        TimerAction(period=8.0, actions=[rtabmap, nav2, rviz]),
    ]


def generate_launch_description():
    pkg = get_package_share_directory('rospider_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'rospider_room.sdf')),
        DeclareLaunchArgument('map', default_value='rtabmap',
                              description='Map made with rtabmap_slam.launch.py: a name in <workspace>/maps '
                                          'or a path to a .db file'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
