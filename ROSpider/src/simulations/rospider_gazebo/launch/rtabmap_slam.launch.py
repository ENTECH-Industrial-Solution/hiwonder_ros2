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
    slam_pkg = get_package_share_directory('slam')
    database = resolve_map(LaunchConfiguration('map').perform(context), '.db')

    # The camera must look ahead, like the real robot's init_horizontal action before RTAB-Map
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'arm_pose': 'horizontal',
        }.items(),
    )

    # Hiwonder's RTAB-Map setup: RGB-D + LiDAR, odometry from /odom. Its rtabmap node is started with
    # -d, so the database is recreated every run and saved when the node shuts down.
    rtabmap = GroupAction([
        SetParameter(name='database_path', value=database),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_pkg, 'launch', 'include', 'rtabmap.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(slam_pkg, 'rviz', 'rtabmap.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [
        LogInfo(msg=f'RTAB-Map map (new, saved on shutdown): {database}'),
        gazebo,
        # Start once the sim clock, TF and camera are running (Hiwonder waits 10 s on the robot)
        TimerAction(period=8.0, actions=[rtabmap, rviz]),
    ]


def generate_launch_description():
    pkg = get_package_share_directory('rospider_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'rospider_room.sdf')),
        DeclareLaunchArgument('map', default_value='rtabmap',
                              description='Map name, kept as <workspace>/maps/<name>.db (overwritten every run), '
                                          'or a path to a .db file'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
