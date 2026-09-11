import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rospider_gazebo.maps import resolve_map


def launch_setup(context):
    """Camera-only visual SLAM with RTAB-Map: the RGB-D camera and /odom, never the LiDAR.

    Uses only V-SLAM's own files: config/vslam.yaml, config/vslam_nav2_params.yaml, rviz/vslam.rviz,
    and maps in <workspace>/maps/vslam. localization:=false builds a new 3D map; localization:=true
    loads it, relocalizes against it with the camera and drives Nav2 on it.
    """
    pkg = get_package_share_directory('rospider_gazebo')
    localization = LaunchConfiguration('localization').perform(context) == 'true'
    database = resolve_map(LaunchConfiguration('map').perform(context), '.db', subdir='vslam')
    vslam_params = os.path.join(pkg, 'config', 'vslam.yaml')

    # The camera must look ahead, like the real robot's init_horizontal action before RTAB-Map
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'arm_pose': 'horizontal',
        }.items(),
    )

    remappings = [
        ('rgb/image', '/depth_cam/rgb/image_raw'),
        ('rgb/camera_info', '/depth_cam/rgb/camera_info'),
        ('depth/image', '/depth_cam/depth/image_raw'),
        ('odom', '/odom'),
    ]

    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[vslam_params, {'use_sim_time': True}],
        remappings=remappings,
    )

    overrides = {'use_sim_time': True, 'database_path': database}
    if localization:
        # Load the whole saved map and only localize in it: no new nodes are added
        overrides.update({'Mem/IncrementalMemory': 'false', 'Mem/InitWMWithAllNodes': 'true'})
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[vslam_params, overrides],
        remappings=remappings,
        # -d deletes the database and starts a new map; never pass it when loading a map
        arguments=[] if localization else ['-d'],
    )

    actions = [rgbd_sync, rtabmap]
    if localization:
        # Obstacles for Nav2: a light cloud built from the depth image (see config/vslam.yaml)
        actions.append(Node(
            package='rtabmap_util',
            executable='point_cloud_xyz',
            name='point_cloud_xyz',
            output='screen',
            parameters=[vslam_params, {'use_sim_time': True}],
            remappings=[
                ('depth/image', '/depth_cam/depth/image_raw'),
                ('depth/camera_info', '/depth_cam/depth/camera_info'),
                ('cloud', '/vslam/obstacle_cloud'),
            ],
        ))
        # Nav2 without map_server/AMCL: RTAB-Map provides the map and the localization
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': 'true',
                'autostart': 'true',
            }.items(),
        ))
    if LaunchConfiguration('rviz').perform(context) == 'true':
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg, 'rviz', 'vslam.rviz')],
            parameters=[{'use_sim_time': True}],
        ))

    return [
        LogInfo(msg=f"V-SLAM map ({'loading' if localization else 'new, saved on shutdown'}): {database}"),
        gazebo,
        # Start once the sim clock, TF and camera are running
        TimerAction(period=8.0, actions=actions),
    ]


def generate_launch_description():
    pkg = get_package_share_directory('rospider_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'rospider_room.sdf')),
        DeclareLaunchArgument('localization', default_value='false', choices=['true', 'false'],
                              description='false: build a new map (overwrites it); true: load it and navigate with Nav2'),
        DeclareLaunchArgument('map', default_value='map',
                              description='Map name, kept as <workspace>/maps/vslam/<name>.db, or a path to a .db file'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'vslam_nav2_params.yaml'),
                              description='Nav2 parameters (localization mode only)'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
