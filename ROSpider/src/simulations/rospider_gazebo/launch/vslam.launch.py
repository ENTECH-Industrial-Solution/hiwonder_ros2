import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rospider_gazebo.maps import localization_copy, resolve_map

#: Brought up in this order by lifecycle_activate.py instead of nav2's lifecycle_manager, which
#: waits forever when a change_state reply is lost (seen in ~1 of 12 V-SLAM launches). Same list
#: as nav2_bringup/launch/navigation_launch.py's lifecycle_nodes.
NAV2_NODES = ('controller_server', 'smoother_server', 'planner_server', 'route_server',
              'behavior_server', 'velocity_smoother', 'collision_monitor', 'bt_navigator',
              'waypoint_follower', 'docking_server')


def launch_setup(context):
    """Camera-only visual SLAM with RTAB-Map: the RGB-D camera and /odom, never the LiDAR.

    Uses only V-SLAM's own files: config/vslam.yaml, config/vslam_nav2_params.yaml, rviz/vslam.rviz,
    and maps in <workspace>/maps/vslam. localization:=false builds a new 3D map; localization:=true
    loads a copy of it, relocalizes against it with the camera and drives Nav2 on it.
    """
    pkg = get_package_share_directory('rospider_gazebo')
    localization = LaunchConfiguration('localization').perform(context) == 'true'
    database = resolve_map(LaunchConfiguration('map').perform(context), '.db', subdir='vslam')
    vslam_params = (LaunchConfiguration('vslam_params').perform(context)
                    or os.path.join(pkg, 'config', 'vslam.yaml'))
    if localization:
        # rtabmap writes its 2D grid back into the database on shutdown, ghost walls included
        # (see maps.localization_copy): localize on a copy so the map file never changes.
        source = database
        database = localization_copy(database)

    # The camera must look ahead, like the real robot's init_horizontal action before RTAB-Map
    # (vslam_challenge sets init on purpose)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'arm_pose': LaunchConfiguration('arm_pose'),
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
        # map_always_update adds this run's views to /map at their (lagging) camera poses:
        # ghost walls in the planner's map. Mapping keeps it (map_saver_cli needs /map).
        overrides['map_always_update'] = False
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
                'autostart': 'false',
            }.items(),
        ))
        actions.append(Node(
            package='rospider_gazebo', executable='lifecycle_activate.py', output='screen',
            arguments=list(NAV2_NODES) + ['--timeout', '180'],
        ))
    if LaunchConfiguration('rviz').perform(context) == 'true':
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg, 'rviz', 'vslam.rviz')],
            parameters=[{'use_sim_time': True}],
        ))

    return [
        LogInfo(msg=f'V-SLAM map (loading a copy of {source}): {database}' if localization
                else f'V-SLAM map (new, saved on shutdown): {database}'),
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
        DeclareLaunchArgument('vslam_params', default_value='',
                              description='RTAB-Map parameters (default: config/vslam.yaml)'),
        DeclareLaunchArgument('arm_pose', default_value='horizontal', choices=['init', 'horizontal'],
                              description='horizontal: the camera looks ahead; init: tilted at the floor'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'vslam_nav2_params.yaml'),
                              description='Nav2 parameters (localization mode only)'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
