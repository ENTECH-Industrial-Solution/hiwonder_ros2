import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    pkg = get_package_share_directory('rospider_gazebo')
    world = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context) == 'true'

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ('-r ' if gui else '-r -s --headless-rendering ') + world,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # The robot's collision geometry is the full-detail STL meshes (~1.5M triangles). With the gait
    # not simulated they add nothing but physics cost -- the sim ran at ~2% real time with them --
    # so only primitive collisions (the skid in rospider_gazebo.urdf.xacro) are kept.
    # Sensors render visuals, so they are unaffected.
    doc = xacro.process_file(os.path.join(pkg, 'urdf', 'rospider_gazebo.urdf.xacro'))
    for collision in doc.getElementsByTagName('collision'):
        if collision.getElementsByTagName('mesh'):
            collision.parentNode.removeChild(collision)
    # The URDF's leg joint limits (velocity 1 rad/s) are placeholders -- Hiwonder bus servos turn
    # ~5 rad/s -- and Gazebo enforces them, so at 1 rad/s the legs lag the gait and feet dip at touch-down.
    for limit in doc.getElementsByTagName('limit'):
        if limit.parentNode.getAttribute('name').split('_')[0] in ('coxa', 'femur', 'tibla'):
            limit.setAttribute('velocity', '5.0')
    # Legs and arm cross the LiDAR's scan plane. Robot visuals get visibility flag 0x2, which the
    # LiDAR's visibility_mask excludes, so it never sees the robot itself; cameras still do.
    for link in doc.getElementsByTagName('link'):
        if link.getElementsByTagName('visual'):
            gazebo = doc.documentElement.appendChild(doc.createElement('gazebo'))
            gazebo.setAttribute('reference', link.getAttribute('name'))
            flags = gazebo.appendChild(doc.createElement('visual')).appendChild(doc.createElement('visibility_flags'))
            flags.appendChild(doc.createTextNode('2'))
    robot_description = doc.toxml()

    # Stands in for the real robot's move_controller: /controller/cmd_vel -> leg steps + body twist
    gait = Node(
        package='rospider_gazebo',
        executable='sim_gait.py',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': os.path.join(pkg, 'config', 'gz_bridge.yaml'), 'use_sim_time': True}],
    )

    # Spawn just above the floor; the robot settles onto its skid (base_footprint ~1 cm above the feet).
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'rospider',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'), '-z', '0.015',
                   '-Y', LaunchConfiguration('yaw')],
    )

    controllers = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', 'leg_controller', 'arm_controller', 'gripper_controller'],
    )

    return [
        gz_sim,
        robot_state_publisher,
        bridge,
        gait,
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[controllers])),
    ]


def generate_launch_description():
    pkg = get_package_share_directory('rospider_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'rospider_room.sdf')),
        DeclareLaunchArgument('gui', default_value='true', description='Show the Gazebo window'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=launch_setup),
    ])
