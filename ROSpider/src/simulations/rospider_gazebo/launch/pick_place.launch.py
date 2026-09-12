import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('rospider_gazebo')
    pick_place_config = os.path.join(pkg, 'config', 'pick_place.yaml')

    # Objects are spawned at run time rather than written into
    # rospider_room.sdf: that world backs maps/rospider_room.* and the Nav2
    # demo's AMCL initial pose. Poses are in the world frame with the robot
    # at the origin, so they are also base_footprint coordinates. See spec
    # section 5.1.
    #
    # Read from pick_place.yaml's `scene`, not hardcoded here, so this file
    # and test/test_arm_ik.py's test_scene_layout_is_graspable share one
    # source of truth for the layout (spec section 4.4) -- a moved object is
    # then caught by the test instead of silently drifting out of step.
    with open(pick_place_config) as f:
        scene_params = yaml.safe_load(f)['pick_and_place']['ros__parameters']['scene']
    SCENE = tuple(scene_params.items())

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

    detector = Node(
        package='rospider_gazebo',
        executable='color_detect.py',
        name='color_detect',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'color_detect.yaml'),
                    {'use_sim_time': True}],
    )

    picker = Node(
        package='rospider_gazebo',
        executable='pick_and_place.py',
        name='pick_and_place',
        output='screen',
        parameters=[pick_place_config,
                    {'use_sim_time': True,
                     # LaunchConfiguration is always a string; without this
                     # wrapper the node receives auto_start:="false" as the
                     # Python string "false", and bool("false") is True.
                     'auto_start': ParameterValue(
                         LaunchConfiguration('auto_start'), value_type=bool)}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', os.path.join(pkg, 'rviz', 'pick_place.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg, 'worlds', 'rospider_room.sdf')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        gazebo,
        # The robot and its controllers need to exist before the cubes land on
        # the pedestal, or they drop through a world that is still loading.
        TimerAction(period=5.0, actions=spawns + [detector, picker, rviz]),
    ])
