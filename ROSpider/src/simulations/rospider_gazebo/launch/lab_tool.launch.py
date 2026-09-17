"""`ros2 launch rospider_gazebo lab_tool.launch.py` -- the sim's answer to
Hiwonder's `python3 ~/software/lab_tool/main.py` (6.1 Color Threshold
Adjustment).

Starts color_detect with tune:=true, which opens the LAB_Tool 1.0 window,
against a simulator that is already running (depth_camera.launch.py or any
other launch that starts Gazebo), the way the real tool needs
`ros2 launch peripherals depth_camera.launch.py` first.

  config       the LAB bands file to start from: a name in config/ or a path
               (default color_detect.yaml; color_detect_arena.yaml for the
               mini game's pastels)
  tuned_path   where Save writes (default ~/.ros/color_detect_tuned.json)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    package = get_package_share_directory('rospider_gazebo')
    config = LaunchConfiguration('config').perform(context)
    if '/' not in config:
        config = os.path.join(package, 'config', config)
    parameters = [config, {'use_sim_time': True, 'tune': True}]
    tuned_path = LaunchConfiguration('tuned_path').perform(context)
    if tuned_path:
        parameters.append({'tuned_path': tuned_path})
    return [Node(package='rospider_gazebo', executable='color_detect.py',
                 name='color_detect', output='screen', parameters=parameters)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='color_detect.yaml',
                              description='LAB bands file: a name in config/ or a path'),
        DeclareLaunchArgument('tuned_path', default_value='',
                              description='where Save writes; empty = the YAML\'s value'),
        OpaqueFunction(function=launch_setup),
    ])
