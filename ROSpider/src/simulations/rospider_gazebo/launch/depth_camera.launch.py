"""`ros2 launch rospider_gazebo depth_camera.launch.py` -- the sim's answer to
Hiwonder's `ros2 launch peripherals depth_camera.launch.py`.

On the robot that starts the RGB-D camera driver. In the simulation the
camera is part of the robot model, so "start the camera" means start the
simulator: this includes gazebo.launch.py unchanged, with the same arguments
(world:=, arm_pose:=, gui:=, ...). /depth_cam/rgb/image_raw,
/depth_cam/depth/image_raw and their camera_info topics then exist, as on
the robot.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package = get_package_share_directory('rospider_gazebo')
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package, 'launch', 'gazebo.launch.py')))])
