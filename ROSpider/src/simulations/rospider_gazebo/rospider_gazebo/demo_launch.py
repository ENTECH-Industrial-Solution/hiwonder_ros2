"""The launch files named after Hiwonder's: one line each, built here.

The docs say `ros2 launch example apriltag_track.launch.py`; the sim
answers to `ros2 launch rospider_gazebo apriltag_track.launch.py`. Each of
those files is a two-line wrapper that calls demo_launch() with the demo's
name, and every other argument on the command
line (color:=, target_tag:=, debug:=, show:=, ...) reaches
vision_demo.launch.py unchanged, because an included launch file shares the
caller's launch configurations.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def demo_launch(demo):
    """A LaunchDescription that opens one demo window by its sim name."""
    package = get_package_share_directory('rospider_gazebo')
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package, 'launch', 'vision_demo.launch.py')),
        launch_arguments={'demo': demo}.items())])
