"""`ros2 launch rospider_gazebo track_and_grab.launch.py` -- the sim's answer to
Hiwonder's `ros2 launch example track_and_grab.launch.py` (8.7 3D Vision:
Object Grasping).

Upstream's launch starts the camera, the controller and the node; this one
starts pick_place.launch.py (the simulator with the blocks, the colour
detector and pick_and_place, held back with auto_start:=false so it waits
for the tracker) and then the track_and_grab window. Arguments for the
window (color:=, start:=, show:=, ...) pass through; pick_place.launch.py's
own (world:=, gui:=, ...) too.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('rospider_gazebo'), 'launch')
    scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'pick_place.launch.py')),
        launch_arguments={'auto_start': 'false'}.items())
    window = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'vision_demo.launch.py')),
        launch_arguments={'demo': 'track_and_grab'}.items())
    # The window waits for the spawn burst, as the other demos do when they
    # follow gazebo.launch.py from a second terminal.
    return LaunchDescription([scene, TimerAction(period=8.0, actions=[window])])
