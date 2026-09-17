"""The launch files named after Hiwonder's: one line each, built here.

The docs say `ros2 launch example color_position.launch.py color:=red`; the
sim answers to `ros2 launch rospider_gazebo color_position.launch.py
color:=red`. Each of those files is a two-line wrapper that calls
demo_launch() with the demo's name, and every other argument on the command
line (color:=, target_tag:=, debug:=, show:=, ...) reaches
vision_demo.launch.py unchanged, because an included launch file shares the
caller's launch configurations.

The MediaPipe demos default to source:=webcam here: on the robot the camera
sees the person, in the simulator nobody is there to see.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

#: Demos whose original looks at a person, so the PC's webcam is the default.
WEBCAM_DEMOS = ('hand_detect', 'hand_gesture', 'finger_trajectory',
                'face_track', 'pose_control')


def demo_launch(demo):
    """A LaunchDescription that opens one demo window by its sim name."""
    package = get_package_share_directory('rospider_gazebo')
    arguments = {'demo': demo}
    actions = []
    if demo in WEBCAM_DEMOS:
        actions.append(DeclareLaunchArgument(
            'source', default_value='webcam', choices=['sim', 'webcam'],
            description="the PC's webcam (default: the sim has no people), "
                        "or the robot's camera"))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package, 'launch', 'vision_demo.launch.py')),
        launch_arguments=arguments.items()))
    return LaunchDescription(actions)
