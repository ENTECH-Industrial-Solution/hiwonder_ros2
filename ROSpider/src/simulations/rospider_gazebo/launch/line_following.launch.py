"""`ros2 launch rospider_gazebo line_following.launch.py` -- the sim's
answer to the app's line following (docs 6 Line Following:
`ros2 service call /line_following/enter`, then click the line).

Upstream's node runs on the phone app over whatever floor the robot is on;
the simulator needs a line to follow, so this starts gazebo.launch.py with
worlds/line_track.sdf (the room, empty but for a black loop on the floor),
spawns the robot on the line facing along it, and then opens the
line_following window. Arguments for the window (threshold:=, start:=,
show:=, ...) pass through; so do gazebo.launch.py's (gui:=, arm_pose:=).

Attach to a simulator that is already running with
    ros2 launch rospider_gazebo vision_demo.launch.py demo:=line_following
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rospider_gazebo.line_track import start_pose


def generate_launch_description():
    package = get_package_share_directory('rospider_gazebo')
    launch_dir = os.path.join(package, 'launch')
    x, y, yaw = start_pose()
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gazebo.launch.py')),
        launch_arguments={
            'world': os.path.join(package, 'worlds', 'line_track.sdf'),
            'x': str(x), 'y': str(y), 'yaw': str(yaw),
        }.items())
    window = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'vision_demo.launch.py')),
        launch_arguments={'demo': 'line_following'}.items())
    # The window waits for the controllers, as track_and_grab.launch.py does:
    # its start pose is dropped if it goes out before joint_states exist.
    return LaunchDescription([sim, TimerAction(period=8.0, actions=[window])])
