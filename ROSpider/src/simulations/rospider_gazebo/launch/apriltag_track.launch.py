"""`ros2 launch rospider_gazebo apriltag_track.launch.py` -- the sim's answer to
Hiwonder's `ros2 launch example apriltag_track.launch.py` (6.8 AprilTag Tag
Tracking).

Same arguments as vision_demo.launch.py demo:=apriltag_track; see
rospider_gazebo/demo_launch.py.
"""

from rospider_gazebo.demo_launch import demo_launch


def generate_launch_description():
    return demo_launch('apriltag_track')
