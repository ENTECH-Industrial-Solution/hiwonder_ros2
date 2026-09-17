"""`ros2 launch rospider_gazebo apriltag_position.launch.py` -- the sim's answer
to Hiwonder's `ros2 launch example apriltag_position.launch.py` (6.7 AprilTag
Tag Positioning).

Same arguments as vision_demo.launch.py demo:=apriltag_position; see
rospider_gazebo/demo_launch.py.
"""

from rospider_gazebo.demo_launch import demo_launch


def generate_launch_description():
    return demo_launch('apriltag_position')
