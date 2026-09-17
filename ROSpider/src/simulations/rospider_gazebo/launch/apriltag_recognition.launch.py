"""`ros2 launch rospider_gazebo apriltag_recognition.launch.py` -- the sim's
answer to Hiwonder's `ros2 launch example apriltag_recognition.launch.py` (6.3
AprilTag Tag Recognition).

The sim has no separate recognition window: apriltag_detect draws the axes,
corner dots and id on /apriltag_detect/image_result, and the apriltag_position
window is exactly that frame, so it stands in.

Same arguments as vision_demo.launch.py demo:=apriltag_position; see
rospider_gazebo/demo_launch.py.
"""

from rospider_gazebo.demo_launch import demo_launch


def generate_launch_description():
    return demo_launch('apriltag_position')
