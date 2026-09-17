"""`ros2 launch rospider_gazebo hand_detect.launch.py` -- the sim's answer to
Hiwonder's `ros2 launch example hand_detect.launch.py` (7.3.3 Gesture
Recognition).

Same arguments as vision_demo.launch.py demo:=hand_detect; see
rospider_gazebo/demo_launch.py.
"""

from rospider_gazebo.demo_launch import demo_launch


def generate_launch_description():
    return demo_launch('hand_detect')
