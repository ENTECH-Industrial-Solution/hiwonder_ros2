"""`ros2 launch rospider_gazebo hand_gesture.launch.py` -- the sim's answer to
Hiwonder's `ros2 launch example hand_gesture.launch.py` (7.3.4 Gesture
Control).

Same arguments as vision_demo.launch.py demo:=hand_gesture; see
rospider_gazebo/demo_launch.py.
"""

from rospider_gazebo.demo_launch import demo_launch


def generate_launch_description():
    return demo_launch('hand_gesture')
