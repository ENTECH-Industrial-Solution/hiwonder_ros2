"""`ros2 launch rospider_gazebo pose_control.launch.py` -- the sim's answer to
Hiwonder's `ros2 launch example pose_control.launch.py` (7.3.6 Body Motion
Control).

Same arguments as vision_demo.launch.py demo:=pose_control; see
rospider_gazebo/demo_launch.py.
"""

from rospider_gazebo.demo_launch import demo_launch


def generate_launch_description():
    return demo_launch('pose_control')
