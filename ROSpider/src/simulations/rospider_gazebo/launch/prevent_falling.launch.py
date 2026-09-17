"""`ros2 launch rospider_gazebo prevent_falling.launch.py` -- the sim's answer to
Hiwonder's `ros2 launch example prevent_falling.launch.py` (RGB-D course:
prevent falling).

Same arguments as vision_demo.launch.py demo:=prevent_falling; see
rospider_gazebo/demo_launch.py.
"""

from rospider_gazebo.demo_launch import demo_launch


def generate_launch_description():
    return demo_launch('prevent_falling')
