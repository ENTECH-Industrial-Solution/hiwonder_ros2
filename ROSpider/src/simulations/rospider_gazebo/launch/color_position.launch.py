"""`ros2 launch rospider_gazebo color_position.launch.py` -- the sim's answer to
Hiwonder's `ros2 launch example color_position.launch.py` (6.5 Color Block
Coordinate Positioning).

Same arguments as vision_demo.launch.py demo:=color_position; see
rospider_gazebo/demo_launch.py.
"""

from rospider_gazebo.demo_launch import demo_launch


def generate_launch_description():
    return demo_launch('color_position')
