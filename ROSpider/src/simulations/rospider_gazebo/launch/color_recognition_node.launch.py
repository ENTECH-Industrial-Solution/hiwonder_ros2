"""`ros2 launch rospider_gazebo color_recognition_node.launch.py` -- the sim's
answer to Hiwonder's `ros2 launch example color_recognition_node.launch.py`
(6.2 Color Recognition).

Same arguments as vision_demo.launch.py demo:=color_recognition; see
rospider_gazebo/demo_launch.py.
"""

from rospider_gazebo.demo_launch import demo_launch


def generate_launch_description():
    return demo_launch('color_recognition')
