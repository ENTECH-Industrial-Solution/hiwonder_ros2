"""`ros2 launch rospider_gazebo color_track_node.launch.py` -- the sim's answer
to Hiwonder's `ros2 launch example color_track_node.launch.py` (8.2 2D Vision:
Color Tracking).

Same arguments as vision_demo.launch.py demo:=color_track; see
rospider_gazebo/demo_launch.py.
"""

from rospider_gazebo.demo_launch import demo_launch


def generate_launch_description():
    return demo_launch('color_track')
