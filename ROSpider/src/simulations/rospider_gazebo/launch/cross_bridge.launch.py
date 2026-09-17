"""`ros2 launch rospider_gazebo cross_bridge.launch.py` -- the sim's answer to
Hiwonder's `ros2 launch example cross_bridge.launch.py` (RGB-D course: cross
bridge).

Same arguments as vision_demo.launch.py demo:=cross_bridge; see
rospider_gazebo/demo_launch.py.
"""

from rospider_gazebo.demo_launch import demo_launch


def generate_launch_description():
    return demo_launch('cross_bridge')
