"""`ros2 launch rospider_gazebo ar.launch.py` -- the sim's answer to Hiwonder's
`ros2 launch example ar.launch.py` (6.4 AR Vision).

Same arguments as vision_demo.launch.py demo:=ar_view; see
rospider_gazebo/demo_launch.py.
"""

from rospider_gazebo.demo_launch import demo_launch


def generate_launch_description():
    return demo_launch('ar_view')
