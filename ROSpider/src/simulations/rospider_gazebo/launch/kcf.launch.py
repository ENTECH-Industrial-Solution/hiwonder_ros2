"""`ros2 launch rospider_gazebo kcf.launch.py` -- the sim's answer to Hiwonder's
`ros2 launch example kcf.launch.py` (6.10 KCF Object Recognition).

Same arguments as vision_demo.launch.py demo:=kcf_track; see
rospider_gazebo/demo_launch.py.
"""

from rospider_gazebo.demo_launch import demo_launch


def generate_launch_description():
    return demo_launch('kcf_track')
