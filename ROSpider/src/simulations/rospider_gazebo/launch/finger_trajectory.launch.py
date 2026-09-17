"""`ros2 launch rospider_gazebo finger_trajectory.launch.py` -- the sim's answer
to Hiwonder's `ros2 launch example finger_trajectory.launch.py` (7.3.2
Fingertip Trajectory Recognition).

Same arguments as vision_demo.launch.py demo:=finger_trajectory; see
rospider_gazebo/demo_launch.py.
"""

from rospider_gazebo.demo_launch import demo_launch


def generate_launch_description():
    return demo_launch('finger_trajectory')
