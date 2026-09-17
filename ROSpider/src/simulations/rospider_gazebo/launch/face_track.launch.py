"""`ros2 launch rospider_gazebo face_track.launch.py` -- the sim's answer to
Hiwonder's `ros2 launch example face_track.launch.py` (7.3.5 Face Detection
and Tracking).

Same arguments as vision_demo.launch.py demo:=face_track; see
rospider_gazebo/demo_launch.py.
"""

from rospider_gazebo.demo_launch import demo_launch


def generate_launch_description():
    return demo_launch('face_track')
