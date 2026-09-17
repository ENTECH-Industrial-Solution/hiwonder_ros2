"""`ros2 launch rospider_gazebo object_classification.launch.py` -- the sim's
answer to Hiwonder's `ros2 launch example object_classification.launch.py`
(RGB-D course: object classification).

Same arguments as vision_demo.launch.py demo:=object_classification; see
rospider_gazebo/demo_launch.py.
"""

from rospider_gazebo.demo_launch import demo_launch


def generate_launch_description():
    return demo_launch('object_classification')
