"""`ros2 launch rospider_gazebo object_tracking.launch.py` -- the sim's
answer to the app's object tracking (docs 6 Object Tracking:
`ros2 service call /object_tracking/enter`, then click the object).

Same arguments as vision_demo.launch.py demo:=object_tracking; see
rospider_gazebo/demo_launch.py.
"""

from rospider_gazebo.demo_launch import demo_launch


def generate_launch_description():
    return demo_launch('object_tracking')
