"""Start one of the ported Hiwonder demo windows.

    ros2 launch rospider_gazebo gazebo.launch.py
    ros2 launch rospider_gazebo vision_demo.launch.py demo:=color_position

One launch file rather than fifteen: every demo takes the same handful of
arguments and differs only in which executable runs and which detector it
needs, so the table below is the whole difference.

This file does not start Gazebo. The demos attach to a simulation that is
already running, which is what makes it possible to drive the robot around
with teleop, or to run pick_place.launch.py for its pedestal and tags, and
then bring a window up against the same world.

Arguments:
  demo          which window to open (see DEMOS below)
  source        sim (the robot's camera) or webcam (the PC's own, for the
                MediaPipe demos -- nobody is in the simulated world)
  detector      auto, color, yolo or none: which detector to start for the
                demos that read one. auto picks the one the demo was
                written for.
  show          false runs headless, publishing only <demo>/image_result
  use_sim_time  false for a camera-only demo with no simulator running

Anything else on the command line that matches a demo's own parameter --
debug, plane_distance, target_tag, color, tracker, model_path, ... -- is
passed through; see OVERRIDES.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#: demo -> the detector node it reads, or None when it reads the camera
#: directly. The keys are also the script names in scripts/ and the node
#: names the config file is keyed by.
DEMOS = {
    'color_position': 'color',
    'color_recognition': 'color',
    'color_track': 'color',
    'object_tracking': None,
    'line_following': None,
    'track_and_grab': None,
    'apriltag_position': 'apriltag',
    'apriltag_track': 'apriltag',
    'ar_view': None,
    'kcf_track': None,
    'prevent_falling': None,
    'cross_bridge': None,
    'object_volume': None,
    'object_classification': None,
    'hand_detect': None,
    'hand_gesture': None,
    'finger_trajectory': None,
    'face_track': None,
    'pose_control': None,
}

#: Demo parameters that can be set on the command line. Left empty, each one
#: falls back to config/vision_demos.yaml and then to the script's own
#: default, so only what is actually given is overridden.
OVERRIDES = (
    'image_topic', 'depth_topic', 'objects_topic', 'tags_topic',
    'color', 'target_tag', 'stop_distance', 'speed_limit',
    'turn_limit', 'tracker', 'model', 'model_path', 'model_scale',
    'model_yaw_deg', 'tag_size', 'debug', 'plane_distance', 'edge_tolerance',
    'steer_tolerance', 'forward_speed', 'turn_speed', 'max_hands',
    'detection_confidence', 'tracking_confidence', 'pan_gain', 'tilt_gain',
    'flip', 'start', 'place_point', 'threshold', 'pick_repeat',
    'stop_threshold', 'scan_topic', 'gui',
)

WEBCAM_TOPIC = '/webcam/image_raw'


def coerce(text):
    """A command-line string as the type the parameter wants.

    ROS 2 parameters are typed, and a launch argument is always a string: a
    bare "false" passed through as one is a non-empty string, which every
    `bool(...)` in the demos reads as True. So true/false, integers and
    floats are converted here rather than guessed at the far end.
    """
    lowered = text.strip().lower()
    if lowered in ('true', 'false'):
        return lowered == 'true'
    for kind in (int, float):
        try:
            return kind(text)
        except ValueError:
            pass
    return text


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('rospider_gazebo')
    demo = LaunchConfiguration('demo').perform(context)
    if demo not in DEMOS:
        raise RuntimeError(
            f'unknown demo {demo!r}; expected one of {", ".join(sorted(DEMOS))}')

    source = LaunchConfiguration('source').perform(context)
    detector = LaunchConfiguration('detector').perform(context)
    if detector == 'auto':
        detector = DEMOS[demo] or 'none'
    use_sim_time = coerce(LaunchConfiguration('use_sim_time').perform(context))

    overrides = {'use_sim_time': use_sim_time,
                 'show': coerce(LaunchConfiguration('show').perform(context))}
    for name in OVERRIDES:
        value = LaunchConfiguration(name).perform(context)
        if value != '':
            overrides[name] = coerce(value)
    if source == 'webcam' and 'image_topic' not in overrides:
        overrides['image_topic'] = WEBCAM_TOPIC
    if detector == 'yolo' and 'image_topic' not in overrides:
        # The colour demos default to /color_detect/image_result; the YOLO
        # node draws its overlay on its own topic instead.
        overrides['image_topic'] = '/yolo_detect/image_result'

    nodes = [Node(
        package='rospider_gazebo',
        executable=f'{demo}.py',
        name=demo,
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'vision_demos.yaml'),
                    overrides],
    )]

    if source == 'webcam':
        nodes.append(Node(
            package='rospider_gazebo',
            executable='webcam_publisher.py',
            name='webcam_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'topic': WEBCAM_TOPIC,
                'device': coerce(
                    LaunchConfiguration('device').perform(context)),
            }],
        ))

    if detector == 'color':
        nodes.append(Node(
            package='rospider_gazebo',
            executable='color_detect.py',
            name='color_detect',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'color_detect.yaml'),
                        {'use_sim_time': use_sim_time}],
        ))
    elif detector == 'yolo':
        nodes.append(Node(
            package='rospider_gazebo',
            executable='yolo_detect.py',
            name='yolo_detect',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'yolo.yaml'),
                        {'use_sim_time': use_sim_time}],
        ))
    elif detector == 'apriltag':
        nodes.append(Node(
            package='rospider_gazebo',
            executable='apriltag_detect.py',
            name='apriltag_detect',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'apriltag.yaml'),
                        {'use_sim_time': use_sim_time}],
        ))
    return nodes


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('demo', choices=sorted(DEMOS),
                              description='which demo window to open'),
        DeclareLaunchArgument('source', default_value='sim',
                              choices=['sim', 'webcam'],
                              description="the robot's camera, or the PC's"),
        DeclareLaunchArgument('detector', default_value='auto',
                              choices=['auto', 'color', 'yolo', 'apriltag',
                                       'none'],
                              description='detector node to start alongside'),
        DeclareLaunchArgument('show', default_value='true',
                              description='false: headless, publish only'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='false with no simulator running'),
        DeclareLaunchArgument('device', default_value='0',
                              description='webcam index or /dev path'),
    ]
    arguments += [DeclareLaunchArgument(name, default_value='')
                  for name in OVERRIDES]
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])
