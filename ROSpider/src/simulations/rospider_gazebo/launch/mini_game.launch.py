"""The workshop mini game: pick pastel cubes, carry them down the arena,
place each on the AprilTag station whose colour marker matches.

    ros2 launch rospider_gazebo mini_game.launch.py detector:=color|yolo seed:=3

Composition: gazebo.launch.py (worlds/arena.sdf, arm level so the survey can
see the stations) -> the pick pedestal, the three cubes and the three
stations (station SDF generated here with the seed's colour assignment) ->
Nav2 on maps/arena.yaml with the arena's speed and footprint -> the chosen
detector -> apriltag_detect -> pick_and_place -> the mission node running
config/missions/<mission>.yaml.

The world file itself stays a bare hall so SLAM and V-SLAM can map it; every
game object is spawned from here.
"""

import os
import random
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rospider_gazebo import stations
from rospider_gazebo.maps import resolve_map

# The pastel cubes and the station markers share these triples, so one HSV
# band per colour sees both. Low saturation on purpose: see
# config/color_detect_arena.yaml.
COLOURS = {
    'pink': (0.98, 0.72, 0.80, 1.0),
    'yellow': (0.98, 0.94, 0.62, 1.0),
    'sky': (0.68, 0.85, 0.95, 1.0),
}

# Pick pedestal and cubes in the world frame. The robot spawns at the
# origin, which is also the mission's pick_table; its dock creeps DOCK
# metres forward to an exact pose, so the cubes then sit at the same reach
# (x 0.235 from the base) as pick_place.yaml's scene. Nav2 never plans
# from beside the pedestal this way: at pick_table the pedestal's front
# edge is 0.695 m away, well outside robot_radius + inflation (0.45). With
# the pedestal 0.25 m nearer, RPP reported "collision ahead" on the very
# first leg and gave up.
DOCK = 0.5
PEDESTAL = (0.275 + DOCK, 0.0, 0.040)      # 0.16 m deep, front edge at 0.195 + DOCK
CUBES = {'pink': (0.235 + DOCK, 0.07, 0.105),
         'yellow': (0.235 + DOCK, 0.0, 0.105),
         'sky': (0.235 + DOCK, -0.07, 0.105)}

# Stations along the east wall, facing down the hall (yaw pi: the model
# faces +x and a robot coming from -x must see the tag). 0.30 m tags, twice
# the demo's: from the survey point 2 m away a 0.15 m tag is 30 px and
# undetectable.
TAG_SIZE = 0.30
STATIONS = {0: (5.1, 0.5, 3.14159),
            1: (5.1, 0.0, 3.14159),
            2: (5.1, -0.5, 3.14159)}

# Nav2 for the arena: Hiwonder's 0.01 m footprint let the robot brush the
# walls. 0.15 m is the skid box's half-diagonal: with 0.10 the box's corner
# clipped a partition end and the robot tipped over. The inflation gradient
# keeps NavFn's path centred in the 0.8 m gaps; RPP (below) follows it.
# 0.15 m/s commanded walks at about 0.10 in the simulator (the gait scales
# the twist down); the room's 0.05 made a lap of the hall take ten minutes.
NAV2_OVERRIDES = {
    'robot_radius': 0.15,
    # A wide, gentle inflation: NavFn then keeps its path centred in the
    # 0.8 m gaps and clear of the pedestal's corner. With 0.18 the gradient
    # was 3 cm wide, the path hugged the inscribed boundary and the robot's
    # own tracking error put it inside, where RPP stops ("collision ahead")
    # and NavFn cannot replan from.
    'inflation_radius': 0.30,
    'cost_scaling_factor': 3.0,
    # Both costmaps wait this long for odom TF before giving up on
    # activation. Nav2's 60 s default lost a run on this machine when
    # Gazebo took longer than that to come up; a workshop laptop will be
    # slower still.
    'initial_transform_timeout': 300.0,
    # The hexapod strafes (docking, the survey pan); AMCL's differential
    # model reads a sideways step as turn-move-turn and its estimate jumped
    # by tens of centimetres after an undock, which then sent Nav2 into
    # the pedestal. Omni model, and update more often than every 0.25 m.
    'robot_model_type': 'nav2_amcl::OmniMotionModel',
    'update_min_d': 0.1,
    'update_min_a': 0.1,
    'max_vel_x': 0.15,
    'max_velocity': [0.15, 0.0, 0.4],
    'max_accel': [0.15, 0.0, 0.5],
}


def _rewrite(node, overrides):
    """Apply overrides to every matching key, at any depth."""
    if isinstance(node, dict):
        for key, value in node.items():
            if key in overrides:
                node[key] = overrides[key]
            else:
                _rewrite(value, overrides)
    elif isinstance(node, list):
        for item in node:
            _rewrite(item, overrides)


def _read_pgm(path):
    """(rows, width, height, maxval) of a binary P5 PGM. No cv2: importing
    it inside `ros2 launch` sets Qt's plugin path for every child process
    and the Gazebo GUI then fails to start."""
    with open(path, 'rb') as handle:
        data = handle.read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    pos += 1                              # the single whitespace after maxval
    if tokens[0] != b'P5':
        raise ValueError(f'{path} is not a binary PGM')
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    pixels = bytearray(data[pos:pos + width * height])
    return pixels, width, height, maxval


def map_with_pedestal(map_yaml):
    """A copy of the 2D map with the pick pedestal painted in.

    The pedestal is 8 cm tall, below the LiDAR's 15 cm scan plane, so no
    SLAM run ever sees it -- and Nav2 then drives through it (the robot
    pitches over the edge). Painting it here means a team can map the hall
    with SLAM or V-SLAM, pass map:=<theirs>, and still not hit it. The
    shipped maps/arena.pgm has it painted already; painting twice is
    harmless.
    """
    with open(map_yaml) as handle:
        info = yaml.safe_load(handle)
    image_path = os.path.join(os.path.dirname(map_yaml), info['image'])
    try:
        pixels, width, height, maxval = _read_pgm(image_path)
    except (OSError, ValueError):
        return map_yaml
    res = float(info['resolution'])
    ox, oy = float(info['origin'][0]), float(info['origin'][1])
    px, py, _ = PEDESTAL
    x0, x1 = px - 0.08, px + 0.08          # arena_pedestal is 0.16 x 0.22
    y0, y1 = py - 0.11, py + 0.11
    col0, col1 = int((x0 - ox) / res), int((x1 - ox) / res)
    row0 = height - 1 - int((y1 - oy) / res)
    row1 = height - 1 - int((y0 - oy) / res)
    occupied = maxval if info.get('negate', 0) else 0
    for row in range(max(row0, 0), min(row1, height - 1) + 1):
        for col in range(max(col0, 0), min(col1, width - 1) + 1):
            pixels[row * width + col] = occupied
    out_dir = tempfile.mkdtemp(prefix='arena_map_')
    with open(os.path.join(out_dir, 'map.pgm'), 'wb') as handle:
        handle.write(f'P5\n{width} {height}\n{maxval}\n'.encode())
        handle.write(bytes(pixels))
    info['image'] = 'map.pgm'
    out_yaml = os.path.join(out_dir, 'map.yaml')
    with open(out_yaml, 'w') as handle:
        yaml.safe_dump(info, handle)
    return out_yaml


def arena_nav2_params(pkg):
    with open(os.path.join(pkg, 'config', 'nav2_params.yaml')) as handle:
        params = yaml.safe_load(handle)
    _rewrite(params, NAV2_OVERRIDES)
    # The pick pedestal is 8 cm tall, below the LiDAR plane, so it is painted
    # into maps/arena.pgm rather than scanned -- and the room's local costmap
    # (voxel + inflation only) would never see it, so DWB drove straight
    # into it. The static layer goes into the local costmap here.
    local = params['local_costmap']['local_costmap']['ros__parameters']
    local['plugins'] = ['static_layer'] + [
        name for name in local['plugins'] if name != 'static_layer']
    local['static_layer'] = {'plugin': 'nav2_costmap_2d::StaticLayer',
                             'map_subscribe_transient_local': True}
    # Regulated Pure Pursuit instead of Hiwonder's DWB. In the S-bend's
    # 0.8 m gaps DWB sampled no valid trajectory at all and sat still
    # through spin/backup recoveries until the goal timed out (measured:
    # 600 s to cover 2.3 m). RPP just follows NavFn's path, rotates in
    # place when the heading is off, and slows down for the corners.
    controller = params['controller_server']['ros__parameters']
    controller['FollowPath'] = {
        'plugin': 'nav2_regulated_pure_pursuit_controller::'
                  'RegulatedPurePursuitController',
        'desired_linear_vel': NAV2_OVERRIDES['max_vel_x'],
        'lookahead_dist': 0.35,
        'min_lookahead_dist': 0.2,
        'max_lookahead_dist': 0.5,
        'lookahead_time': 1.5,
        'use_velocity_scaled_lookahead_dist': False,
        'rotate_to_heading_angular_vel': 0.4,
        'transform_tolerance': 0.2,
        'min_approach_linear_velocity': 0.05,
        'approach_velocity_scaling_dist': 0.2,
        'use_collision_detection': True,
        'max_allowed_time_to_collision_up_to_carrot': 1.0,
        'use_regulated_linear_velocity_scaling': True,
        'use_cost_regulated_linear_velocity_scaling': False,
        'regulated_linear_scaling_min_radius': 0.3,
        'regulated_linear_scaling_min_speed': 0.08,
        'use_rotate_to_heading': True,
        'rotate_to_heading_min_angle': 0.5,
        'max_angular_accel': 1.0,
        'max_robot_pose_search_dist': 10.0,
        'allow_reversing': False,
    }
    # Waypoints are places to stand, not docking targets: 15 cm is plenty,
    # and RPP inching toward a goal 10 cm away next to an inflated obstacle
    # is how a survey ended in "collision ahead" and an aborted goal.
    controller['general_goal_checker']['xy_goal_tolerance'] = 0.15
    handle = tempfile.NamedTemporaryFile('w', suffix='_nav2_arena.yaml',
                                         delete=False)
    yaml.safe_dump(params, handle)
    handle.close()
    return handle.name


def launch_setup(context):
    pkg = get_package_share_directory('rospider_gazebo')
    seed = int(LaunchConfiguration('seed').perform(context))
    mission = LaunchConfiguration('mission').perform(context)
    detector = LaunchConfiguration('detector').perform(context)
    color_config = LaunchConfiguration('color_config').perform(context)
    map_yaml = resolve_map(LaunchConfiguration('map').perform(context), '.yaml')
    tune = ParameterValue(LaunchConfiguration('tune'), value_type=bool)

    if os.sep not in mission:
        mission = os.path.join(pkg, 'config', 'missions', mission + '.yaml')
    if os.sep not in color_config:
        color_config = os.path.join(pkg, 'config', color_config)

    # Which marker colour goes on which station: a permutation by seed, so
    # every round can be different and no team can hardcode it.
    names = list(COLOURS)
    random.Random(seed).shuffle(names)
    assignment = dict(zip(sorted(STATIONS), names))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': os.path.join(pkg, 'worlds', 'arena.sdf'),
            'gui': LaunchConfiguration('gui'),
            'arm_pose': LaunchConfiguration('arm_pose'),
        }.items(),
    )

    def spawn_file(name, model, pose, yaw=0.0):
        return Node(package='ros_gz_sim', executable='create', name=f'spawn_{name}',
                    output='screen',
                    arguments=['-file', os.path.join(pkg, 'models', model, 'model.sdf'),
                               '-name', name, '-x', str(pose[0]), '-y', str(pose[1]),
                               '-z', str(pose[2]), '-Y', str(yaw)])

    spawns = [spawn_file('arena_pedestal', 'arena_pedestal', PEDESTAL)]
    spawns += [spawn_file(f'pick_cube_{colour}', f'pick_cube_{colour}', pose)
               for colour, pose in CUBES.items()]
    for tag_id, (x, y, yaw) in STATIONS.items():
        # -string has no file to resolve the texture path against.
        sdf = stations.station_sdf(tag_id, COLOURS[assignment[tag_id]], TAG_SIZE).replace(
            '../../worlds/textures/', os.path.join(pkg, 'worlds', 'textures') + '/')
        spawns.append(Node(package='ros_gz_sim', executable='create',
                           name=f'spawn_tag_station_{tag_id}', output='screen',
                           arguments=['-string', sdf, '-name', f'tag_station_{tag_id}',
                                      '-x', str(x), '-y', str(y), '-z', '0', '-Y', str(yaw)]))

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_with_pedestal(map_yaml),
            'params_file': arena_nav2_params(pkg),
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )

    color_detect = Node(
        package='rospider_gazebo', executable='color_detect.py', name='color_detect',
        output='screen',
        parameters=[color_config, {'use_sim_time': True, 'tune': tune}],
        condition=IfCondition(PythonExpression(["'", detector, "' == 'color'"])),
    )
    yolo_detect = Node(
        package='rospider_gazebo', executable='yolo_detect.py', name='yolo_detect',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'yolo.yaml'),
                    {'use_sim_time': True, 'tune': tune,
                     'model_path': LaunchConfiguration('model'),
                     'tuned_path': '~/.ros/yolo_detect_game_tuned.json'}],
        condition=IfCondition(PythonExpression(["'", detector, "' == 'yolo'"])),
    )
    apriltag = Node(
        package='rospider_gazebo', executable='apriltag_detect.py', name='apriltag_detect',
        output='screen',
        # Its own tuned file: a behaviour row saved during the morning's
        # AprilTag demo (tag0: approach) would otherwise be live the moment
        # the mission enables behaviours for a delivery.
        parameters=[os.path.join(pkg, 'config', 'apriltag.yaml'),
                    {'use_sim_time': True, 'tune': tune,
                     'tag_size': TAG_SIZE,
                     'tuned_path': '~/.ros/apriltag_game_tuned.json'}],
    )
    picker = Node(
        package='rospider_gazebo', executable='pick_and_place.py', name='pick_and_place',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'pick_place.yaml'),
                    {'use_sim_time': True, 'auto_start': False,
                     'colors': list(COLOURS)}],
    )
    mission_node = Node(
        package='rospider_gazebo', executable='mission.py', name='mission',
        output='screen',
        parameters=[{'use_sim_time': True, 'mission_file': mission,
                     'auto_start': ParameterValue(LaunchConfiguration('auto_start'),
                                                  value_type=bool)}],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', os.path.join(pkg, 'rviz', 'pick_place.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [
        LogInfo(msg=f'mini game: seed {seed}, markers ' + ', '.join(
            f'tag {t} = {c}' for t, c in assignment.items()) +
            f', detector {detector}, mission {mission}'),
        gazebo,
        # The robot and its controllers first, or the cubes drop through a
        # world that is still loading (same 5 s as pick_place.launch.py).
        TimerAction(period=5.0, actions=spawns + [color_detect, yolo_detect,
                                                  apriltag, picker, rviz]),
        TimerAction(period=10.0, actions=[nav2]),
        # Nav2 needs a while to activate; the mission retries a rejected
        # goal for 90 s, this just keeps the log readable.
        TimerAction(period=20.0, actions=[mission_node]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('detector', default_value='color', choices=['color', 'yolo'],
                              description='which node publishes /yolo/object_detect'),
        DeclareLaunchArgument('color_config', default_value='color_detect_arena.yaml',
                              description='HSV bounds file for detector:=color: a name in '
                                          'config/ or a path'),
        DeclareLaunchArgument('model', default_value='models/yolo/cubes.pt',
                              description='YOLO weights for detector:=yolo (the shipped '
                                          'cubes.pt does not know the pastel cubes; train one)'),
        DeclareLaunchArgument('map', default_value=os.path.join(
            get_package_share_directory('rospider_gazebo'), 'maps', 'arena.yaml')),
        DeclareLaunchArgument('mission', default_value='basic',
                              description='config/missions/<name>.yaml or a path'),
        DeclareLaunchArgument('seed', default_value='0',
                              description='which marker colour goes on which station'),
        DeclareLaunchArgument('auto_start', default_value='true',
                              description='false waits for /mission/start'),
        DeclareLaunchArgument('tune', default_value='false',
                              description='open the detector and AprilTag tuning windows'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('arm_pose', default_value='horizontal',
                              choices=['init', 'horizontal'],
                              description='horizontal for the game (the survey must see the '
                                          'stations); init to capture a YOLO dataset of the '
                                          'cubes with the camera looking down'),
        OpaqueFunction(function=launch_setup),
    ])
