"""The Nav2 exercise's config: six friendly keys written into config/nav2_params.yaml.

Nav2's parameters are nested per node, and several meanings live in more
than one place -- the speed cap is in DWB and again in the velocity
smoother, the robot's size in both costmaps. Changing one copy would leave
the robot's behaviour as it was, so each friendly key is written to every
path it occupies (KEYS). Validation is challenge_params.merge's: unknown
keys, blank values and wrong types are refused; whole numbers become floats.

The challenge also swaps Hiwonder's DWB for Regulated Pure Pursuit
(challenge_base): measured in the exercise room, DWB stood still in front of
the 0.5 m doorway and again when a goal needed a U-turn, as it did in the
old mini game's 0.8 m gaps, which is where the RPP values below come from.
Pure Python (no cv2, no rclpy): nav_challenge.launch.py imports it.
"""

import copy
import tempfile

import yaml

from rospider_gazebo import challenge_params

#: friendly key -> every path it occupies in nav2_params.yaml. An int in a
#: path indexes a list (velocity_smoother.max_velocity is [x, y, theta]).
KEYS = {
    'xy_goal_tolerance': (
        ('controller_server', 'ros__parameters', 'general_goal_checker', 'xy_goal_tolerance'),
    ),
    'yaw_goal_tolerance': (
        ('controller_server', 'ros__parameters', 'general_goal_checker', 'yaw_goal_tolerance'),
    ),
    'robot_radius': (
        ('local_costmap', 'local_costmap', 'ros__parameters', 'robot_radius'),
        ('global_costmap', 'global_costmap', 'ros__parameters', 'robot_radius'),
    ),
    'inflation_radius': (
        ('local_costmap', 'local_costmap', 'ros__parameters', 'inflation_layer', 'inflation_radius'),
        ('global_costmap', 'global_costmap', 'ros__parameters', 'inflation_layer', 'inflation_radius'),
    ),
    'max_vel_x': (
        ('controller_server', 'ros__parameters', 'FollowPath', 'desired_linear_vel'),
        ('velocity_smoother', 'ros__parameters', 'max_velocity', 0),
    ),
    'max_vel_theta': (
        ('controller_server', 'ros__parameters', 'FollowPath', 'rotate_to_heading_angular_vel'),
        ('velocity_smoother', 'ros__parameters', 'max_velocity', 2),
        ('velocity_smoother', 'ros__parameters', 'min_velocity', 2),
    ),
}

#: Paths that take the negated value: the smoother clamps turning into
#: [min_velocity[2], max_velocity[2]], so both ends move together.
NEGATED = {('velocity_smoother', 'ros__parameters', 'min_velocity', 2)}


#: Regulated Pure Pursuit, as the old mini game ran it (git history:
#: launch/mini_game.launch.py). desired_linear_vel and
#: rotate_to_heading_angular_vel are overwritten by max_vel_x/max_vel_theta.
RPP = {
    'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
    'desired_linear_vel': 0.05,
    'lookahead_dist': 0.35,
    'min_lookahead_dist': 0.2,
    'max_lookahead_dist': 0.5,
    'lookahead_time': 1.5,
    'use_velocity_scaled_lookahead_dist': False,
    'rotate_to_heading_angular_vel': 0.25,
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

#: A gentle inflation gradient, so NavFn keeps its path centred in the
#: doorway (the mini game's value; Nav2's default 5.0 falls off in ~3 cm).
COST_SCALING_FACTOR = 3.0

#: The smoother's forward acceleration limit. Hiwonder's +-0.05 m/s^2 suits
#: its 0.05 m/s: at 0.3 m/s it needs 0.9 m to stop, and RPP (which starts
#: slowing 0.2 m out) overshot G1 into the wall (reviewer measurement).
LINEAR_ACCEL = 0.3


def challenge_base(nav2):
    """A copy of nav2_params.yaml with RPP as FollowPath and the gentle gradient."""
    base = copy.deepcopy(nav2)
    base['controller_server']['ros__parameters']['FollowPath'] = dict(RPP)
    for costmap in ('local_costmap', 'global_costmap'):
        layer = base[costmap][costmap]['ros__parameters']['inflation_layer']
        layer['cost_scaling_factor'] = COST_SCALING_FACTOR
    smoother = base['velocity_smoother']['ros__parameters']
    smoother['max_accel'][0] = LINEAR_ACCEL
    smoother['max_decel'][0] = -LINEAR_ACCEL
    return base


def _get(tree, path):
    for part in path:
        tree = tree[part]
    return tree


def _put(tree, path, value):
    _get(tree, path[:-1])[path[-1]] = value


def defaults(nav2):
    """Each friendly key's current value: the first path's, which also sets its type."""
    return {key: _get(nav2, paths[0]) for key, paths in KEYS.items()}


def apply(nav2, overrides, source):
    """challenge_base(nav2) with `overrides` written to every path."""
    merged = challenge_base(nav2)
    values = challenge_params.merge(defaults(merged), overrides, source)
    for key in overrides:
        for path in KEYS[key]:
            _put(merged, path, -values[key] if path in NEGATED else values[key])
    return merged


def merged_params_file(nav2_path, user_path):
    """Write nav2_params.yaml with the participant's values to a temp file; return its path."""
    with open(nav2_path) as handle:
        nav2 = yaml.safe_load(handle)
    merged = apply(nav2, challenge_params.load_params(user_path), user_path)
    with tempfile.NamedTemporaryFile('w', prefix='nav_challenge_', suffix='.yaml',
                                     delete=False) as handle:
        yaml.safe_dump(merged, handle)
        return handle.name
