import os

import pytest
import yaml
from rospider_gazebo import challenge_params, nav_params
from rospider_gazebo.challenge_params import ChallengeConfigError

PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAV2 = os.path.join(PACKAGE, 'config', 'nav2_params.yaml')


def _nav2():
    with open(NAV2) as handle:
        return yaml.safe_load(handle)


def _base():
    """nav2_params.yaml as the challenge runs it: RPP instead of DWB."""
    return nav_params.challenge_base(_nav2())


def _get(tree, path):
    for part in path:
        tree = tree[part]
    return tree


def test_controller_is_regulated_pure_pursuit():
    follow = _base()['controller_server']['ros__parameters']['FollowPath']
    assert follow['plugin'].endswith('RegulatedPurePursuitController')
    assert 'critics' not in follow          # nothing of DWB's left behind
    merged = nav_params.apply(_nav2(), {}, 'f.yaml')
    assert merged['controller_server']['ros__parameters']['FollowPath'] == follow


def test_inflation_has_a_gentle_gradient():
    base = _base()
    for costmap in ('local_costmap', 'global_costmap'):
        layer = base[costmap][costmap]['ros__parameters']['inflation_layer']
        assert layer['cost_scaling_factor'] == 3.0


def test_every_path_exists_in_nav2_params():
    nav2 = _base()
    for key, paths in nav_params.KEYS.items():
        for path in paths:
            _get(nav2, path)            # KeyError/IndexError names the stale path


def test_each_key_reaches_every_path():
    values = {key: 0.123 + i for i, key in enumerate(nav_params.KEYS)}
    merged = nav_params.apply(_nav2(), values, 'f.yaml')
    for key, paths in nav_params.KEYS.items():
        for path in paths:
            expected = -values[key] if path in nav_params.NEGATED else values[key]
            assert _get(merged, path) == pytest.approx(expected), (key, path)


def test_turn_speed_limit_is_symmetric():
    """The smoother clamps into [min_velocity, max_velocity]; both ends must move."""
    merged = nav_params.apply(_nav2(), {'max_vel_theta': 0.5}, 'f.yaml')
    smoother = merged['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'][2] == 0.5
    assert smoother['min_velocity'][2] == -0.5


def test_smoother_can_accelerate_to_any_sensible_speed():
    """Hiwonder's +-0.05 m/s^2 needs 0.9 m to stop from 0.3 m/s: RPP overshoots goals."""
    smoother = _base()['velocity_smoother']['ros__parameters']
    assert smoother['max_accel'][0] >= 0.3
    assert smoother['max_decel'][0] <= -0.3


def test_list_entries_are_replaced_by_index():
    original = _get(_base(), ('velocity_smoother', 'ros__parameters', 'max_velocity'))
    merged = nav_params.apply(_nav2(), {'max_vel_x': 0.2}, 'f.yaml')
    velocity = _get(merged, ('velocity_smoother', 'ros__parameters', 'max_velocity'))
    assert velocity == [0.2, original[1], original[2]]


def test_keys_not_given_keep_nav2_values():
    nav2 = _base()
    merged = nav_params.apply(_nav2(), {'robot_radius': 0.3}, 'f.yaml')
    for key, paths in nav_params.KEYS.items():
        if key == 'robot_radius':
            continue
        for path in paths:
            assert _get(merged, path) == _get(nav2, path)


def test_input_tree_is_not_modified():
    nav2 = _nav2()
    nav_params.apply(nav2, {'robot_radius': 0.3}, 'f.yaml')
    assert nav2 == _nav2()


def test_whole_number_becomes_float():
    merged = nav_params.apply(_nav2(), {'max_vel_x': 1}, 'f.yaml')
    value = _get(merged, ('controller_server', 'ros__parameters', 'FollowPath', 'desired_linear_vel'))
    assert value == 1.0 and isinstance(value, float)


@pytest.mark.parametrize('overrides, key', [
    ({'robot_radus': 0.2}, 'robot_radus'),
    ({'robot_radius': None}, 'robot_radius'),
    ({'max_vel_x': 'fast'}, 'max_vel_x'),
])
def test_bad_values_are_refused_by_name(overrides, key):
    with pytest.raises(ChallengeConfigError, match=key):
        nav_params.apply(_nav2(), overrides, 'f.yaml')


def test_shipped_template_and_answer_key_merge_cleanly():
    for name in ('nav_challenge.yaml', 'nav_challenge_solved.yaml'):
        user = os.path.join(PACKAGE, 'config', name)
        if not os.path.exists(user):    # the answer key is not in the repo (instructors keep it)
            continue
        params = challenge_params.load_params(user)
        assert set(params) == set(nav_params.KEYS)
        nav_params.apply(_nav2(), params, user)


def test_merged_file_is_nav2_shaped(tmp_path):
    user = tmp_path / 'user.yaml'
    user.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'robot_radius': 0.3}}}))
    with open(nav_params.merged_params_file(NAV2, str(user))) as handle:
        merged = yaml.safe_load(handle)
    assert merged['global_costmap']['global_costmap']['ros__parameters']['robot_radius'] == 0.3
    assert set(merged) == set(_nav2())
