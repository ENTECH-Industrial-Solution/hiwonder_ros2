import pytest
import yaml
from rospider_gazebo import challenge_params
from rospider_gazebo.challenge_params import ChallengeConfigError

BASE = {'scan_topic': 'scan', 'max_laser_range': 12.0, 'resolution': 0.05,
        'map_update_interval': 2.0, 'mode': 'mapping'}


def _write(path, params):
    path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    return str(path)


def test_ensure_user_file_copies_once(tmp_path):
    template = tmp_path / 'template.yaml'
    template.write_text('a: 1\n')
    user = tmp_path / 'home' / '.ros' / 'slam_challenge.yaml'
    assert challenge_params.ensure_user_file(str(template), str(user)) is True
    user.write_text('edited\n')
    assert challenge_params.ensure_user_file(str(template), str(user)) is False
    assert user.read_text() == 'edited\n'


def test_participant_values_win_and_others_come_from_base():
    merged = challenge_params.merge(BASE, {'resolution': 0.25}, 'f.yaml')
    assert merged['resolution'] == 0.25
    assert merged['mode'] == 'mapping'


def test_whole_number_for_a_float_is_converted():
    merged = challenge_params.merge(BASE, {'max_laser_range': 12}, 'f.yaml')
    assert merged['max_laser_range'] == 12.0
    assert isinstance(merged['max_laser_range'], float)


def test_text_for_a_number_is_refused():
    with pytest.raises(ChallengeConfigError, match='max_laser_range'):
        challenge_params.merge(BASE, {'max_laser_range': '12'}, 'f.yaml')


def test_blank_value_is_refused():
    with pytest.raises(ChallengeConfigError, match='scan_topic'):
        challenge_params.merge(BASE, {'scan_topic': None}, 'f.yaml')


def test_number_for_a_text_parameter_is_refused():
    with pytest.raises(ChallengeConfigError, match='scan_topic'):
        challenge_params.merge(BASE, {'scan_topic': 5}, 'f.yaml')


def test_bool_parameter_needs_true_or_false():
    base = dict(BASE, do_loop_closing=True)
    assert challenge_params.merge(base, {'do_loop_closing': False}, 'f.yaml')['do_loop_closing'] is False
    with pytest.raises(ChallengeConfigError, match='do_loop_closing'):
        challenge_params.merge(base, {'do_loop_closing': 'no'}, 'f.yaml')


def test_misspelled_key_is_refused_by_name():
    with pytest.raises(ChallengeConfigError, match='max_laser_rang'):
        challenge_params.merge(BASE, {'max_laser_rang': 12.0}, 'f.yaml')


def test_yaml_syntax_error_names_file_and_line(tmp_path):
    bad = tmp_path / 'bad.yaml'
    bad.write_text('/**:\n  ros__parameters:\n    resolution: [0.05\n')
    with pytest.raises(ChallengeConfigError) as err:
        challenge_params.load_params(str(bad))
    assert 'bad.yaml' in str(err.value)
    assert 'บรรทัด' in str(err.value)


def test_wrong_shape_is_refused(tmp_path):
    flat = tmp_path / 'flat.yaml'
    flat.write_text('resolution: 0.05\n')
    with pytest.raises(ChallengeConfigError, match='ros__parameters'):
        challenge_params.load_params(str(flat))


def test_merged_file_is_slam_toolbox_shaped(tmp_path):
    base = _write(tmp_path / 'base.yaml', BASE)
    user = _write(tmp_path / 'user.yaml', {'scan_topic': 'scan_raw'})
    path = challenge_params.merged_params_file(base, user)
    params = yaml.safe_load(open(path))['/**']['ros__parameters']
    assert params['scan_topic'] == 'scan_raw'
    assert params['max_laser_range'] == 12.0


def test_fixed_values_sit_between_base_and_participant(tmp_path):
    base = _write(tmp_path / 'base.yaml', dict(BASE, do_loop_closing=True))
    user = _write(tmp_path / 'user.yaml', {'resolution': 0.25})
    fixed = {'do_loop_closing': False}
    params = yaml.safe_load(open(challenge_params.merged_params_file(base, user, fixed)))
    assert params['/**']['ros__parameters']['do_loop_closing'] is False
    user2 = _write(tmp_path / 'user2.yaml', {'do_loop_closing': True})
    params = yaml.safe_load(open(challenge_params.merged_params_file(base, user2, fixed)))
    assert params['/**']['ros__parameters']['do_loop_closing'] is True


def test_shipped_template_and_answer_key_merge_cleanly():
    """The two files in config/ must only use keys Hiwonder's slam.yaml has."""
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    package = os.path.dirname(here)
    base = os.path.join(package, '..', '..', 'slam', 'config', 'slam.yaml')
    for name in ('slam_challenge.yaml', 'slam_challenge_solved.yaml'):
        user = os.path.join(package, 'config', name)
        challenge_params.merge(challenge_params.load_params(base),
                               challenge_params.load_params(user), user)
