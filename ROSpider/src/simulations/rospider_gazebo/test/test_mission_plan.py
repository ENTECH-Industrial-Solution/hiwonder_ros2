import math
import pathlib

import pytest
from rospider_gazebo import mission_plan
from rospider_gazebo.challenge_params import ChallengeConfigError
from rospider_gazebo.mission_plan import Step

PKG = pathlib.Path(__file__).resolve().parents[1]
GOOD = {'speed': 0.15, 'steps': [{'go_to': [3.8, 0.7, 90]}, {'go_to_tag': 3}, {'pick': 'blue'},
                                 {'go_to': [0.56, 0.55, 180]}, {'place': '0.16 0.0 0.035'}]}


def _parse(data):
    return mission_plan.parse(data, 'm.yaml')


def test_a_good_file_becomes_typed_steps():
    speed, steps = _parse(GOOD)
    assert speed == 0.15
    assert steps[0].kind == 'go_to' and steps[0].value[:2] == (3.8, 0.7)
    assert steps[0].value[2] == pytest.approx(math.pi / 2)
    assert steps[3].value[2] == pytest.approx(math.pi)
    assert steps[1] == Step('go_to_tag', 3)
    assert steps[2] == Step('pick', 'blue')
    assert steps[4] == Step('place', (0.16, 0.0, 0.035))


@pytest.mark.parametrize('speed', [0.04, 0.21, '0.1', True, None])
def test_speed_outside_the_range_or_not_a_number_is_refused(speed):
    with pytest.raises(ChallengeConfigError, match='speed'):
        _parse(dict(GOOD, speed=speed))


def test_missing_speed_or_steps_is_refused():
    with pytest.raises(ChallengeConfigError, match='speed'):
        _parse({'steps': GOOD['steps']})
    with pytest.raises(ChallengeConfigError, match='steps'):
        _parse({'speed': 0.15})


def test_unknown_top_level_key_is_refused():
    with pytest.raises(ChallengeConfigError, match='sped'):
        _parse(dict(GOOD, sped=0.1))


@pytest.mark.parametrize('step, word', [
    ({'goto': [1, 1, 0]}, 'goto'),                 # misspelled step
    ({'go_to': [1, 1]}, 'go_to'),                  # two numbers
    ({'go_to': ['a', 1, 0]}, 'go_to'),
    ({'go_to_tag': 'three'}, 'go_to_tag'),
    ({'go_to_tag': True}, 'go_to_tag'),
    ({'pick': 'yellow'}, 'pick'),
    ({'place': '0.16 0.0'}, 'place_point'),
    ({'place': 0.16}, 'place_point'),
    ({'go_to': [1, 1, 0], 'pick': 'red'}, 'ขั้นที่ 1'),   # two keys in one step
    ('pick blue', 'ขั้นที่ 1'),                     # not a mapping
])
def test_bad_steps_are_refused_with_the_step_number(step, word):
    with pytest.raises(ChallengeConfigError, match=word) as err:
        _parse({'speed': 0.15, 'steps': [step]})
    assert 'ขั้นที่ 1' in str(err.value)


def test_empty_steps_are_refused():
    with pytest.raises(ChallengeConfigError, match='steps'):
        _parse({'speed': 0.15, 'steps': []})


def test_describe_is_thai_and_names_the_values():
    _speed, steps = _parse(GOOD)
    assert '3.80' in mission_plan.describe(steps[0]) and '180' in mission_plan.describe(steps[3])
    assert '3' in mission_plan.describe(steps[1])
    assert 'blue' in mission_plan.describe(steps[2])
    assert '0.16' in mission_plan.describe(steps[4])


@pytest.mark.parametrize('name', ['mission_challenge.yaml', 'mission_challenge_solved.yaml'])
def test_shipped_files_load(name):
    path = PKG / 'config' / name
    if not path.exists():
        pytest.skip('written in Task 7')
    mission_plan.load(str(path))


def test_load_reports_yaml_errors_with_the_line(tmp_path):
    bad = tmp_path / 'm.yaml'
    bad.write_text('speed: 0.15\nsteps:\n  - go_to: [1, 2\n')
    with pytest.raises(ChallengeConfigError, match='บรรทัด'):
        mission_plan.load(str(bad))
