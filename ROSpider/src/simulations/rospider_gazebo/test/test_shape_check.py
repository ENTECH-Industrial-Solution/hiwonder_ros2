import pytest
from rospider_gazebo import shape_check
from rospider_gazebo.challenge_params import ChallengeConfigError
from rospider_gazebo.shape_check import EXPECTED, TARGET

ROI = [0, 350, 150, 500]


def _obj(name, kind=None, grow=0):
    x, y, w, h = EXPECTED[name]
    return {'kind': kind or name, 'box': [x - grow, y - grow, w + 2 * grow, h + 2 * grow],
            'depth': 250.0}


def _report(objects, target=None, roi=ROI, near=247.0, median=280.0, plane=280.0):
    return {'plane_distance': plane, 'roi': roi, 'shapes': ['sphere'], 'near': near,
            'median': median, 'objects': objects, 'target': target}


def _status(levels):
    return [level.status for level in levels]


def test_answer_key_passes_every_level():
    objects = [_obj('cylinder'), _obj('cuboid'), _obj(TARGET)]
    levels = shape_check.evaluate(_report(objects, target=2))
    assert _status(levels) == ['pass', 'pass', 'pass']
    assert 'ผ่านครบทุกด่าน' in shape_check.format_report(levels)


def test_floor_seen_as_one_object_fails_level_one_with_the_floor_depth():
    floor = {'kind': 'cuboid', 'box': [150, 0, 350, 350], 'depth': 279.0}
    levels = shape_check.evaluate(_report([floor], target=0, plane=350.0))
    assert _status(levels) == ['fail', 'skip', 'skip']
    text = shape_check.format_report(levels)
    assert 'พื้น' in text and '280' in text


def test_nothing_seen_fails_level_one():
    levels = shape_check.evaluate(_report([], plane=240.0))
    assert _status(levels) == ['fail', 'skip', 'skip']
    assert 'ไม่เห็น' in shape_check.format_report(levels)


def test_only_the_legs_seen_fails_level_one():
    leg = {'kind': 'cuboid', 'box': [0, 223, 50, 79], 'depth': 181.0}
    levels = shape_check.evaluate(_report([leg], target=0, roi=[5, 350, 0, 640], near=150.0))
    assert _status(levels) == ['fail', 'skip', 'skip']
    assert 'ขา' in shape_check.format_report(levels)


def test_missing_sphere_fails_level_two():
    levels = shape_check.evaluate(_report([_obj('cylinder'), _obj('cuboid')], target=0,
                                          roi=[120, 350, 150, 500]))
    assert _status(levels) == ['pass', 'fail', 'skip']
    assert 'ขอบบน' in shape_check.format_report(levels)


def test_misnamed_object_fails_level_two():
    objects = [_obj('cylinder'), _obj('cuboid'), _obj(TARGET, kind='cylinder')]
    levels = shape_check.evaluate(_report(objects, target=0))
    assert _status(levels) == ['pass', 'fail', 'skip']
    assert shape_check.THAI[TARGET] in shape_check.format_report(levels)


def test_a_floor_strip_beside_the_objects_fails_level_two():
    strip = {'kind': 'cuboid', 'box': [150, 318, 350, 32], 'depth': 274.0}
    objects = [_obj('cylinder'), _obj('cuboid'), _obj(TARGET), strip]
    levels = shape_check.evaluate(_report(objects, target=2))
    assert _status(levels) == ['pass', 'fail', 'skip']


def test_a_box_much_bigger_than_the_object_does_not_count_as_it():
    objects = [_obj('cylinder', grow=60), _obj('cuboid'), _obj(TARGET)]
    levels = shape_check.evaluate(_report(objects, target=2))
    assert levels[1].status == 'fail'


def test_wrong_target_fails_level_three():
    objects = [_obj('cylinder'), _obj('cuboid'), _obj(TARGET)]
    levels = shape_check.evaluate(_report(objects, target=0))
    assert _status(levels) == ['pass', 'pass', 'fail']
    assert shape_check.THAI['cylinder'] in shape_check.format_report(levels)


def test_no_target_fails_level_three():
    objects = [_obj('cylinder'), _obj('cuboid'), _obj(TARGET)]
    levels = shape_check.evaluate(_report(objects, target=None))
    assert _status(levels) == ['pass', 'pass', 'fail']


def test_validate_accepts_the_answer_key():
    shape_check.validate({'plane_distance': 280.0, 'roi': [0, 350, 150, 500],
                          'shapes': ['sphere']}, 'f.yaml')


@pytest.mark.parametrize('values, key', [
    ({'roi': [0, 350, 150]}, 'roi'),
    ({'roi': [350, 0, 150, 500]}, 'roi'),
    ({'roi': [0, 350, 150, 700]}, 'roi'),
    ({'shapes': ['ball']}, 'shapes'),
    ({'plane_distance': -5.0}, 'plane_distance'),
])
def test_validate_refuses_impossible_values(values, key):
    base = {'plane_distance': 280.0, 'roi': [0, 350, 150, 500], 'shapes': ['sphere']}
    with pytest.raises(ChallengeConfigError, match=key):
        shape_check.validate(dict(base, **values), 'f.yaml')
