"""grasp_check: the 3D Object Grasping exercise's levels, from the end of a run."""
import math

from rospider_gazebo import grasp_check

FORBIDDEN = ('walk', 'target_x', 'place_point')
ALL = ['IDLE', 'LOOK', 'LOCALIZE', 'PRE_GRASP', 'DESCEND', 'GRASP', 'LIFT', 'CARRY',
       'TO_DROP', 'LOWER', 'RELEASE', 'RETREAT', 'IDLE']
# measured: the answer key's stop and where it put the block (twice)
STOP = (-0.076, -0.523, math.radians(-74.0))


def _levels(robot, states, block):
    levels = grasp_check.evaluate(grasp_check.summarise(robot, states, block))
    return [level.status for level in levels], grasp_check.format_report(levels)


def test_parse_gz_pose():
    text = ('Requesting state for world [rospider_room]...\n\nModel: [170]\n  - Name: pick_cube_blue\n'
            '  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n    [-0.233813 -0.583779 0.025000]\n'
            '    [-0.000001 -0.000000 -1.858190]\n')
    assert grasp_check.parse_gz_pose(text) == (-0.233813, -0.583779, 0.025)
    assert grasp_check.parse_gz_pose('Model not found') is None


def test_answer_key_passes():
    for block in ((-0.234, -0.584, 0.025), (-0.226, -0.575, 0.025)):
        statuses, text = _levels(STOP, ALL, block)
        assert statuses == ['pass'] * 3
        assert 'ผ่านครบทุกด่าน' in text


def test_never_setting_off_fails_level_one():
    statuses, text = _levels((0.0, 0.0, 0.0), ['IDLE'], (0.0, -0.8, 0.105))
    assert statuses == ['fail', 'skip', 'skip']
    assert 'ไม่ได้ออกเดิน' in text


def test_standing_off_and_not_picking_fails_level_two():
    statuses, text = _levels((-0.10, -0.40, math.radians(-75.0)), ['IDLE'], (0.0, -0.8, 0.105))
    assert statuses == ['pass', 'fail', 'skip']
    assert 'ไม่ยอมหยิบ' in text


def test_a_grasp_that_misses_fails_level_two():
    statuses, text = _levels(STOP, ['IDLE', 'LOOK', 'LOCALIZE', 'IDLE'], (0.0, -0.8, 0.105))
    assert statuses == ['pass', 'fail', 'skip']
    assert 'จับบล็อกไม่ได้' in text


def test_putting_it_down_on_the_left_fails_level_three_and_says_where():
    statuses, text = _levels(STOP, ALL, (0.090, -0.496, 0.025))
    assert statuses == ['pass', 'pass', 'fail']
    assert 'ทางขวา' in text and 'ทางซ้าย' in text


def test_a_block_left_up_high_is_not_on_the_pad():
    statuses, _text = _levels(STOP, ALL, (-0.23, -0.58, 0.105))
    assert statuses[2] == 'fail'


def test_hints_name_no_parameter():
    cases = [((0.0, 0.0, 0.0), ['IDLE'], None),
             ((-0.10, -0.40, math.radians(-75.0)), ['IDLE'], None),
             (STOP, ['IDLE', 'LOOK', 'IDLE'], None),
             (STOP, ALL, (0.090, -0.496, 0.025))]
    for robot, states, block in cases:
        assert not any(key in _levels(robot, states, block)[1] for key in FORBIDDEN)


def test_place_point_is_three_numbers():
    import pytest
    from rospider_gazebo.challenge_params import ChallengeConfigError
    assert grasp_check.parse_point('0.02 -0.16 0.035', 'f.yaml') == (0.02, -0.16, 0.035)
    for bad in ('0.02 -0.16', '0.02, -0.16, 0.035', 'a b c'):
        with pytest.raises(ChallengeConfigError, match='place_point'):
            grasp_check.parse_point(bad, 'f.yaml')


def test_stopping_short_of_the_reach_is_a_level_two_problem():
    """Measured: target_x 0.40 stops the base 0.46 m from the block, facing it, and never picks."""
    robot = (-0.1, -0.35, math.radians(-78.0))
    assert math.dist(robot[:2], grasp_check.BLOCK[:2]) > 0.45
    statuses, text = _levels(robot, ['IDLE'], (0.0, -0.8, 0.105))
    assert statuses == ['pass', 'fail', 'skip']
