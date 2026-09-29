"""tag_check: where the AprilTag exercise's robot stopped, from odometry samples."""
import math

import numpy as np
import pytest
from rospider_gazebo import tag_check

FORBIDDEN = ('target_tag', 'stop_distance', 'speed_limit', 'turn_limit')


def _run(end, seconds, yaw_deg=None, moved=True):
    """Walk straight from the origin to `end`, then stand still for 8 s."""
    run = tag_check.TagRun()
    start = np.array([0.0, 0.0])
    end = np.array(end)
    heading = math.atan2(end[1], end[0]) if yaw_deg is None else math.radians(yaw_deg)
    for t in np.arange(0.0, seconds, 0.1):
        p = start + (end - start) * (t / seconds if moved else 0.0)
        run.feed(float(p[0]), float(p[1]), heading, float(t))
    for t in np.arange(seconds, seconds + 8.0, 0.1):
        run.feed(float(end[0]), float(end[1]), heading, float(t))
    return run


def _statuses(levels):
    return [level.status for level in levels]


# measured stops (stop_distance 0.35 / 0.30 / 0.40, wrong tag, 0.8, speed 0.01)
ANSWER = ((0.691, -0.248), 15.3, -22.9)


def test_answer_key_passes():
    run = _run(ANSWER[0], ANSWER[1], ANSWER[2])
    assert run.done(run.last_t)
    levels = tag_check.evaluate(run.summary())
    assert _statuses(levels) == ['pass'] * 3
    assert 'ผ่านครบทุกด่าน' in tag_check.format_report(levels)


def test_the_measured_band_edges_pass():
    for end, seconds, yaw in (((0.739, -0.269), 16.1, -23.2), ((0.643, -0.228), 14.2, -22.7)):
        assert _statuses(tag_check.evaluate(_run(end, seconds, yaw).summary())) == ['pass'] * 3


def test_the_wrong_tag_barely_moving_fails_level_one():
    levels = tag_check.evaluate(_run((0.115, 0.0), 2.3, 0.0).summary())
    assert _statuses(levels) == ['fail', 'skip', 'skip']


def test_walking_to_another_tag_fails_level_one_and_names_it():
    board0 = tag_check.board(0)
    levels = tag_check.evaluate(_run((board0[0] - 0.47, board0[1]), 15.0).summary())
    assert levels[0].status == 'fail' and levels[0].numbers['faced'] == 0
    assert 'หมายเลข 0' in tag_check.format_report(levels)


def test_stopping_far_fails_level_two():
    levels = tag_check.evaluate(_run((0.264, -0.074), 5.9, -21.6).summary())
    assert _statuses(levels) == ['pass', 'fail', 'skip']
    assert 'ห่าง' in tag_check.format_report(levels)


def test_slow_fails_level_three():
    levels = tag_check.evaluate(_run((0.688, -0.256), 73.0, -21.9).summary())
    assert _statuses(levels) == ['pass', 'pass', 'fail']


def test_not_moving_at_all_is_reported():
    run = _run((0.0, 0.0), 30.0, moved=False)
    assert run.done(run.last_t)
    levels = tag_check.evaluate(run.summary())
    assert levels[0].status == 'fail' and 'ไม่ขยับ' in tag_check.format_report(levels)


def test_facing_away_at_the_right_spot_does_not_pass():
    # turned 40 degrees off it faces another tag's board: level 1 says so
    levels = tag_check.evaluate(_run(ANSWER[0], ANSWER[1], ANSWER[2] + 40.0).summary())
    assert levels[0].status == 'fail' and levels[0].numbers['faced'] != tag_check.TARGET


def test_slightly_off_the_board_at_the_right_spot_fails_level_two():
    levels = tag_check.evaluate(_run(ANSWER[0], ANSWER[1], ANSWER[2] + 18.0).summary())
    assert _statuses(levels) == ['pass', 'fail', 'skip']


def test_hints_name_no_parameter():
    runs = [_run((0.115, 0.0), 2.3, 0.0), _run((0.264, -0.074), 5.9, -21.6),
            _run((0.688, -0.256), 73.0, -21.9), _run((0.0, 0.0), 30.0, moved=False)]
    for run in runs:
        report = tag_check.format_report(tag_check.evaluate(run.summary()))
        assert not any(word in report for word in FORBIDDEN), report


def test_a_short_step_is_not_called_going_to_another_tag():
    # measured with the starting file: target tag 1 moved 0.1 m and stopped facing tag 1; the
    # nearest board was then tag 0's by a few centimetres, which is not what it followed
    report = tag_check.format_report(tag_check.evaluate(_run((0.115, 0.01), 2.3, 0.0).summary()))
    assert 'หมายเลข 1' in report and 'หมายเลข 0' not in report



# ------------------------------------------------------------ review fixes

def _towards_target(metres):
    """A point `metres` from the start along the line to tag 2's board (negative: backwards)."""
    bx, by = tag_check.board(tag_check.TARGET)
    norm = math.hypot(bx, by)
    return (bx / norm * metres, by / norm * metres), math.degrees(math.atan2(by, bx))


def test_right_tag_but_a_short_walk_is_a_distance_problem_not_a_tag_problem():
    # reviewer's case: stop_distance 0.95 walks only ~0.1 m toward tag 2
    end, yaw = _towards_target(0.1)
    levels = tag_check.evaluate(_run(end, 3.0, yaw).summary())
    assert _statuses(levels) == ['pass', 'fail', 'skip']
    assert 'ห่าง' in tag_check.format_report(levels)


def test_right_tag_backing_away_is_a_distance_problem():
    # reviewer's case: stop_distance 35 (centimetres) backs the robot away, still facing tag 2
    end, yaw = _towards_target(-0.4)
    levels = tag_check.evaluate(_run(end, 8.0, yaw).summary())
    assert _statuses(levels) == ['pass', 'fail', 'skip']
    assert 'หมายเลข' not in tag_check.format_report(levels).split('\n')[1]


def test_still_walking_when_time_is_up_gets_a_speed_hint():
    # reviewer's case: speed 0.005 is still walking at RUN_LIMIT, 0.61 m from the board
    end, yaw = _towards_target(0.6)
    run = tag_check.TagRun()
    for t in np.arange(0.0, tag_check.RUN_LIMIT + 1.0, 0.1):
        f = t / (tag_check.RUN_LIMIT + 1.0)
        run.feed(end[0] * f, end[1] * f, math.radians(yaw), float(t))
    assert run.done(run.last_t)
    levels = tag_check.evaluate(run.summary())
    report = tag_check.format_report(levels)
    assert levels[1].status == 'fail' and 'ยังไม่หยุด' in report and 'เร็ว' in report


def test_turning_to_the_right_tag_without_walking_gets_a_speed_hint():
    # speed 0.001: the robot turns to face tag 2 but barely moves in 20 s
    _, yaw = _towards_target(0.0)
    run = tag_check.TagRun()
    for t in np.arange(0.0, tag_check.NEVER_MOVED_S + 1.0, 0.1):
        run.feed(0.0, 0.0, math.radians(yaw), float(t))
    levels = tag_check.evaluate(run.summary())
    assert _statuses(levels) == ['pass', 'fail', 'skip']
    assert 'เร็ว' in tag_check.format_report(levels)


def test_the_hint_does_not_blame_occlusion():
    report = tag_check.format_report(tag_check.evaluate(_run((0.115, 0.0), 2.3, 0.0).summary()))
    assert 'บัง' not in report


def test_a_run_can_be_scored_against_other_targets():
    """track_check reuses TagRun with its balls: points by name, and how much closer the robot got."""
    run = tag_check.TagRun(targets={'red': (1.0, 0.0), 'orange': (0.0, 1.0)}, target='red')
    run.feed(0.0, 0.0, 0.0, 0.0)
    run.feed(0.4, 0.0, 0.0, 1.0)
    summary = run.summary()
    assert summary['faced'] == 'red'
    assert summary['distance'] == pytest.approx(0.6)
    assert summary['closer'] == pytest.approx(0.4)


def test_summary_says_how_far_the_robot_turned():
    run = tag_check.TagRun()
    run.feed(0.0, 0.0, math.radians(170.0), 0.0)
    run.feed(0.0, 0.0, math.radians(-160.0), 1.0)
    assert run.summary()['turned'] == pytest.approx(30.0)
