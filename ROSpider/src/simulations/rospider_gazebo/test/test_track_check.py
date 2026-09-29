"""track_check: where the Color Tracking exercise's robot stopped, from odometry samples."""
import math

import numpy as np
from rospider_gazebo import track_check

FORBIDDEN = ('threshold', 'stop_y', 'max_speed', 'max_turn')


def _run(end, seconds, yaw_deg, moved=True):
    """Walk straight from the origin to `end` facing `yaw_deg`, then stand still for 8 s."""
    run = track_check.new_run()
    end = np.array(end)
    heading = math.radians(yaw_deg)
    for t in np.arange(0.0, seconds, 0.1):
        p = end * (t / seconds if moved else 0.0)
        run.feed(float(p[0]), float(p[1]), heading, float(t))
    for t in np.arange(seconds, seconds + 8.0, 0.1):
        run.feed(float(end[0]) if moved else 0.0, float(end[1]) if moved else 0.0, heading, float(t))
    return run


def _statuses(levels):
    return [level.status for level in levels]


def _report(run):
    levels = track_check.evaluate(run.summary())
    return levels, track_check.format_report(levels)


# measured stops: answer key (stop_y 400), orange ball (threshold 0.7), stop_y 300 and 200-ish
ANSWER = ((0.878, 0.256), 21.3, 20.0)


def test_answer_key_passes():
    levels, text = _report(_run(*ANSWER))
    assert _statuses(levels) == ['pass'] * 3
    assert 'ผ่านครบทุกด่าน' in text


def test_going_to_the_orange_ball_fails_level_one_and_names_it():
    levels, text = _report(_run((0.311, -0.070), 12.0, -17.0))
    assert _statuses(levels) == ['fail', 'skip', 'skip']
    assert 'ส้ม' in text


def test_backing_away_fails_level_one():
    """A band wide enough to take in the robot's own gripper: it backs off."""
    levels, text = _report(_run((-0.266, -0.077), 5.5, 32.0))
    assert _statuses(levels)[0] == 'fail'
    assert 'มือจับ' in text


def test_not_moving_fails_level_one():
    levels, text = _report(_run((0.0, 0.0), 1.0, 0.0, moved=False))
    assert _statuses(levels)[0] == 'fail'
    assert 'ไม่ขยับ' in text


def test_stopping_far_fails_level_two():
    levels, text = _report(_run((0.713, 0.198), 21.3, 19.0))       # stop_y 300: 0.64 m
    assert _statuses(levels) == ['pass', 'fail', 'skip']
    assert '0.64' in text


def test_slow_fails_level_three():
    levels, text = _report(_run(ANSWER[0], 75.0, ANSWER[2]))
    assert _statuses(levels) == ['pass', 'pass', 'fail']
    assert '75' in text


def test_hints_name_no_parameter():
    runs = [_run((0.311, -0.070), 12.0, -17.0), _run((0.713, 0.198), 21.3, 19.0),
            _run(ANSWER[0], 75.0, ANSWER[2]), _run((-0.266, -0.077), 5.5, 32.0)]
    for run in runs:
        text = _report(run)[1]
        assert not any(key in text for key in FORBIDDEN)


def test_turning_to_the_orange_ball_without_walking_names_it():
    """The starting file: the orange ball already sits at the stop row, so the robot only turns."""
    run = track_check.new_run()
    for t in np.arange(0.0, 25.0, 0.1):
        run.feed(0.0, 0.0, math.radians(-min(t, 3.0) * 6.0), float(t))
    levels, text = _report(run)
    assert _statuses(levels)[0] == 'fail'
    assert 'หันไปหาลูกบอลสีส้ม' in text
