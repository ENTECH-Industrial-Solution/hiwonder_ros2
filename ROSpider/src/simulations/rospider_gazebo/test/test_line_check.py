"""line_check: follow a run along worlds/line_track.sdf's loop from odometry samples."""
import math

import numpy as np
from rospider_gazebo import line_check
from rospider_gazebo.line_track import track_points

TRACK = track_points()
FORBIDDEN = ('kp', 'speed', 'max_turn', 'threshold')


def _drive(fraction_path, seconds, offset=None):
    """Feed a run that covers `fraction_path` (list of lap fractions) over `seconds`;
    offset(f) -> sideways metres from the line at lap fraction f."""
    tracker = line_check.LapTracker()
    times = np.linspace(0.0, seconds, len(fraction_path))
    for f, t in zip(fraction_path, times):
        i = int(round(f * len(TRACK))) % len(TRACK)
        x, y = TRACK[i]
        if offset:
            nx, ny = TRACK[(i + 1) % len(TRACK)] - TRACK[i - 1]
            norm = math.hypot(nx, ny)
            x, y = x - ny / norm * offset(f), y + nx / norm * offset(f)
        tracker.feed(float(x), float(y), float(t))
    return tracker


def _statuses(levels):
    return [level.status for level in levels]


def test_a_full_lap_in_time_passes():
    tracker = _drive(np.linspace(0, 1.02, 800), line_check.LAP_LIMIT * 0.8)
    levels = line_check.evaluate(tracker.summary())
    assert _statuses(levels) == ['pass'] * 3
    assert 'ผ่านครบทุกด่าน' in line_check.format_report(levels)


def test_a_slow_lap_fails_level_three():
    tracker = _drive(np.linspace(0, 1.02, 800), line_check.LAP_LIMIT * 1.3)
    assert _statuses(line_check.evaluate(tracker.summary())) == ['pass', 'pass', 'fail']


def test_leaving_the_line_early_fails_level_one():
    tracker = _drive(np.linspace(0, 0.3, 300), 60.0, offset=lambda f: 0.0 if f < 0.1 else 0.4)
    summary = tracker.summary()
    assert summary['lost'] and summary['reached'] < line_check.QUARTER
    assert _statuses(line_check.evaluate(summary)) == ['fail', 'skip', 'skip']


def test_leaving_the_line_later_fails_level_two():
    tracker = _drive(np.linspace(0, 0.8, 600), 100.0, offset=lambda f: 0.0 if f < 0.6 else 0.4)
    summary = tracker.summary()
    assert 0.55 < summary['reached'] < 0.65
    assert _statuses(line_check.evaluate(summary)) == ['pass', 'fail', 'skip']


def test_a_robot_that_never_moves_is_reported_as_standing_still():
    tracker = line_check.LapTracker()
    x, y = TRACK[0]
    for t in range(0, 40):
        tracker.feed(float(x), float(y), float(t))
    assert tracker.done(40.0)
    summary = tracker.summary()
    assert summary['stalled']
    report = line_check.format_report(line_check.evaluate(summary))
    assert 'ไม่ออกเดิน' in report or 'หยุด' in report


def test_small_wobble_is_not_leaving_the_line():
    tracker = _drive(np.linspace(0, 1.02, 800), line_check.LAP_LIMIT * 0.8,
                     offset=lambda f: 0.05 * math.sin(40 * f))
    assert not tracker.summary()['lost']


def test_hints_name_no_parameter():
    runs = [_drive(np.linspace(0, 0.3, 300), 60.0, offset=lambda f: 0.0 if f < 0.1 else 0.4),
            _drive(np.linspace(0, 0.8, 600), 100.0, offset=lambda f: 0.0 if f < 0.6 else 0.4),
            _drive(np.linspace(0, 1.02, 800), line_check.LAP_LIMIT * 1.3)]
    for tracker in runs:
        report = line_check.format_report(line_check.evaluate(tracker.summary()))
        assert not any(word in report for word in FORBIDDEN), report


def test_shipped_files_hold_the_measured_values():
    import pathlib
    import yaml
    config = pathlib.Path(__file__).resolve().parents[1] / 'config'
    def values(name):
        return yaml.safe_load((config / name).read_text())['/**']['ros__parameters']
    # measured: kp 0.3 stops at 6-10% of the lap (level 1); speed 0.03 laps in 237 s (level 3);
    # max_turn 0.1 or threshold 0.05-0.1 alone still lap in 144-148 s, so they ship right
    assert values('line_challenge.yaml') == {'threshold': 0.5, 'kp': 0.3, 'speed': 0.03, 'max_turn': 0.35}
    assert values('line_challenge_solved.yaml') == {'threshold': 0.5, 'kp': 1.1, 'speed': 0.05,
                                                    'max_turn': 0.35}


# ------------------------------------------------------------ review fixes

def test_a_run_that_runs_out_of_time_gets_the_slow_hint():
    # reviewer's case: at speed 0.02 the run stays on the line but hits RUN_LIMIT at ~85%
    tracker = _drive(np.linspace(0, 0.85, 700), line_check.RUN_LIMIT + 1.0)
    summary = tracker.summary()
    assert not summary['lost'] and not summary['stalled'] and summary['lap_time'] is None
    report = line_check.format_report(line_check.evaluate(summary))
    assert 'ช้า' in report and 'หมุน' not in report


def test_a_very_slow_run_timing_out_early_also_gets_the_slow_hint():
    tracker = _drive(np.linspace(0, 0.21, 300), line_check.RUN_LIMIT + 1.0)
    report = line_check.format_report(line_check.evaluate(tracker.summary()))
    assert 'ช้า' in report and 'โค้งแรก' not in report
