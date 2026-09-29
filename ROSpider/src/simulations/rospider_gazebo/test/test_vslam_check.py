import math
import sqlite3
import struct

import numpy as np
import pytest
from rospider_gazebo import vslam_check
from rospider_gazebo.slam_check import FREE, OCCUPIED, UNKNOWN, GridMap

RES, ORIGIN = 0.05, (-1.2, -2.2)
HINT_FORBIDDEN = ('camera_view', 'Grid/', 'Vis/', 'RangeMax', 'MinInliers')


def _room(walls=True, inside_known=True):
    """5 x 4 m room, 0.1 m wall bands, free inside, unknown outside."""
    xs = ORIGIN[0] + (np.arange(108) + 0.5) * RES
    ys = ORIGIN[1] + (np.arange(88) + 0.5) * RES
    x, y = np.meshgrid(xs, ys)
    inside = (x > -1.05) & (x < 4.05) & (y > -2.05) & (y < 2.05)
    wall = inside & ((np.abs(x + 1.0) < .05) | (np.abs(x - 4.0) < .05)
                     | (np.abs(y + 2.0) < .05) | (np.abs(y - 2.0) < .05))
    states = np.full(x.shape, UNKNOWN, dtype=np.int8)
    if inside_known:
        states[inside] = FREE
    if walls:
        states[wall] = OCCUPIED
    return GridMap(states, RES, ORIGIN)


def _pose(x, y, yaw_deg):
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    m = np.eye(4)
    m[:2, :2] = [[c, -s], [s, c]]
    m[0, 3], m[1, 3] = x, y
    return m


# (from, to, metres, degrees, back at the start) as closure_errors returns
GOOD = [(205, 1, 0.01, 0.5, True)]
WRONG = [(97, 22, 1.2, 1.8, False)]


def _statuses(levels):
    return [level.status for level in levels]


def test_answer_key_passes_all():
    levels = vslam_check.evaluate(_room(), _room(), GOOD)
    assert _statuses(levels) == ['pass'] * 3
    assert 'ผ่านครบทุกด่าน' in vslam_check.format_report(levels)


def test_tiny_map_fails_level_one():
    tiny = _room(walls=False, inside_known=False)
    tiny.states[40:50, 20:30] = FREE                     # 0.25 m2
    assert _statuses(vslam_check.evaluate(tiny, _room(), GOOD)) == ['fail', 'skip', 'skip']


def test_missing_map_report():
    report = vslam_check.format_report(vslam_check.evaluate(None, _room(), None, missing=True))
    assert 'ไม่พบ' in report and 'map:=' in report


def test_floor_only_map_fails_level_one_on_walls():
    levels = vslam_check.evaluate(_room(walls=False), _room(), GOOD)
    assert _statuses(levels) == ['fail', 'skip', 'skip']
    assert levels[0].numbers['walls'] < vslam_check.MIN_WALLS
    assert 'กล้อง' in vslam_check.format_report(levels)


def _room_a_only():
    """Only x < 1 explored: its floor and walls known, the rest unknown."""
    grid = _room()
    x, _ = grid.cell_centres()
    grid.states[x > 1.0] = UNKNOWN
    return grid


def test_one_room_only_fails_level_two_not_one():
    levels = vslam_check.evaluate(_room_a_only(), _room(), GOOD)
    assert _statuses(levels) == ['pass', 'fail', 'skip']
    assert levels[0].numbers['walls'] > 0.9          # the walls it saw are there
    assert 'ห้อง' in vslam_check.format_report(levels)


def test_small_registration_error_is_not_a_wrong_closure():
    # measured with the answer key: 0.12 m / 2.9 deg (depth registration noise);
    # the aliasing closures that bend the map were 1.0 m and more
    levels = vslam_check.evaluate(_room(), _room(), GOOD + [(96, 22, 0.12, 2.9, False)])
    assert levels[2].status == 'pass'


def test_no_loop_closure_fails_level_three():
    levels = vslam_check.evaluate(_room(), _room(), [])
    assert _statuses(levels) == ['pass', 'pass', 'fail']
    assert 'กลับมา' in vslam_check.format_report(levels)


def test_wrong_loop_closure_fails_level_three():
    levels = vslam_check.evaluate(_room(), _room(), GOOD + WRONG)
    assert _statuses(levels) == ['pass', 'pass', 'fail']
    assert levels[2].numbers['wrong'] == 1


def test_thick_walls_fail_level_three():
    thick = _room()
    thick.states[(thick.states == FREE) & (np.arange(108)[None, :] % 9 == 0)] = OCCUPIED
    levels = vslam_check.evaluate(thick, _room(), GOOD)
    assert levels[2].status == 'fail' and levels[2].numbers['ratio'] > vslam_check.MAX_THICKNESS


def test_hints_name_no_parameter():
    cases = [vslam_check.evaluate(None, _room(), None, missing=True),
             vslam_check.evaluate(_room(walls=False, inside_known=False), _room(), None),
             vslam_check.evaluate(_room(walls=False), _room(), GOOD),
             vslam_check.evaluate(_room(), _room(), []),
             vslam_check.evaluate(_room(), _room(), WRONG)]
    for levels in cases:
        report = vslam_check.format_report(levels)
        assert not any(word in report for word in HINT_FORBIDDEN), report


def test_wall_recall_tolerates_a_small_shift():
    shifted = _room()
    shifted.states = np.roll(shifted.states, 1, axis=1)          # 5 cm
    assert vslam_check.wall_recall(shifted, _room()) == pytest.approx(1.0, abs=0.02)
    assert vslam_check.wall_recall(_room(walls=False), _room()) == 0.0
    assert vslam_check.wall_recall(_room_a_only(), _room()) == pytest.approx(1.0, abs=0.02)


def test_closure_errors_compare_link_with_odometry():
    poses = {1: _pose(0, 0, 0), 5: _pose(1.0, 0.5, 90)}
    rel = np.linalg.inv(poses[5]) @ poses[1]
    right = (5, 1, 1, rel)
    wrong = (5, 1, 1, rel @ _pose(1.2, 0, 0))
    errors = vslam_check.closure_errors(poses, [right, wrong])
    assert errors[0][2] == pytest.approx(0.0, abs=1e-9)
    assert errors[1][2] == pytest.approx(1.2, abs=1e-6)


def test_only_a_late_closure_at_the_start_counts_as_back_at_the_start():
    # measured in the answer key: closures also form inside room B (97 -> 22) and while the robot
    # stands at the spawn point (2 -> 1); only one made after the route, near node 1, is a return
    poses = {1: _pose(0, 0, 0), 2: _pose(0, 0, 0), 22: _pose(1.45, 0.5, 0),
             97: _pose(3.2, 0, 90), 221: _pose(0.02, 0.01, 0)}
    def link(f, t):
        return (f, t, 1, np.linalg.inv(poses[f]) @ poses[t])
    flags = [e[4] for e in vslam_check.closure_errors(poses, [link(2, 1), link(97, 22), link(221, 1)])]
    assert flags == [False, False, True]


def test_closures_without_a_return_fail_level_three_with_the_return_hint():
    levels = vslam_check.evaluate(_room(), _room(), [(97, 22, 0.02, 0.5, False), (2, 1, 0.0, 0.0, False)])
    assert levels[2].status == 'fail' and levels[2].numbers['returns'] == 0
    assert 'กลับมา' in vslam_check.format_report(levels)


def _blob(m):
    return struct.pack('12f', *m[:3, :4].reshape(-1))


def test_read_graph_reads_rtabmap_tables(tmp_path):
    db = tmp_path / 'm.db'
    con = sqlite3.connect(db)
    con.execute('CREATE TABLE Node (id INTEGER, pose BLOB)')
    con.execute('CREATE TABLE Link (from_id INTEGER, to_id INTEGER, type INTEGER, transform BLOB)')
    con.executemany('INSERT INTO Node VALUES (?, ?)', [(1, _blob(_pose(0, 0, 0))),
                                                       (2, _blob(_pose(1, 0, 0)))])
    con.executemany('INSERT INTO Link VALUES (?, ?, ?, ?)', [
        (2, 1, 0, _blob(_pose(-1, 0, 0))), (1, 2, 0, _blob(_pose(1, 0, 0))),   # neighbours
        (2, 1, 1, _blob(_pose(-1, 0, 0))), (1, 2, 1, _blob(_pose(1, 0, 0))),   # one closure, both ways
        (2, 2, 9, _blob(_pose(0, 0, 0)))])                                       # gravity
    con.commit()
    con.close()
    poses, links = vslam_check.read_graph(str(db))
    assert set(poses) == {1, 2} and poses[2][0, 3] == pytest.approx(1.0)
    assert [(f, t, k) for f, t, k, _ in links] == [(2, 1, 1)]


def test_db_in_use_scans_open_files(tmp_path):
    db = tmp_path / 'm.db'
    db.write_bytes(b'')
    proc = tmp_path / 'proc'
    (proc / '123' / 'fd').mkdir(parents=True)
    (proc / 'self').mkdir()
    assert not vslam_check.db_in_use(str(db), str(proc))
    (proc / '123' / 'fd' / '7').symlink_to(db)
    assert vslam_check.db_in_use(str(db), str(proc))


def test_level_one_failure_also_reports_wrong_closures():
    # measured: the camera tilted at the floor also made 9-14 wrong closures (the floor looks
    # the same everywhere), so level 1 says so alongside the camera hint
    levels = vslam_check.evaluate(_room(walls=False), _room(), GOOD + WRONG)
    report = vslam_check.format_report(levels)
    assert levels[0].status == 'fail' and 'จำผิดที่ 1 ครั้ง' in report
    quiet = vslam_check.format_report(vslam_check.evaluate(_room(walls=False), _room(), GOOD))
    assert 'จำผิดที่' not in quiet
