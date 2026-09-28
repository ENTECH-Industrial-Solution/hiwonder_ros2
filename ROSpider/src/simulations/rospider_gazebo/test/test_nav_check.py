from rospider_gazebo import nav_check
from rospider_gazebo.nav_check import COURSE_LIMIT, GoalResult


def _ok(seconds=20.0):
    return [GoalResult(name, 'succeeded', seconds) for name, _, _, _ in nav_check.COURSE]


def _statuses(levels):
    return [level.status for level in levels]


def test_whole_course_in_time_passes():
    levels = nav_check.evaluate(_ok(COURSE_LIMIT / 4))
    assert _statuses(levels) == ['pass'] * 3
    assert 'ผ่านครบทุกด่าน' in nav_check.format_report(levels)


def test_first_goal_never_finished_fails_level_one():
    levels = nav_check.evaluate([GoalResult('G1', 'timeout', 120.0)])
    assert _statuses(levels) == ['fail', 'skip', 'skip']
    assert 'ผ่านด่าน 1 ก่อน' in nav_check.format_report(levels)


def test_no_path_to_room_b_fails_level_two():
    results = [GoalResult('G1', 'succeeded', 30.0), GoalResult('G2', 'no_path', 0.0)]
    levels = nav_check.evaluate(results)
    assert _statuses(levels) == ['pass', 'fail', 'skip']
    assert 'costmap' in nav_check.format_report(levels)


def test_over_time_fails_level_three():
    levels = nav_check.evaluate(_ok(COURSE_LIMIT / 2))
    assert _statuses(levels) == ['pass', 'pass', 'fail']
    assert levels[2].numbers['total'] > COURSE_LIMIT


def test_last_goal_failing_fails_level_three():
    results = _ok()[:2] + [GoalResult('G3', 'aborted', 90.0)]
    assert _statuses(nav_check.evaluate(results)) == ['pass', 'pass', 'fail']


def test_each_goal_has_its_own_timeout():
    timeouts = [timeout for _, _, _, timeout in nav_check.COURSE]
    assert timeouts[0] < timeouts[1]        # a circling robot is caught at G1 quickly
    assert all(t <= COURSE_LIMIT for t in timeouts)


def test_cut_at_the_course_limit_is_reported_as_slow():
    results = _ok(60.0)[:2] + [GoalResult('G3', 'over_limit', COURSE_LIMIT - 120.0)]
    levels = nav_check.evaluate(results)
    assert [level.status for level in levels] == ['pass', 'pass', 'fail']
    assert 'ช้า' in nav_check.format_report(levels)
    results = [GoalResult('G1', 'succeeded', 100.0), GoalResult('G2', 'over_limit', COURSE_LIMIT - 100.0)]
    levels = nav_check.evaluate(results)
    assert [level.status for level in levels] == ['pass', 'fail', 'skip']
    assert 'ช้า' in nav_check.format_report(levels)


def test_rejected_goal_is_a_readable_failure():
    levels = nav_check.evaluate([GoalResult('G1', 'rejected', 0.0)])
    assert levels[0].status == 'fail'
    assert 'launch' in nav_check.format_report(levels)


def test_missing_results_are_failures():
    assert _statuses(nav_check.evaluate([])) == ['fail', 'skip', 'skip']
    assert _statuses(nav_check.evaluate(_ok()[:1])) == ['pass', 'fail', 'skip']


def test_hints_name_no_parameter():
    cases = [
        [GoalResult('G1', 'timeout', 120.0)],
        [GoalResult('G1', 'succeeded', 30.0), GoalResult('G2', 'no_path', 0.0)],
        _ok(COURSE_LIMIT / 2),
    ]
    for results in cases:
        text = nav_check.format_report(nav_check.evaluate(results))
        for key in ('xy_goal_tolerance', 'robot_radius', 'max_vel_x', 'inflation_radius'):
            assert key not in text
