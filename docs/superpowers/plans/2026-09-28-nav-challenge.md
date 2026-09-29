# Nav2 Challenge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A 10-15 minute workshop exercise: participants fix three wrong Nav2 values in one YAML file, and an automatic checker drives a fixed three-goal course through the SLAM exercise room and scores it in three levels.

**Architecture:** Two pure-Python modules hold the logic and are unit tested: `nav_params.py` maps six friendly keys onto every nested path they occupy in `config/nav2_params.yaml` (validation reused from `challenge_params`), and `nav_check.py` holds the course, the level logic and the Thai report. A launch file and a checker CLI are thin wrappers; the launch reuses `navigation.launch.py` (which already takes `world:=` and `map:=` by name).

**Tech Stack:** ROS 2 Jazzy, Nav2 (nav2_bringup, DWB, NavFn), Gazebo Harmonic, Python 3.12, PyYAML, rclpy actions, pytest.

**Spec:** `docs/superpowers/specs/2026-09-28-nav-challenge-design.md`

## Global Constraints

- Package root: `ROSpider/src/simulations/rospider_gazebo/` (paths below are relative to it unless they start with `ROSpider/` or `docs/`).
- Build/run from `ROSpider/`: `source /opt/ros/jazzy/setup.bash`, `colcon build --packages-select rospider_gazebo --symlink-install`, `source install/local_setup.bash`, `export need_compile=True`.
- Room, map and spawn are the SLAM exercise's: `worlds/slam_challenge.sdf`, `maps/slam_challenge.yaml`, robot at (0, 0) yaw 0 = map origin.
- Participant file: `config/nav_challenge.yaml`, edited in place (like `config/slam_challenge.yaml`); `params:=<path>` uses another file (copied from the template if missing).
- Starting values: `xy_goal_tolerance: 0.002`, `robot_radius: 0.3`, `max_vel_x: 0.03`; distractors `inflation_radius: 0.15`, `yaw_goal_tolerance: 0.1`, `max_vel_theta: 0.25`. Solved: `0.05`, `0.15`, `0.15`, distractors unchanged. All draft until Task 5 measures them.
- Course (map frame): G1 (-0.4, -1.3), G2 (3.3, -0.4), G3 (0.0, 0.0); per-goal timeout 120 s; planner check 15 s; server wait 30 s; times on the sim clock.
- Levels: 1 = G1 succeeded; 2 = G2 succeeded; 3 = all three succeeded and total ≤ `COURSE_LIMIT` (set in Task 5 from measurements). Unjudgeable levels say which level to pass first.
- Learner-facing text is Thai; hints describe symptoms and never name a parameter.
- No cv2 and no rclpy in `nav_params.py` / `nav_check.py`.
- The user wants ONE commit per finished feature: no per-task commits; Task 6 ends by asking.
- Unit tests: `cd ROSpider/src/simulations/rospider_gazebo && python3 -m pytest -q test/<file>.py`.

## Review Focus

- Only one of a meaning's paths changed (e.g. DWB `max_vel_x` but not the velocity smoother's cap) would leave the robot's behaviour unchanged — every key must reach every path. Test in Task 1 (`test_each_key_reaches_every_path`).
- `velocity_smoother.max_velocity` is a list; replacing the whole list with a scalar would crash the smoother. Test in Task 1 (`test_list_entries_are_replaced_by_index`).
- A key the participant does not set must keep `nav2_params.yaml`'s value, not the template's. Test in Task 1 (`test_keys_not_given_keep_nav2_values`).
- The checker started while Nav2 is not up must say so in Thai and exit 1, not hang or trace back. Checked in Task 3 Step 2.
- A goal Nav2 rejects, or a run cut short, must produce a failed level with a readable hint, not a crash in `evaluate`. Test in Task 2 (`test_rejected_goal_is_a_readable_failure`, `test_missing_results_are_failures`).

---

### Task 1: `nav_params.py` and the two config files

**Files:**
- Create: `rospider_gazebo/nav_params.py`
- Create: `config/nav_challenge.yaml`, `config/nav_challenge_solved.yaml`
- Create: `test/test_nav_params.py`
- Modify: `CMakeLists.txt` (register the test)

**Interfaces:**
- Consumes: `challenge_params.load_params(path) -> dict`, `challenge_params.merge(base, override, source) -> dict`, `challenge_params.ChallengeConfigError` (existing).
- Produces: `KEYS: dict[str, tuple[tuple, ...]]`; `defaults(nav2: dict) -> dict`; `apply(nav2: dict, overrides: dict, source: str) -> dict` (deep copy); `merged_params_file(nav2_path: str, user_path: str) -> str`.

- [ ] **Step 1: Write the failing tests**

Create `test/test_nav_params.py`:

```python
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


def _get(tree, path):
    for part in path:
        tree = tree[part]
    return tree


def test_every_path_exists_in_nav2_params():
    nav2 = _nav2()
    for key, paths in nav_params.KEYS.items():
        for path in paths:
            _get(nav2, path)            # KeyError/IndexError names the stale path


def test_each_key_reaches_every_path():
    values = {key: 0.123 + i for i, key in enumerate(nav_params.KEYS)}
    merged = nav_params.apply(_nav2(), values, 'f.yaml')
    for key, paths in nav_params.KEYS.items():
        for path in paths:
            assert _get(merged, path) == pytest.approx(values[key]), (key, path)


def test_list_entries_are_replaced_by_index():
    original = _get(_nav2(), ('velocity_smoother', 'ros__parameters', 'max_velocity'))
    merged = nav_params.apply(_nav2(), {'max_vel_x': 0.2}, 'f.yaml')
    velocity = _get(merged, ('velocity_smoother', 'ros__parameters', 'max_velocity'))
    assert velocity == [0.2, original[1], original[2]]


def test_keys_not_given_keep_nav2_values():
    nav2 = _nav2()
    merged = nav_params.apply(nav2, {'robot_radius': 0.3}, 'f.yaml')
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
    value = _get(merged, ('controller_server', 'ros__parameters', 'FollowPath', 'max_vel_x'))
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
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd ROSpider/src/simulations/rospider_gazebo && python3 -m pytest -q test/test_nav_params.py`
Expected: collection error, `ImportError: cannot import name 'nav_params'`.

- [ ] **Step 3: Write the implementation and the config files**

Create `rospider_gazebo/nav_params.py`:

```python
"""The Nav2 exercise's config: six friendly keys written into config/nav2_params.yaml.

Nav2's parameters are nested per node, and several meanings live in more
than one place -- the speed cap is in DWB and again in the velocity
smoother, the robot's size in both costmaps. Changing one copy would leave
the robot's behaviour as it was, so each friendly key is written to every
path it occupies (KEYS). Validation is challenge_params.merge's: unknown
keys, blank values and wrong types are refused; whole numbers become floats.
Pure Python (no cv2, no rclpy): nav_challenge.launch.py imports it.
"""

import copy
import tempfile

import yaml

from rospider_gazebo import challenge_params

#: friendly key -> every path it occupies in nav2_params.yaml. An int in a
#: path indexes a list (velocity_smoother.max_velocity is [x, y, theta]).
KEYS = {
    'xy_goal_tolerance': (
        ('controller_server', 'ros__parameters', 'general_goal_checker', 'xy_goal_tolerance'),
        ('controller_server', 'ros__parameters', 'FollowPath', 'xy_goal_tolerance'),
    ),
    'yaw_goal_tolerance': (
        ('controller_server', 'ros__parameters', 'general_goal_checker', 'yaw_goal_tolerance'),
    ),
    'robot_radius': (
        ('local_costmap', 'local_costmap', 'ros__parameters', 'robot_radius'),
        ('global_costmap', 'global_costmap', 'ros__parameters', 'robot_radius'),
    ),
    'inflation_radius': (
        ('local_costmap', 'local_costmap', 'ros__parameters', 'inflation_layer', 'inflation_radius'),
        ('global_costmap', 'global_costmap', 'ros__parameters', 'inflation_layer', 'inflation_radius'),
    ),
    'max_vel_x': (
        ('controller_server', 'ros__parameters', 'FollowPath', 'max_vel_x'),
        ('velocity_smoother', 'ros__parameters', 'max_velocity', 0),
    ),
    'max_vel_theta': (
        ('controller_server', 'ros__parameters', 'FollowPath', 'max_vel_theta'),
        ('velocity_smoother', 'ros__parameters', 'max_velocity', 2),
    ),
}


def _get(tree, path):
    for part in path:
        tree = tree[part]
    return tree


def _put(tree, path, value):
    _get(tree, path[:-1])[path[-1]] = value


def defaults(nav2):
    """Each friendly key's current value: the first path's, which also sets its type."""
    return {key: _get(nav2, paths[0]) for key, paths in KEYS.items()}


def apply(nav2, overrides, source):
    """A copy of the nav2 params tree with `overrides` written to every path."""
    values = challenge_params.merge(defaults(nav2), overrides, source)
    merged = copy.deepcopy(nav2)
    for key in overrides:
        for path in KEYS[key]:
            _put(merged, path, values[key])
    return merged


def merged_params_file(nav2_path, user_path):
    """Write nav2_params.yaml with the participant's values to a temp file; return its path."""
    with open(nav2_path) as handle:
        nav2 = yaml.safe_load(handle)
    merged = apply(nav2, challenge_params.load_params(user_path), user_path)
    with tempfile.NamedTemporaryFile('w', prefix='nav_challenge_', suffix='.yaml',
                                     delete=False) as handle:
        yaml.safe_dump(merged, handle)
        return handle.name
```

Create `config/nav_challenge.yaml`:

```yaml
# โจทย์ Nav2
# แก้ค่าในไฟล์นี้ แล้วปิด launch (Ctrl+C) และเปิดใหม่ด้วยคำสั่งเดิม
# ค่าที่ไม่ได้อยู่ในไฟล์นี้ใช้ของ config/nav2_params.yaml
# อยากเริ่มโจทย์ใหม่: git checkout -- ไฟล์นี้ แล้ว launch ใหม่
/**:
  ros__parameters:
    # ต้องเข้าใกล้เป้าหมายแค่ไหน (เมตร) ถึงนับว่าถึงแล้ว
    xy_goal_tolerance: 0.002
    # ต้องหันตรงทิศของเป้าหมายแค่ไหน (เรเดียน) ถึงนับว่าถึงแล้ว
    yaw_goal_tolerance: 0.1
    # รัศมีตัวหุ่นในแผนที่ (เมตร) - planner จะไม่ให้หุ่นเข้าใกล้ผนังกว่านี้
    robot_radius: 0.3
    # ระยะที่ costmap เริ่มกันหุ่นให้ห่างผนัง (เมตร)
    inflation_radius: 0.15
    # ความเร็วเดินหน้าสูงสุด (เมตร/วินาที)
    max_vel_x: 0.03
    # ความเร็วหมุนสูงสุด (เรเดียน/วินาที)
    max_vel_theta: 0.25
```

Create `config/nav_challenge_solved.yaml`:

```yaml
# เฉลยโจทย์ Nav2 (สำหรับวิทยากร)
# ros2 launch rospider_gazebo nav_challenge.launch.py params:=<path ของไฟล์นี้>
/**:
  ros__parameters:
    xy_goal_tolerance: 0.05
    yaw_goal_tolerance: 0.1
    robot_radius: 0.15
    inflation_radius: 0.15
    max_vel_x: 0.15
    max_vel_theta: 0.25
```

Add to `CMakeLists.txt` after the `test_maps` entry:

```cmake
  ament_add_pytest_test(test_nav_params test/test_nav_params.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd ROSpider/src/simulations/rospider_gazebo && python3 -m pytest -q test/test_nav_params.py`
Expected: 11 passed.

---

### Task 2: `nav_check.py` — course, levels, Thai report

**Files:**
- Create: `rospider_gazebo/nav_check.py`
- Create: `test/test_nav_check.py`
- Modify: `CMakeLists.txt` (register the test)

**Interfaces:**
- Consumes: `slam_check.Level(number: int, status: str, numbers: dict)` (existing).
- Produces: `COURSE: tuple[tuple[str, str, tuple[float, float]], ...]` (name, Thai label, (x, y)); `GOAL_TIMEOUT = 120.0`; `PLAN_TIMEOUT = 15.0`; `COURSE_LIMIT: float` (sim seconds); `@dataclass GoalResult(name: str, status: str, seconds: float)` with status in `'succeeded' | 'no_path' | 'aborted' | 'timeout' | 'canceled' | 'rejected'`; `evaluate(results: list[GoalResult]) -> list[Level]`; `format_report(levels) -> str`.

- [ ] **Step 1: Write the failing tests**

Create `test/test_nav_check.py`:

```python
from rospider_gazebo import nav_check
from rospider_gazebo.nav_check import COURSE_LIMIT, GoalResult


def _ok(seconds=20.0):
    return [GoalResult(name, 'succeeded', seconds) for name, _, _ in nav_check.COURSE]


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
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd ROSpider/src/simulations/rospider_gazebo && python3 -m pytest -q test/test_nav_check.py`
Expected: collection error, `ImportError: cannot import name 'nav_check'`.

- [ ] **Step 3: Write the implementation**

Create `rospider_gazebo/nav_check.py`:

```python
"""Score the Nav2 exercise's course run (docs/superpowers/specs/2026-09-28-nav-challenge-design.md).

scripts/check_nav.py drives the course and hands the per-goal results here.
Pure Python (no rclpy), so the level logic and the Thai report are unit
tested. The checker stops at the first failed goal, so `results` may be
shorter than COURSE.
"""

from dataclasses import dataclass

from rospider_gazebo.slam_check import Level

#: (name, Thai label, (x, y) in the map frame = world frame; the robot spawns at the origin).
COURSE = (
    ('G1', 'มุมห้อง A', (-0.4, -1.3)),
    ('G2', 'ห้อง B ผ่านประตู', (3.3, -0.4)),
    ('G3', 'กลับจุดเริ่ม', (0.0, 0.0)),
)

GOAL_TIMEOUT = 120.0    # sim seconds per goal
PLAN_TIMEOUT = 15.0     # wall seconds to wait for the planner's answer
COURSE_LIMIT = 180.0    # sim seconds for the whole course; set from measurements


@dataclass
class GoalResult:
    name: str
    status: str         # succeeded, no_path, aborted, timeout, canceled, rejected
    seconds: float


def evaluate(results):
    """The three levels for a course run."""
    def done(i):
        return len(results) > i and results[i].status == 'succeeded'

    def result(i):
        return results[i] if len(results) > i else None

    if not done(0):
        return [Level(1, 'fail', {'result': result(0)}),
                Level(2, 'skip', {'after': 1}), Level(3, 'skip', {'after': 1})]
    if not done(1):
        return [Level(1, 'pass', {'result': result(0)}),
                Level(2, 'fail', {'result': result(1)}), Level(3, 'skip', {'after': 2})]
    total = sum(r.seconds for r in results)
    passed = done(2) and total <= COURSE_LIMIT
    return [Level(1, 'pass', {'result': result(0)}), Level(2, 'pass', {'result': result(1)}),
            Level(3, 'pass' if passed else 'fail', {'result': result(2), 'total': total})]


_TITLES = {1: 'ถึงเป้าหมายแรก', 2: 'ผ่านประตูไปห้อง B', 3: 'วิ่งครบทันเวลา'}


def _failure(result):
    """Why a goal failed, and a hint that names the symptom, not the parameter."""
    if result is None:
        return 'ยังไม่ได้วิ่ง', 'ตัวตรวจหยุดก่อนถึงจุดนี้ - ลองตรวจใหม่อีกครั้ง'
    if result.status == 'no_path':
        return ('Nav2 หาเส้นทางไปไม่ได้',
                'ใน RViz ดู costmap ช่องทางที่ต้องผ่านตันไหม? '
                'หุ่นในแผนที่ตัวใหญ่แค่ไหนเทียบกับช่องประตู')
    if result.status in ('timeout', 'aborted'):
        return (f'ไปไม่ถึงหรือไม่ยอมจบ ({result.seconds:.0f} วินาที)',
                'หุ่นไปถึงแถวเป้าแล้วแต่วนอยู่ไม่ยอมหยุดไหม? '
                'Nav2 ต้องเข้าใกล้เป้าแค่ไหนถึงนับว่าถึงแล้ว')
    return ('Nav2 ไม่รับหรือยกเลิกเป้าหมาย',
            'ปิด-เปิด launch ใหม่ รอให้ RViz ขึ้นแผนที่ แล้วตรวจอีกครั้ง')


def format_report(levels):
    """Thai text for the terminal."""
    lines = []
    for level in levels:
        head = f'ด่าน {level.number} {_TITLES[level.number]}:'
        n = level.numbers
        if level.status == 'skip':
            lines.append(f'{head} ยังไม่ตรวจ (ผ่านด่าน {n["after"]} ก่อน)')
        elif level.number == 3 and level.status == 'fail' and n['result'] is not None \
                and n['result'].status == 'succeeded':
            lines.append(f'{head} ไม่ผ่าน (วิ่งครบ {n["total"]:.0f} วินาที '
                         f'ต้องไม่เกิน {COURSE_LIMIT:.0f})')
            lines.append('  คำใบ้: หุ่นเดินช้าไปไหม? ดูว่ามันเดินเร็วแค่ไหนตอนวิ่งทางตรง')
        elif level.status == 'fail':
            why, hint = _failure(n['result'])
            lines.append(f'{head} ไม่ผ่าน - {why}')
            lines.append(f'  คำใบ้: {hint}')
        elif level.number == 3:
            lines.append(f'{head} ผ่าน ({n["total"]:.0f} วินาที ต้องไม่เกิน {COURSE_LIMIT:.0f})')
        else:
            lines.append(f'{head} ผ่าน ({n["result"].seconds:.0f} วินาที)')
    if all(level.status == 'pass' for level in levels):
        lines.append('ผ่านครบทุกด่าน!')
    return '\n'.join(lines)
```

Add to `CMakeLists.txt` after the `test_nav_params` entry:

```cmake
  ament_add_pytest_test(test_nav_check test/test_nav_check.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd ROSpider/src/simulations/rospider_gazebo && python3 -m pytest -q test/test_nav_check.py`
Expected: 8 passed.

---

### Task 3: `check_nav.py` — drive the course

**Files:**
- Create: `scripts/check_nav.py` (mode 755)
- Modify: `CMakeLists.txt` (install it)

**Interfaces:**
- Consumes: `nav_check.COURSE`, `GOAL_TIMEOUT`, `PLAN_TIMEOUT`, `GoalResult`, `evaluate`, `format_report` (Task 2).
- Produces: `ros2 run rospider_gazebo check_nav.py`, exit 0 only when all levels pass; exit 1 with a Thai line when Nav2 is not up.

- [ ] **Step 1: Write the script**

Create `scripts/check_nav.py`:

```python
#!/usr/bin/env python3
"""ตรวจโจทย์ Nav2: สั่งหุ่นวิ่งตามเส้นทาง แล้วบอกผลทีละด่าน

    ros2 run rospider_gazebo check_nav.py

Before each goal it asks the planner for a path, so a blocked doorway is
reported at once instead of after Nav2's recovery retries; then it sends the
goal and waits for the result. Times are on the sim clock, so a slow PC is
not penalised. It stops at the first failed goal. Scoring: rospider_gazebo/nav_check.py.
"""

import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rospider_gazebo import nav_check
from rospider_gazebo.nav_check import GoalResult

SERVER_WAIT = 30.0
STATUS = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_ABORTED: 'aborted',
          GoalStatus.STATUS_CANCELED: 'canceled'}


class Checker:
    def __init__(self):
        self.node = rclpy.create_node(
            'check_nav', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.plan = ActionClient(self.node, ComputePathToPose, 'compute_path_to_pose')
        self.nav = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.handle = None          # the goal in flight, cancelled on Ctrl+C

    def sim_now(self):
        return self.node.get_clock().now().nanoseconds / 1e9

    def wait(self, future, wall_timeout):
        end = time.monotonic() + wall_timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return future.result() if future.done() else None

    def ready(self):
        end = time.monotonic() + 10.0
        while self.sim_now() == 0.0 and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return (self.sim_now() > 0.0 and self.plan.wait_for_server(timeout_sec=SERVER_WAIT)
                and self.nav.wait_for_server(timeout_sec=SERVER_WAIT))

    def pose(self, x, y):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.w = 1.0
        return msg

    def has_path(self, x, y):
        goal = ComputePathToPose.Goal()
        goal.goal = self.pose(x, y)
        handle = self.wait(self.plan.send_goal_async(goal), nav_check.PLAN_TIMEOUT)
        if handle is None or not handle.accepted:
            return False
        answer = self.wait(handle.get_result_async(), nav_check.PLAN_TIMEOUT)
        return (answer is not None and answer.status == GoalStatus.STATUS_SUCCEEDED
                and len(answer.result.path.poses) > 0)

    def drive(self, name, x, y):
        if not self.has_path(x, y):
            return GoalResult(name, 'no_path', 0.0)
        goal = NavigateToPose.Goal()
        goal.pose = self.pose(x, y)
        start = self.sim_now()
        self.handle = self.wait(self.nav.send_goal_async(goal), 10.0)
        if self.handle is None or not self.handle.accepted:
            return GoalResult(name, 'rejected', 0.0)
        future = self.handle.get_result_async()
        # Sim-clock timeout, with a wall-clock backstop in case the sim stalls.
        wall_end = time.monotonic() + nav_check.GOAL_TIMEOUT * 5
        while not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.sim_now() - start > nav_check.GOAL_TIMEOUT or time.monotonic() > wall_end:
                self.cancel()
                return GoalResult(name, 'timeout', self.sim_now() - start)
        self.handle = None
        status = STATUS.get(future.result().status, 'aborted')
        return GoalResult(name, status, self.sim_now() - start)

    def cancel(self):
        if self.handle is not None:
            self.wait(self.handle.cancel_goal_async(), 5.0)
            self.handle = None


def main():
    rclpy.init()
    checker = Checker()
    print('ตรวจโจทย์ Nav2 - ถ้าเพิ่งตรวจไปรอบหนึ่ง ให้ปิด-เปิด launch ใหม่ก่อน '
          'หุ่นจะได้เริ่มที่จุดเกิดและเวลาเทียบกันได้')
    try:
        if not checker.ready():
            print('ไม่พบ Nav2 - เปิด ros2 launch rospider_gazebo nav_challenge.launch.py '
                  'แล้วรอให้ RViz ขึ้นแผนที่ก่อน')
            return 1
        results = []
        for name, label, (x, y) in nav_check.COURSE:
            print(f'{name} {label}: กำลังไป ({x:.1f}, {y:.1f}) ...', flush=True)
            result = checker.drive(name, x, y)
            print(f'  -> {result.status} ({result.seconds:.0f} วินาที)', flush=True)
            results.append(result)
            if result.status != 'succeeded':
                break
        levels = nav_check.evaluate(results)
        print(nav_check.format_report(levels))
        return 0 if all(level.status == 'pass' for level in levels) else 1
    except KeyboardInterrupt:
        checker.cancel()
        print('หยุดตรวจแล้ว (ยกเลิกเป้าหมายให้หุ่นหยุด)')
        return 130
    finally:
        checker.node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
```

In `CMakeLists.txt`, add `scripts/check_nav.py` to the first `install(PROGRAMS ...)` block, next to `scripts/check_slam.py`.

- [ ] **Step 2: Build and check the no-Nav2 path**

Run (from `ROSpider/`, with no simulation running):

```bash
chmod 755 src/simulations/rospider_gazebo/scripts/check_nav.py
colcon build --packages-select rospider_gazebo --symlink-install && source install/local_setup.bash
timeout 60 ros2 run rospider_gazebo check_nav.py; echo "exit $?"
```

Expected: the reminder line, then `ไม่พบ Nav2 - ...`, `exit 1`, no traceback. (With no `/clock` the sim-time wait ends after 10 s, so this takes ~10 s.)

---

### Task 4: `nav_challenge.launch.py`

**Files:**
- Create: `launch/nav_challenge.launch.py`

**Interfaces:**
- Consumes: `nav_params.merged_params_file(nav2_path, user_path)` (Task 1); `challenge_params.ensure_user_file`, `ChallengeConfigError` (existing); `navigation.launch.py` args `world`, `map`, `params_file`, `gui`, `rviz` (existing).
- Produces: `ros2 launch rospider_gazebo nav_challenge.launch.py [params:=] [map:=] [gui:=] [rviz:=]`.

- [ ] **Step 1: Write the launch file**

```python
"""`ros2 launch rospider_gazebo nav_challenge.launch.py` -- the Nav2 exercise.

The SLAM exercise's room (worlds/slam_challenge.sdf) and its reference map
(maps/slam_challenge.yaml) with navigation.launch.py, on
config/nav2_params.yaml with the participant's six values from
config/nav_challenge.yaml written into every place they belong
(rospider_gazebo/nav_params.py). The participant edits that file in the
source tree and relaunches (needs `colcon build --symlink-install`);
`git checkout -- <file>` starts over. Score the run with
`ros2 run rospider_gazebo check_nav.py`.

  params  another participant file (default: config/nav_challenge.yaml);
          a path that does not exist yet gets a copy of the starting file.
          Instructors pass config/nav_challenge_solved.yaml's path.
  map     the participant's own map instead of the reference (a name in
          ROSpider/maps or ./maps, or a path)
  gui, rviz  as navigation.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from rospider_gazebo import challenge_params, nav_params


def launch_setup(context):
    pkg = get_package_share_directory('rospider_gazebo')
    template = os.path.join(pkg, 'config', 'nav_challenge.yaml')
    user = os.path.expanduser(LaunchConfiguration('params').perform(context)) or template
    created = challenge_params.ensure_user_file(template, user)
    user = os.path.realpath(user)       # under --symlink-install: the source copy to edit
    try:
        merged = nav_params.merged_params_file(os.path.join(pkg, 'config', 'nav2_params.yaml'), user)
    except challenge_params.ChallengeConfigError as err:
        raise RuntimeError(f'ไฟล์โจทย์ใช้ไม่ได้: {err}') from None
    map_value = (LaunchConfiguration('map').perform(context)
                 or os.path.join(pkg, 'maps', 'slam_challenge.yaml'))

    note = ('สร้างไฟล์โจทย์ใหม่ที่' if created else 'ใช้ไฟล์โจทย์') + f' {user}'
    if f'{os.sep}install{os.sep}' in user:
        note += (' (ไฟล์นี้อยู่ใน install: แก้ที่ src แล้วต้อง colcon build ใหม่ '
                 'หรือ build ด้วย --symlink-install)')
    return [
        LogInfo(msg=f'[nav_challenge] {note} -- แก้ไฟล์นี้แล้ว launch ใหม่'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'world': 'slam_challenge',
                'map': map_value,
                'params_file': merged,
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
            }.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value='',
                              description="the participant's Nav2 values "
                                          '(default: config/nav_challenge.yaml)'),
        DeclareLaunchArgument('map', default_value='',
                              description='your own map instead of the reference'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
```

- [ ] **Step 2: Build and check it loads and refuses a bad file**

Run (from `ROSpider/`):

```bash
colcon build --packages-select rospider_gazebo --symlink-install && source install/local_setup.bash
ros2 launch rospider_gazebo nav_challenge.launch.py --show-args
printf '/**:\n  ros__parameters:\n    robot_radus: 0.2\n' > <scratchpad>/nav_typo.yaml
timeout 20 ros2 launch rospider_gazebo nav_challenge.launch.py params:=<scratchpad>/nav_typo.yaml 2>&1 | grep ไฟล์โจทย์
```

Expected: `params`, `map`, `gui`, `rviz` listed; the second prints `ไฟล์โจทย์ใช้ไม่ได้: ... ไม่รู้จัก parameter 'robot_radus'` before Gazebo starts.

---

### Task 5: verify in the sim and set `COURSE_LIMIT`

**Files:**
- Possibly modify: `rospider_gazebo/nav_check.py` (`COURSE_LIMIT`, `GOAL_TIMEOUT`, with the measured numbers in a comment); `config/nav_challenge*.yaml` (only if a wrong value does not produce its symptom — then ask the user first)
- Scratch only: run scripts in the session scratchpad

- [ ] **Step 1: Run the four states on fresh launches**

For each state write a scratch copy of `config/nav_challenge.yaml` with these values, launch with `params:=<copy> rviz:=false` under an isolated `ROS_DOMAIN_ID`/`GZ_PARTITION`, wait until `ros2 lifecycle get /bt_navigator` is `active`, run `ros2 run rospider_gazebo check_nav.py`, record each goal's status and seconds, then stop every process of that partition:

| State | xy_goal_tolerance | robot_radius | max_vel_x | Expected |
|---|---|---|---|---|
| start | 0.002 | 0.3 | 0.03 | level 1 fail (G1 timeout/aborted) |
| tolerance fixed | 0.05 | 0.3 | 0.03 | level 1 pass, level 2 fail with no_path |
| + radius fixed | 0.05 | 0.15 | 0.03 | levels 1-2 pass, level 3 fail over time |
| solved | 0.05 | 0.15 | 0.15 | all pass |

Also run `max_vel_x` 0.05 (Hiwonder's real-robot value) with the other two fixed, and record its course time.

- [ ] **Step 2: Set `COURSE_LIMIT` from the measurements**

Pick a limit with a clear margin between the solved time and the 0.03 time. If 0.05's time also fits under a limit that still fails 0.03 with margin, prefer a limit that lets 0.05 pass. Write the measured times in the comment above the constant. If no limit separates solved from 0.03, or a wrong value does not produce its symptom, stop and report to the user with the numbers.

- [ ] **Step 3: Run the whole unit suite**

Run: `cd ROSpider/src/simulations/rospider_gazebo && python3 -m pytest -q test`
Expected: all pass (216 existing + 19 new).

---

### Task 6: docs, then hand back for the commit

**Files:**
- Modify: `ROSpider/SIMULATION.md`, `CLAUDE.md`, the spec (measured values)

- [ ] **Step 1: `ROSpider/SIMULATION.md`**

After the SLAM exercise block (the line starting `นำทางในห้องโจทย์ด้วยแผนที่ที่เซฟ`), add:

````markdown

**โจทย์ Nav2: ให้หุ่นวิ่งครบเส้นทาง** (10–15 นาที)

ใช้ห้องโจทย์เดิมกับแผนที่เฉลย ค่า Nav2 ตั้งผิดไว้ 3 จาก 6 ค่า แก้จนหุ่นวิ่งครบ 3 จุด (มุมห้อง A → ห้อง B → จุดเริ่ม) ทันเวลา

```bash
ros2 launch rospider_gazebo nav_challenge.launch.py      # map:=<ชื่อ> ใช้แผนที่ของตัวเอง
ros2 run rospider_gazebo check_nav.py                    # ตัวตรวจสั่งวิ่งเองแล้วบอกผลทีละด่าน
```

แก้ค่าใน `src/simulations/rospider_gazebo/config/nav_challenge.yaml` แล้วปิด-เปิด launch ใหม่ก่อนตรวจทุกครั้ง — ด่าน: 1 ถึงเป้าหมายแรก · 2 ผ่านประตูไปห้อง B · 3 วิ่งครบทันเวลา
````

- [ ] **Step 2: `CLAUDE.md`**

After the `- **SLAM exercise ...**` bullet add one bullet: `nav_challenge.launch.py` = `navigation.launch.py world:=slam_challenge` + the reference map + `nav2_params.yaml` with six friendly keys from `config/nav_challenge.yaml` written to every path (`rospider_gazebo/nav_params.py`, tested; both costmaps' `robot_radius`, DWB + `velocity_smoother.max_velocity[i]`); starting values; why `robot_radius` not `inflation_radius` closes the doorway; `check_nav.py` plans each leg first (no_path at once), stops at the first failure, times on the sim clock; `COURSE_LIMIT` and the measured times.

- [ ] **Step 3: Update the spec with the measured values**

Replace "draft" values and the time limit in `docs/superpowers/specs/2026-09-28-nav-challenge-design.md` with what Task 5 measured.

- [ ] **Step 4: Final checks and hand back**

Run the full unit suite and `colcon test --packages-select rospider_gazebo && colcon test-result --test-result-base build/rospider_gazebo`. Report measured times and any value changes; remind the user that `config/nav_challenge.yaml` must hold the starting values at commit time, and ask how to commit (with or separately from the SLAM exercise and the earlier trim).
