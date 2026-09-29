# V-SLAM Challenges Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Two workshop exercises on RTAB-Map camera-only V-SLAM — map the exercise room (scored by `check_vslam.py`) and navigate the Nav2 course on that map (scored by `check_nav.py`) — plus the room and launch fixes the probes showed they need.

**Architecture:** Same pattern as the SLAM and Nav2 exercises: a participant YAML edited in place is merged over the package's own config into a temp file by a pure, tested module (`vslam_params.py`); a pure, tested scoring module (`vslam_check.py`) plus a thin CLI (`check_vslam.py`); thin launch files that include `vslam.launch.py`. `vslam.launch.py` itself gains localization-on-a-copy, `map_always_update: false` while localizing, and Nav2 activation by the existing lost-reply-proof activator.

**Tech Stack:** ROS 2 Jazzy, Gazebo Harmonic, RTAB-Map (`rtabmap_slam`, `rtabmap-export` from `ros-jazzy-rtabmap`), Nav2 (RPP via `nav_params`), Python 3.12, numpy, sqlite3, pytest via `ament_add_pytest_test`.

**Spec:** `docs/superpowers/specs/2026-09-28-vslam-challenge-design.md`

## Global Constraints

- All paths below are relative to `ROSpider/src/simulations/rospider_gazebo/` unless they start with `ROSpider/`, `docs/` or `/`.
- Participant-facing text (hints, launch errors, checker output) is Thai; hints describe the symptom and never name a parameter (`camera_view`, `Grid/`, `Vis/`, `RangeMax`, `MinInliers`, `robot_radius`, `max_vel` must not appear in any hint).
- Modules under `rospider_gazebo/` that launch files or tests import must not import `rclpy` or `cv2` (the launch process and the unit tests have neither context).
- Participant files are edited in place in the source tree (`--symlink-install`), exactly like `config/slam_challenge.yaml`; `params:=<path>` uses another file, copied from the template if missing.
- Both exercises start the robot at the spawn origin; relocalization is out of scope.
- **Commits:** the user wants one commit per finished feature and never a commit without asking. No task commits. Task 10 ends by asking the user.
- **Sim runs:** one at a time, never overlapping, each with its own `GZ_PARTITION=vslamplan ROS_DOMAIN_ID=61`; always stop a run completely before starting the next (Step "stop" below).
- `<scratch>` in commands means the session's scratchpad directory (logs and temp params; never the repo).
- Build/test commands (from `ROSpider/`): `source /opt/ros/jazzy/setup.bash && colcon build --packages-select rospider_gazebo --symlink-install && source install/local_setup.bash`; unit tests: `cd src/simulations/rospider_gazebo && PYTHONPATH=. python3 -m pytest test/<file> -q`.

Sim helpers used by Tasks 7-9 (paste into a shell; `$W` is the workspace):

```bash
W=/home/earth157/entech_hiwonder_ros2_ws/ROSpider
simenv() { source /opt/ros/jazzy/setup.bash; source $W/install/local_setup.bash
           export need_compile=True GZ_PARTITION=vslamplan ROS_DOMAIN_ID=61; }
# start: setsid ros2 launch ... > /tmp/claude-1000/<scratch>/<tag>.log 2>&1 &  then  PGID=$!
stop() {  # SIGINT the launch group, wait for rtabmap to write its database, then clean up
  kill -INT -- -$1 2>/dev/null
  for i in $(seq 1 60); do pgrep -x rtabmap >/dev/null || break; sleep 1; done
  sleep 3; kill -KILL -- -$1 2>/dev/null; sleep 2; }
```

## Review Focus

1. `check_vslam.py` run while the mapping launch is still open (the most likely participant mistake): it must refuse and say to close the launch, not score a half-written database — test `test_database_still_open_is_refused` in Task 6.
2. A participant who never drives back to the start: the map is straight but has no loop closure; level 3 must fail with the "drive back" hint, not pass — test `test_no_loop_closure_fails_level_three` in Task 5.
3. Numbers typed without quotes (`Grid/RangeMax: 5.0`), booleans, blanks and typos in the participant file: converted to RTAB-Map's string form or refused with a Thai message naming the key — tests in Task 4.
4. A map name that does not exist (typo, or the mapping exercise not done): `check_vslam.py` prints where it looked; `vslam_nav_challenge.launch.py` stops before Gazebo starts with a message pointing to the mapping exercise — tests `test_missing_map_report` (Task 5) and `test_require_map_missing` (Task 4).
5. `rtabmap-export` missing or failing: a Thai message and exit 1, no traceback — test `test_export_failure_is_reported` in Task 6.

---

### Task 1: One texture per wall face in `worlds/slam_challenge.sdf`

**Files:**
- Create: `tools/make_face_textures.py`, `worlds/textures/face_*.png` (14, generated)
- Modify: `worlds/slam_challenge.sdf` (the 7 inner-wall visuals; header comment)
- Test: `test/test_slam_challenge_world.py`

**Interfaces:**
- Produces: visuals named `<wall>_a_v` / `<wall>_b_v` for `partition_n, partition_s, stub, maze_1..maze_4`, textures `textures/face_<wall>_<a|b>.png`. Collisions unchanged.

- [ ] **Step 1: Write the failing test**

```python
"""worlds/slam_challenge.sdf: every inner-wall face carries its own texture (V-SLAM loop
closure matched identical faces 1.2 m apart), and the collision geometry the LiDAR map was
recorded with is unchanged."""
import pathlib
import xml.etree.ElementTree as ET

PKG = pathlib.Path(__file__).resolve().parents[1]
WALLS = ('partition_n', 'partition_s', 'stub', 'maze_1', 'maze_2', 'maze_3', 'maze_4')


def _walls_link():
    root = ET.parse(PKG / 'worlds' / 'slam_challenge.sdf').getroot()
    return root.find(".//model[@name='walls']/link")


def test_each_inner_face_has_its_own_existing_texture():
    link = _walls_link()
    textures = []
    for wall in WALLS:
        assert link.find(f"visual[@name='{wall}_v']") is None
        for side in 'ab':
            visual = link.find(f"visual[@name='{wall}_{side}_v']")
            assert visual is not None, f'{wall}_{side}_v missing'
            texture = visual.find('.//albedo_map').text
            assert (PKG / 'worlds' / texture).is_file(), texture
            textures.append(texture)
    assert len(set(textures)) == 14
    assert 'partition.png' not in (PKG / 'worlds' / 'slam_challenge.sdf').read_text()


def test_inner_wall_collisions_unchanged():
    link = _walls_link()
    for wall in WALLS:
        assert link.find(f"collision[@name='{wall}_c']") is not None
    assert len(link.findall('collision')) == 11      # 4 outer + 7 inner
```

- [ ] **Step 2: Run it — expect FAIL**

Run: `PYTHONPATH=. python3 -m pytest test/test_slam_challenge_world.py -q`
Expected: FAIL (`partition_n_a_v missing`).

- [ ] **Step 3: Write `tools/make_face_textures.py`**

```python
#!/usr/bin/env python3
"""Draw the per-face wall posters of worlds/slam_challenge.sdf (worlds/textures/face_*.png).

V-SLAM needs every wall face to look different: with one partition.png on all seven inner
walls, both faces, RTAB-Map accepted 4-5 wrong loop closures per run (room A matched onto the
corridor, 1.2 m off). Fixed seeds, so re-running draws the same images.

    python3 tools/make_face_textures.py
"""
import pathlib
import random

from PIL import Image, ImageDraw, ImageFont

PKG = pathlib.Path(__file__).resolve().parents[1]
WALLS = ('partition_n', 'partition_s', 'stub', 'maze_1', 'maze_2', 'maze_3', 'maze_4')
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'


def poster(path, seed, label):
    rnd = random.Random(seed)
    image = Image.new('RGB', (768, 512), tuple(rnd.randint(120, 235) for _ in range(3)))
    draw = ImageDraw.Draw(image)
    for _ in range(14):
        colour = tuple(rnd.randint(0, 255) for _ in range(3))
        x, y, s = rnd.randint(0, 740), rnd.randint(0, 480), rnd.randint(40, 170)
        kind = rnd.choice('rct')
        if kind == 'r':
            draw.rectangle([x, y, x + s, y + int(s * rnd.uniform(0.5, 1.5))], fill=colour)
        elif kind == 'c':
            draw.ellipse([x, y, x + s, y + s], fill=colour)
        else:
            draw.polygon([(x, y + s), (x + s // 2, y), (x + s, y + s)], fill=colour)
    draw.text((16, 430), label, fill=(15, 15, 15), font=ImageFont.truetype(FONT, 64))
    image.save(path)


def main():
    out = PKG / 'worlds' / 'textures'
    for number, (wall, side) in enumerate(((w, s) for w in WALLS for s in 'ab'), start=1):
        poster(out / f'face_{wall}_{side}.png', 1000 + number, f'W{number:02d}')


if __name__ == '__main__':
    main()
```

Run: `python3 tools/make_face_textures.py && ls worlds/textures/face_* | wc -l` → `14`.

- [ ] **Step 4: Split the seven inner-wall visuals (one-off script, run once)**

```bash
python3 - <<'EOF'
import math, re, pathlib
p = pathlib.Path('worlds/slam_challenge.sdf'); sdf = p.read_text()
pat = re.compile(r'        <visual name="(\w+)_v"><pose>([-\d. ]+)</pose><geometry><box><size>([\d.]+ [\d.]+) 1.0</size></box></geometry>(<material>.*?partition\.png.*?</material>)</visual>\n')
def split(m):
    name, pose, size, mat = m.group(1), [float(v) for v in m.group(2).split()], m.group(3), m.group(4)
    x, y, z, yaw = pose[0], pose[1], pose[2], pose[5]
    sx, sy = (float(v) for v in size.split()); thin_x = sx < sy
    a = yaw if not thin_x else yaw - math.pi / 2          # the face normal's direction
    out = ''
    for side, sgn in (('a', 1), ('b', -1)):
        ox, oy = -math.sin(a) * 0.0125 * sgn, math.cos(a) * 0.0125 * sgn
        box = f'0.025 {sy:g} 1.0' if thin_x else f'{sx:g} 0.025 1.0'
        out += (f'        <visual name="{name}_{side}_v"><pose>{x + ox:.4f} {y + oy:.4f} {z:g} 0 0 {yaw:.4f}</pose>'
                f'<geometry><box><size>{box}</size></box></geometry>'
                + mat.replace('textures/partition.png', f'textures/face_{name}_{side}.png') + '</visual>\n')
    return out
new, n = pat.subn(split, sdf); assert n == 7, n
p.write_text(new)
EOF
```

Expected: no output; `grep -c "_a_v\|_b_v" worlds/slam_challenge.sdf` → `14`.

- [ ] **Step 5: Header comment.** In the comment block at the top of `worlds/slam_challenge.sdf`, add one sentence: `Each inner wall's visual is two 0.025 m faces with their own poster (tools/make_face_textures.py): with one texture on every face, V-SLAM loop closure matched room A onto the corridor, 1.2 m off. Collisions are the single 0.05 m boxes, so the LiDAR map is unchanged.`

- [ ] **Step 6: Run the test — expect PASS.** `PYTHONPATH=. python3 -m pytest test/test_slam_challenge_world.py -q` → `2 passed`.

- [ ] **Step 7: Register the test** in `CMakeLists.txt` after `test_nav_check`:

```cmake
  ament_add_pytest_test(test_slam_challenge_world test/test_slam_challenge_world.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
```

---

### Task 2: The activator brings up several lifecycle nodes in order

**Files:**
- Modify: `rospider_gazebo/lifecycle.py`, `scripts/lifecycle_activate.py`
- Test: `test/test_lifecycle.py`

**Interfaces:**
- Consumes: `bring_up(get_state, change_state, timeout, clock, sleep, period) -> bool` (existing).
- Produces: `bring_up_all(names, get_state, change_state, timeout=120.0, clock=time.monotonic, sleep=time.sleep, period=0.5) -> list[str]` — `get_state(name)`, `change_state(name, transition)`; returns the names **not** active at the end (empty = success). `lifecycle_activate.py NODE [NODE ...] [--timeout S]`.

- [ ] **Step 1: Failing tests** (append to `test/test_lifecycle.py`; `FakeNode` and `Clock` already exist there):

```python
from rospider_gazebo.lifecycle import bring_up_all


def _nodes(**states):
    return {name: FakeNode(state) for name, state in states.items()}


def test_all_nodes_come_up_in_order_despite_lost_replies():
    nodes = _nodes(controller_server='unconfigured', planner_server='unconfigured',
                   bt_navigator='unconfigured')
    order = []

    def change(name, transition):
        order.append((name, transition))
        nodes[name].change_state(transition)

    clock = Clock()
    left = bring_up_all(list(nodes), lambda n: nodes[n].get_state(), change,
                        clock=clock, sleep=clock.sleep)
    assert left == []
    assert [n for n, _ in order] == ['controller_server'] * 2 + ['planner_server'] * 2 + ['bt_navigator'] * 2


def test_a_node_that_never_answers_is_reported_and_blocks_the_rest():
    nodes = _nodes(controller_server='unconfigured', planner_server='unconfigured')
    clock = Clock()
    left = bring_up_all(['controller_server', 'ghost', 'planner_server'],
                        lambda n: nodes[n].get_state() if n in nodes else None,
                        lambda n, t: nodes[n].change_state(t), timeout=30.0,
                        clock=clock, sleep=clock.sleep)
    assert left == ['ghost', 'planner_server']
    assert nodes['planner_server'].state == 'unconfigured'   # order kept: never started early
```

- [ ] **Step 2: Run — expect FAIL** (`ImportError: bring_up_all`): `PYTHONPATH=. python3 -m pytest test/test_lifecycle.py -q`

- [ ] **Step 3: Implement** in `rospider_gazebo/lifecycle.py`:

```python
def bring_up_all(names, get_state, change_state, timeout=120.0, clock=time.monotonic,
                 sleep=time.sleep, period=0.5):
    """bring_up each node in `names` in order, sharing one deadline, as nav2's
    lifecycle_manager does (a later node may need an earlier one active). Returns the
    names not active at the end: the one that timed out and every one after it."""
    deadline = clock() + timeout
    for index, name in enumerate(names):
        left = deadline - clock()
        if left <= 0 or not bring_up(lambda: get_state(name),
                                     lambda transition: change_state(name, transition),
                                     left, clock, sleep, period):
            return list(names[index:])
    return []
```

Also update the module docstring's first paragraph end: `The same holds for Nav2's servers (smoother_server lost a change_state reply in ~1 of 12 V-SLAM launches), so bring_up_all does them in order.`

- [ ] **Step 4: Run — expect PASS** (all of `test_lifecycle.py`).

- [ ] **Step 5: `scripts/lifecycle_activate.py` takes several nodes.** Replace the parser, clients and result handling:

```python
    parser = argparse.ArgumentParser()
    parser.add_argument('nodes', nargs='+')
    parser.add_argument('--timeout', type=float, default=60.0)
    args, _ = parser.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    names = [name.strip('/') for name in args.nodes]

    rclpy.init()
    node = rclpy.create_node('lifecycle_activate_' + names[0].replace('/', '_'))
    clients = {name: (node.create_client(GetState, f'/{name}/get_state'),
                      node.create_client(ChangeState, f'/{name}/change_state'))
               for name in names}
```

with `get_state(name)` using `clients[name][0]`, `change_state(name, transition)` using `clients[name][1]` and logging `f'{name}: {transition}'`, then:

```python
    left = bring_up_all(names, get_state, change_state, args.timeout)
    if left:
        node.get_logger().error(f'not active after {args.timeout:.0f} s: {", ".join(left)}')
    else:
        node.get_logger().info(f'active: {", ".join(names)}')
    node.destroy_node()
    rclpy.try_shutdown()
    return 1 if left else 0
```

Update its docstring usage line to `ros2 run rospider_gazebo lifecycle_activate.py slam_toolbox` / `... controller_server planner_server ...` and "Replaces nav2_lifecycle_manager for slam.launch.py and vslam.launch.py's Nav2". `slam.launch.py` keeps passing one name (still valid).

- [ ] **Step 6:** `python3 -m py_compile scripts/lifecycle_activate.py` → no output.

---

### Task 3: `check_nav.py` resends a goal whose response was lost

**Files:**
- Modify: `rospider_gazebo/nav_check.py`, `scripts/check_nav.py`
- Test: `test/test_nav_check.py`

**Interfaces:**
- Produces: `nav_check.send_with_retry(send, wait, tries=3, timeout=10.0)` — `send()` returns a future, `wait(future, timeout)` returns the goal handle or `None`; returns the first handle received (accepted or not) or `None` after `tries` lost responses.

- [ ] **Step 1: Failing tests** (append to `test/test_nav_check.py`):

```python
def test_lost_goal_response_is_resent():
    sent = []
    answers = iter([None, None, 'handle'])
    handle = nav_check.send_with_retry(lambda: sent.append(1) or len(sent),
                                       lambda future, timeout: next(answers))
    assert handle == 'handle' and len(sent) == 3


def test_three_lost_responses_give_up():
    sent = []
    assert nav_check.send_with_retry(lambda: sent.append(1), lambda f, t: None) is None
    assert len(sent) == 3


def test_a_refusal_is_not_resent():
    class Refused:
        accepted = False
    sent = []
    handle = nav_check.send_with_retry(lambda: sent.append(1), lambda f, t: Refused())
    assert handle.accepted is False and len(sent) == 1
```

- [ ] **Step 2: Run — expect FAIL** (`AttributeError: send_with_retry`).

- [ ] **Step 3: Implement** in `rospider_gazebo/nav_check.py` (after `PLAN_TIMEOUT`):

```python
def send_with_retry(send, wait, tries=3, timeout=10.0):
    """Send an action goal; if its response is lost, send it again.

    On Jazzy a server sometimes accepts a goal but its response never arrives ("Failed to send
    goal response (timeout)", seen in a V-SLAM run): waiting once would report 'rejected' while
    the robot drives off. Nav2 replaces the first goal with the resent one. A response that
    arrives, accepted or not, is returned as it is; None after `tries` lost responses.
    """
    for _ in range(tries):
        handle = wait(send(), timeout)
        if handle is not None:
            return handle
    return None
```

- [ ] **Step 4: Use it in `scripts/check_nav.py`.** In `plan_to`: `handle = nav_check.send_with_retry(lambda: self.plan.send_goal_async(goal), self.wait, timeout=nav_check.PLAN_TIMEOUT)`. In `drive`, replace the `self.pending = ...; self.handle = self.wait(self.pending, 10.0); self.pending = None` lines with:

```python
        def send():
            self.pending = self.nav.send_goal_async(goal)
            return self.pending
        self.handle = nav_check.send_with_retry(send, self.wait)
        self.pending = None
```

and change the not-found message to `'ไม่พบ Nav2 - เปิด launch ของโจทย์ (nav_challenge หรือ vslam_nav_challenge) แล้วรอให้ RViz ขึ้นแผนที่ก่อน'`.

- [ ] **Step 5: Run — expect PASS:** `PYTHONPATH=. python3 -m pytest test/test_nav_check.py -q`; `python3 -m py_compile scripts/check_nav.py`.

---

### Task 4: `vslam_params.py` — participant files for both exercises

**Files:**
- Create: `rospider_gazebo/vslam_params.py`
- Modify: `config/vslam.yaml` (add three keys at their current effective values)
- Test: `test/test_vslam_params.py`

**Interfaces:**
- Consumes: `challenge_params.load_params(path) -> dict`, `challenge_params.ChallengeConfigError`, `challenge_params._typed(key, value, default, source)`, `nav_params.apply(nav2, overrides, source) -> dict`.
- Produces:
  - `CAMERA_VIEWS = {'floor': 'init', 'ahead': 'horizontal'}`
  - `split_camera_view(params, source) -> (arm_pose: str, rest: dict)`
  - `apply_rtabmap(vslam: dict, overrides: dict, source) -> dict` (copy of `vslam` with overrides under `rtabmap.ros__parameters`)
  - `mapping_params_file(vslam_path, user_path) -> (temp_yaml_path, arm_pose)`
  - `nav_params_file(nav2_path, user_path) -> (temp_yaml_path, arm_pose)`
  - `require_map(value, workspace_maps, cwd) -> str` (raises `ChallengeConfigError` with a Thai message naming the mapping exercise)

- [ ] **Step 1: Add the keys to `config/vslam.yaml`** under `rtabmap: ros__parameters:` after `Vis/MaxFeatures`, with a comment:

```yaml
    # RTAB-Map's own defaults, written out so vslam_challenge.yaml may set them
    Vis/MinInliers: "20"          # matched features needed to accept a loop closure
    Grid/CellSize: "0.05"
    Rtabmap/DetectionRate: "1"
```

- [ ] **Step 2: Failing tests** `test/test_vslam_params.py`:

```python
import pathlib

import pytest
import yaml
from rospider_gazebo import vslam_params
from rospider_gazebo.challenge_params import ChallengeConfigError

PKG = pathlib.Path(__file__).resolve().parents[1]
VSLAM = yaml.safe_load((PKG / 'config' / 'vslam.yaml').read_text())


def _rtab(merged):
    return merged['rtabmap']['ros__parameters']


def test_camera_view_becomes_arm_pose():
    assert vslam_params.split_camera_view({'camera_view': 'floor', 'a': 1}, 'f') == ('init', {'a': 1})
    assert vslam_params.split_camera_view({'camera_view': 'ahead'}, 'f') == ('horizontal', {})
    assert vslam_params.split_camera_view({}, 'f') == ('horizontal', {})


@pytest.mark.parametrize('value', ['down', '', None, True, 1])
def test_bad_camera_view_is_refused(value):
    with pytest.raises(ChallengeConfigError, match='camera_view'):
        vslam_params.split_camera_view({'camera_view': value}, 'f')


def test_rtabmap_keys_land_as_strings():
    merged = vslam_params.apply_rtabmap(VSLAM, {'Grid/RangeMax': 0.5, 'Vis/MinInliers': 5,
                                                'Grid/CellSize': '0.05'}, 'f')
    assert _rtab(merged)['Grid/RangeMax'] == '0.5'
    assert _rtab(merged)['Vis/MinInliers'] == '5'
    assert _rtab(merged)['Grid/CellSize'] == '0.05'
    assert _rtab(VSLAM)['Grid/RangeMax'] == '5.0'            # the base is not modified


@pytest.mark.parametrize('overrides, word', [
    ({'Grid/RangMax': '5.0'}, 'Grid/RangMax'),       # typo
    ({'Grid/RangeMax': None}, 'Grid/RangeMax'),      # blank
    ({'Grid/RangeMax': ''}, 'Grid/RangeMax'),
    ({'Grid/RangeMax': True}, 'Grid/RangeMax'),
    ({'map_always_update': 'yes'}, 'map_always_update'),   # non-string key keeps its type
])
def test_bad_values_are_refused(overrides, word):
    with pytest.raises(ChallengeConfigError, match=word):
        vslam_params.apply_rtabmap(VSLAM, overrides, 'f')


@pytest.mark.parametrize('name, pose', [('vslam_challenge.yaml', 'init'),
                                        ('vslam_challenge_solved.yaml', 'horizontal')])
def test_shipped_mapping_files_merge(name, pose):
    path, arm_pose = vslam_params.mapping_params_file(str(PKG / 'config' / 'vslam.yaml'),
                                                      str(PKG / 'config' / name))
    assert arm_pose == pose
    assert 'camera_view' not in yaml.safe_load(open(path))['rtabmap']['ros__parameters']


@pytest.mark.parametrize('name, pose', [('vslam_nav_challenge.yaml', 'init'),
                                        ('vslam_nav_challenge_solved.yaml', 'horizontal')])
def test_shipped_nav_files_merge(name, pose):
    path, arm_pose = vslam_params.nav_params_file(str(PKG / 'config' / 'vslam_nav2_params.yaml'),
                                                  str(PKG / 'config' / name))
    merged = yaml.safe_load(open(path))
    assert arm_pose == pose
    assert 'RegulatedPurePursuit' in merged['controller_server']['ros__parameters']['FollowPath']['plugin']


def test_require_map_missing(tmp_path):
    with pytest.raises(ChallengeConfigError, match='vslam_challenge'):
        vslam_params.require_map('nothere', str(tmp_path), str(tmp_path))


def test_require_map_found(tmp_path):
    (tmp_path / 'mine.db').write_bytes(b'')
    assert vslam_params.require_map('mine', str(tmp_path), '/') == str(tmp_path / 'mine.db')
```

- [ ] **Step 3: Create the four participant files** (Thai comments; header like `config/slam_challenge.yaml`).

`config/vslam_challenge.yaml`:
```yaml
# โจทย์ V-SLAM (สร้างแผนที่ด้วยกล้อง)
# แก้ค่าในไฟล์นี้ แล้วปิด launch (Ctrl+C) และเปิดใหม่ด้วยคำสั่งเดิม
# ค่าที่ไม่ได้อยู่ในไฟล์นี้ใช้ของ config/vslam.yaml
# อยากเริ่มโจทย์ใหม่: git checkout -- ไฟล์นี้ แล้ว launch ใหม่
/**:
  ros__parameters:
    # กล้องมองไปทางไหน: floor (ก้มมองพื้น) หรือ ahead (มองตรงไปข้างหน้า)
    camera_view: floor
    # ระยะไกลสุดจากกล้อง depth ที่เอามาวาดแผนที่ (เมตร)
    Grid/RangeMax: "0.5"
    # ต้องมีจุดเด่นในภาพตรงกันกี่จุด ถึงยอมรับว่ากลับมาที่เดิม (loop closure)
    Vis/MinInliers: "5"
    # ขนาดช่องหนึ่งช่องในแผนที่ (เมตร)
    Grid/CellSize: "0.05"
    # RTAB-Map ประมวลผลภาพกี่ครั้งต่อวินาที
    Rtabmap/DetectionRate: "1"
```
`config/vslam_challenge_solved.yaml`: same header replaced by `# เฉลยโจทย์ V-SLAM (สำหรับวิทยากร)` / `# ros2 launch rospider_gazebo vslam_challenge.launch.py params:=<path ของไฟล์นี้>`, values `ahead`, `"5.0"`, `"20"`, `"0.05"`, `"1"`.

`config/vslam_nav_challenge.yaml`:
```yaml
# โจทย์นำทางด้วย V-SLAM (ใช้แผนที่จากโจทย์ V-SLAM ของตัวเอง)
# แก้ค่าในไฟล์นี้ แล้วปิด launch (Ctrl+C) และเปิดใหม่ด้วยคำสั่งเดิม
# ค่าที่ไม่ได้อยู่ในไฟล์นี้ใช้ของ config/vslam_nav2_params.yaml
# อยากเริ่มโจทย์ใหม่: git checkout -- ไฟล์นี้ แล้ว launch ใหม่
/**:
  ros__parameters:
    # กล้องมองไปทางไหน: floor (ก้มมองพื้น) หรือ ahead (มองตรงไปข้างหน้า)
    camera_view: floor
    # ต้องเข้าใกล้เป้าหมายแค่ไหน (เมตร) ถึงนับว่าถึงแล้ว
    xy_goal_tolerance: 0.05
    # รัศมีตัวหุ่นในแผนที่ (เมตร) - planner จะไม่ให้หุ่นเข้าใกล้ผนังกว่านี้
    robot_radius: 0.3
    # ระยะที่ costmap เริ่มกันหุ่นให้ห่างผนัง (เมตร)
    inflation_radius: 0.3
    # ความเร็วเดินหน้าสูงสุด (เมตร/วินาที)
    max_vel_x: 0.05
    # ความเร็วหมุนสูงสุด (เรเดียน/วินาที)
    max_vel_theta: 0.25
```
`config/vslam_nav_challenge_solved.yaml`: answer-key header, values `ahead`, `0.05`, `0.15`, `0.3`, `0.15`, `0.25`.

- [ ] **Step 4: Run — expect FAIL** (`ModuleNotFoundError: vslam_params`).

- [ ] **Step 5: Implement** `rospider_gazebo/vslam_params.py`:

```python
"""The V-SLAM exercises' participant files (docs/superpowers/specs/2026-09-28-vslam-challenge-design.md).

Both files have the other exercises' shape (/**: ros__parameters:). `camera_view` is a
friendly key: it becomes vslam.launch.py's arm_pose (the camera rides the arm; `init` tilts
it 52 degrees at the floor). In the mapping file every other key is an RTAB-Map parameter
already under rtabmap: in config/vslam.yaml; RTAB-Map declares its parameters as strings, so
a number typed without quotes is written as its text. In the navigation file every other key
is one of nav_params' friendly Nav2 keys. Pure Python (no rclpy): launch files import it.
"""

import copy
import os
import tempfile

import yaml
from rospider_gazebo import challenge_params, maps, nav_params
from rospider_gazebo.challenge_params import ChallengeConfigError

CAMERA_VIEWS = {'floor': 'init', 'ahead': 'horizontal'}


def split_camera_view(params, source):
    """(arm_pose, the other keys). A missing camera_view means ahead."""
    rest = dict(params)
    view = rest.pop('camera_view', 'ahead')
    if not isinstance(view, str) or view not in CAMERA_VIEWS:
        raise ChallengeConfigError(
            f"{source}: 'camera_view' ต้องเป็น floor หรือ ahead แต่ได้ {view!r}")
    return CAMERA_VIEWS[view], rest


def _rtabmap_value(key, value, default, source):
    if not isinstance(default, str):
        return challenge_params._typed(key, value, default, source)
    if isinstance(value, bool) or value is None or value == '':
        raise ChallengeConfigError(f"{source}: '{key}' ต้องเป็นตัวเลขหรือข้อความ แต่ได้ {value!r}")
    return str(value) if isinstance(value, (int, float)) else value


def apply_rtabmap(vslam, overrides, source):
    merged = copy.deepcopy(vslam)
    params = merged['rtabmap']['ros__parameters']
    for key, value in overrides.items():
        if key not in params:
            raise ChallengeConfigError(
                f"{source}: ไม่รู้จัก parameter '{key}' (พิมพ์ชื่อผิดหรือเปล่า?)")
        params[key] = _rtabmap_value(key, value, params[key], source)
    return merged


def _write(data, prefix):
    with tempfile.NamedTemporaryFile('w', prefix=prefix, suffix='.yaml', delete=False) as handle:
        yaml.safe_dump(data, handle)
        return handle.name


def mapping_params_file(vslam_path, user_path):
    """config/vslam.yaml with the participant's values -> (temp file, arm_pose)."""
    with open(vslam_path) as handle:
        vslam = yaml.safe_load(handle)
    arm_pose, rest = split_camera_view(challenge_params.load_params(user_path), user_path)
    return _write(apply_rtabmap(vslam, rest, user_path), 'vslam_challenge_'), arm_pose


def nav_params_file(nav2_path, user_path):
    """config/vslam_nav2_params.yaml through nav_params.apply -> (temp file, arm_pose)."""
    with open(nav2_path) as handle:
        nav2 = yaml.safe_load(handle)
    arm_pose, rest = split_camera_view(challenge_params.load_params(user_path), user_path)
    return _write(nav_params.apply(nav2, rest, user_path), 'vslam_nav_challenge_'), arm_pose


def require_map(value, workspace_maps, cwd):
    """The participant's V-SLAM database for `value`, or a Thai error naming the mapping exercise."""
    path = maps.find_map(value, '.db', workspace_maps, cwd)
    if path is None:
        raise ChallengeConfigError(
            f"ไม่พบแผนที่ V-SLAM '{value}' (หาใน {workspace_maps}) - ทำโจทย์สร้างแผนที่ก่อน: "
            f'ros2 launch rospider_gazebo vslam_challenge.launch.py map:={value} '
            'ขับให้ทั่วแล้วปิด launch (Ctrl+C)')
    return path
```

- [ ] **Step 6: Run — expect PASS:** `PYTHONPATH=. python3 -m pytest test/test_vslam_params.py -q`. Register in `CMakeLists.txt` (`test_vslam_params`, same pattern as Task 1 Step 7).

---

### Task 5: `vslam_check.py` — scoring (pure)

**Files:**
- Create: `rospider_gazebo/vslam_check.py`
- Test: `test/test_vslam_check.py`

**Interfaces:**
- Consumes: `slam_check.FREE/OCCUPIED/UNKNOWN`, `GridMap(states, resolution, origin)`, `GridMap.area/cell_centres/states_at`, `slam_check.coverage(learner, reference)`, `slam_check.thickness_ratio(learner, reference)`, `slam_check.Level(number, status, numbers)`.
- Produces: constants `MIN_KNOWN_M2=1.0, MIN_COVERAGE=0.90, MIN_WALLS=0.60, WALL_SLACK_CELLS=2, MAX_THICKNESS=1.2, MAX_CLOSURE_ERROR_M=0.1, MAX_CLOSURE_ERROR_DEG=5.0`; `read_graph(db_path) -> (poses: dict[int, 4x4], links: list[(from, to, type, 4x4)])`; `closure_errors(poses, links) -> list[(from, to, metres, degrees)]`; `wall_recall(learner, reference, slack=WALL_SLACK_CELLS) -> float`; `db_in_use(db_path, proc_root='/proc') -> bool`; `evaluate(learner, reference, closures, missing=False) -> list[Level]`; `format_report(levels) -> str`.

- [ ] **Step 1: Failing tests** `test/test_vslam_check.py`:

```python
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


GOOD = [(5, 1, 0.01, 0.5)]            # (from, to, metres, degrees) as closure_errors returns
WRONG = [(5, 1, 1.2, 1.8)]


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


def test_floor_only_map_fails_level_two_on_walls():
    levels = vslam_check.evaluate(_room(walls=False), _room(), GOOD)
    assert _statuses(levels) == ['pass', 'fail', 'skip']
    assert levels[1].numbers['walls'] < vslam_check.MIN_WALLS


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


def test_closure_errors_compare_link_with_odometry():
    poses = {1: _pose(0, 0, 0), 5: _pose(1.0, 0.5, 90)}
    rel = np.linalg.inv(poses[5]) @ poses[1]
    right = (5, 1, 1, rel)
    wrong = (5, 1, 1, rel @ _pose(1.2, 0, 0))
    errors = vslam_check.closure_errors(poses, [right, wrong])
    assert errors[0][2] == pytest.approx(0.0, abs=1e-9)
    assert errors[1][2] == pytest.approx(1.2, abs=1e-6)


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
    con.commit(); con.close()
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
```

- [ ] **Step 2: Run — expect FAIL** (`ModuleNotFoundError: vslam_check`).

- [ ] **Step 3: Implement** `rospider_gazebo/vslam_check.py`:

```python
"""Score the V-SLAM mapping exercise (docs/superpowers/specs/2026-09-28-vslam-challenge-design.md).

scripts/check_vslam.py exports the 2D grid from the participant's RTAB-Map database with
rtabmap-export and reads its graph here. Pure Python (sqlite3, numpy; no rclpy), unit tested.

Loop closures are judged against odometry: in the sim odometry is exact, and RTAB-Map stores
each node's odometry pose in Node.pose, so a closure link (Link.type 1 global, 2 local) whose
transform disagrees with the odometry between its nodes is a place the robot "recognised"
wrongly. On a real robot, with drifting odometry, this test would not hold.

Thresholds, measured with probes in the textured maze room (reference = Hiwonder's values):
answer key 100% coverage, 100% walls, 1.0x, 0 wrong closures (6-13 correct); Grid/RangeMax 0.5
0.5 m2 known; camera tilted at the floor 94% coverage but 34% walls; Vis/MinInliers 5
1-2 wrong closures, 1.6-2.2x, 75-77% walls -- hence MIN_WALLS 0.60, so that mistake fails
level 3 and not level 2.
"""

import math
import os
import sqlite3
import struct
from contextlib import closing

import numpy as np
from rospider_gazebo.slam_check import (FREE, OCCUPIED, GridMap, Level, coverage,
                                        thickness_ratio)

MIN_KNOWN_M2 = 1.0
MIN_COVERAGE = 0.90
MIN_WALLS = 0.60
WALL_SLACK_CELLS = 2
MAX_THICKNESS = 1.2
MAX_CLOSURE_ERROR_M = 0.1
MAX_CLOSURE_ERROR_DEG = 5.0
CLOSURE_TYPES = (1, 2)


def _matrix(blob):
    m = np.eye(4)
    m[:3, :4] = np.array(struct.unpack('12f', blob[:48])).reshape(3, 4)
    return m


def read_graph(db_path):
    """Odometry poses by node id, and each closure link once (RTAB-Map stores both directions)."""
    with closing(sqlite3.connect(f'file:{db_path}?mode=ro', uri=True)) as db:
        poses = {i: _matrix(p) for i, p in db.execute('SELECT id, pose FROM Node')}
        links = [(f, t, k, _matrix(tr)) for f, t, k, tr in db.execute(
            'SELECT from_id, to_id, type, transform FROM Link WHERE type IN (1, 2) '
            'AND from_id > to_id ORDER BY from_id, to_id')]
    return poses, links


def closure_errors(poses, links):
    """(from, to, metres, degrees) between each link and the odometry between its nodes."""
    errors = []
    for f, t, _, transform in links:
        if f not in poses or t not in poses:
            continue
        odometry = np.linalg.inv(poses[f]) @ poses[t]
        d = np.linalg.inv(odometry) @ transform
        errors.append((f, t, math.hypot(d[0, 3], d[1, 3]),
                       abs(math.degrees(math.atan2(d[1, 0], d[0, 0])))))
    return errors


def _wrong(errors):
    return sum(1 for _, _, m, deg in errors if m > MAX_CLOSURE_ERROR_M or deg > MAX_CLOSURE_ERROR_DEG)


def wall_recall(learner, reference, slack=WALL_SLACK_CELLS):
    """Share of the reference's occupied cells with a learner occupied cell within `slack` cells."""
    occupied = learner.states == OCCUPIED
    padded = np.pad(occupied, slack)
    h, w = occupied.shape
    grown = np.zeros_like(occupied)
    for dy in range(2 * slack + 1):
        for dx in range(2 * slack + 1):
            grown |= padded[dy:dy + h, dx:dx + w]
    grown_map = GridMap(np.where(grown, OCCUPIED, FREE).astype(np.int8),
                        learner.resolution, learner.origin)
    x, y = reference.cell_centres()
    walls = reference.states == OCCUPIED
    if not walls.any():
        return 0.0
    hit = grown_map.states_at(x[walls], y[walls]) == OCCUPIED
    return float(np.count_nonzero(hit)) / float(np.count_nonzero(walls))


def db_in_use(db_path, proc_root='/proc'):
    """True if a running process (rtabmap, until the launch is closed) has the file open."""
    target = os.path.realpath(db_path)
    for pid in os.listdir(proc_root):
        if not pid.isdigit():
            continue
        fd_dir = os.path.join(proc_root, pid, 'fd')
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue
        for fd in fds:
            try:
                if os.path.realpath(os.path.join(fd_dir, fd)) == target:
                    return True
            except OSError:
                continue
    return False


def evaluate(learner, reference, closures, missing=False):
    """Levels for a learner's grid (None if it could not be made) and its closure errors."""
    known = 0.0 if learner is None else learner.area(FREE) + learner.area(OCCUPIED)
    if known < MIN_KNOWN_M2:
        return [Level(1, 'fail', {'known': known, 'missing': missing}),
                Level(2, 'skip', {'after': 1}), Level(3, 'skip', {'after': 1})]
    share, walls = coverage(learner, reference), wall_recall(learner, reference)
    level2 = {'coverage': share, 'walls': walls}
    if share < MIN_COVERAGE or walls < MIN_WALLS:
        return [Level(1, 'pass', {'known': known}), Level(2, 'fail', level2),
                Level(3, 'skip', {'after': 2})]
    closures = closures or []
    wrong, ratio = _wrong(closures), thickness_ratio(learner, reference)
    straight = len(closures) > 0 and wrong == 0 and ratio <= MAX_THICKNESS
    return [Level(1, 'pass', {'known': known}), Level(2, 'pass', level2),
            Level(3, 'pass' if straight else 'fail',
                  {'closures': len(closures), 'wrong': wrong, 'ratio': ratio})]


_TITLES = {1: 'มีแผนที่', 2: 'เห็นห้องและผนัง', 3: 'แผนที่ไม่เบี้ยว'}


def format_report(levels):
    """Thai text for the terminal. Hints describe symptoms, never parameters."""
    lines = []
    for level in levels:
        head = f'ด่าน {level.number} {_TITLES[level.number]}:'
        n = level.numbers
        verdict = 'ผ่าน' if level.status == 'pass' else 'ไม่ผ่าน'
        if level.status == 'skip':
            lines.append(f'{head} ยังไม่ตรวจ (ผ่านด่าน {n["after"]} ก่อน)')
        elif level.number == 1 and n.get('missing'):
            lines.append(f'{head} ไม่ผ่าน - ไม่พบไฟล์แผนที่')
            lines.append('  คำใบ้: ตอน launch ใส่ map:= ชื่อเดียวกับที่ตรวจไหม? '
                         'แผนที่ถูกบันทึกตอนปิด launch (Ctrl+C) ปิดแล้วหรือยัง')
        elif level.number == 1:
            lines.append(f'{head} {verdict} (รู้จักพื้นที่ {n["known"]:.1f} ตร.ม.)')
            if level.status == 'fail':
                lines.append('  คำใบ้: แผนที่เห็นแค่รอบ ๆ ตัวหุ่นไหม? '
                             'ภาพความลึกจากกล้องถูกเอามาวาดแผนที่ไกลแค่ไหน')
        elif level.number == 2:
            lines.append(f'{head} {verdict} (สำรวจได้ {n["coverage"]:.0%} ต้องได้อย่างน้อย '
                         f'{MIN_COVERAGE:.0%}, เจอผนัง {n["walls"]:.0%} ต้องได้อย่างน้อย {MIN_WALLS:.0%})')
            if level.status == 'fail' and n['walls'] < MIN_WALLS:
                lines.append('  คำใบ้: ใน RViz เห็นพื้นแต่แทบไม่เห็นผนังไหม? กล้องหันไปทางไหนอยู่')
            elif level.status == 'fail':
                lines.append('  คำใบ้: ขับเข้าไปทั้งสองห้องและทางเดินหรือยัง?')
        else:
            lines.append(f'{head} {verdict} (จำที่เดิมได้ {n["closures"]} ครั้ง, จำผิดที่ '
                         f'{n["wrong"]} ครั้ง, ผนังหนา {n["ratio"]:.1f} เท่าของเฉลย '
                         f'ต้องไม่เกิน {MAX_THICKNESS:g} เท่า)')
            if level.status == 'fail' and n['closures'] == 0:
                lines.append('  คำใบ้: ขับกลับมาที่จุดเริ่มหรือยัง? '
                             'หุ่นต้องกลับมาเห็นที่เดิมถึงจะรู้ว่าเคยมาแล้ว')
            elif level.status == 'fail' and n['wrong'] > 0:
                lines.append('  คำใบ้: หุ่นคิดว่ากลับมาที่เดิมทั้งที่ไม่ใช่ ผนังจึงซ้อนหรือเบี้ยว - '
                             'เกณฑ์ที่ใช้ตัดสินว่าภาพสองภาพเป็นที่เดียวกันหลวมไปไหม')
            elif level.status == 'fail':
                lines.append('  คำใบ้: ผนังในแผนที่ซ้อนเป็นหลายชั้นหรือหนาเกินไหม?')
    if all(level.status == 'pass' for level in levels):
        lines.append('ผ่านครบทุกด่าน!')
    return '\n'.join(lines)
```

- [ ] **Step 4: Run — expect PASS:** `PYTHONPATH=. python3 -m pytest test/test_vslam_check.py -q`. If `test_thick_walls_fail_level_three` passes level 2 but the ratio is below 1.2, raise the stripe density (`% 5`) in the test, not the threshold. Register `test_vslam_check` in `CMakeLists.txt`.

---

### Task 6: `check_vslam.py` and `tools/drive_route.py`

**Files:**
- Create: `scripts/check_vslam.py`, `tools/drive_route.py`
- Modify: `CMakeLists.txt` (install `scripts/check_vslam.py`), `package.xml` (`<exec_depend>rtabmap</exec_depend>` after `rtabmap_slam`)
- Test: `test/test_check_vslam.py`

**Interfaces:**
- Consumes: `vslam_check.*` (Task 5), `maps.find_map`, `maps.workspace_maps_dir('vslam')`, `slam_check.load_map`, `slam_check.MapLoadError`.
- Produces: `check_vslam.export_grid(db_path, work_dir, run=subprocess.run) -> str | None` (yaml path, None on failure; prints the Thai reason); `check_vslam.main(argv=None) -> int`.

- [ ] **Step 1: Failing tests** `test/test_check_vslam.py` (the script is importable: `scripts/` goes on `sys.path` in the test):

```python
import pathlib
import subprocess
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'scripts'))
import check_vslam  # noqa: E402


def test_export_failure_is_reported(tmp_path, capsys):
    def missing(*args, **kwargs):
        raise FileNotFoundError('rtabmap-export')
    assert check_vslam.export_grid('x.db', str(tmp_path), run=missing) is None
    assert 'rtabmap-export' in capsys.readouterr().out

    def failing(*args, **kwargs):
        raise subprocess.CalledProcessError(1, 'rtabmap-export', output='boom')
    assert check_vslam.export_grid('x.db', str(tmp_path), run=failing) is None


def test_database_still_open_is_refused(tmp_path, monkeypatch, capsys):
    db = tmp_path / 'mine.db'
    db.write_bytes(b'')
    monkeypatch.setattr(check_vslam.vslam_check, 'db_in_use', lambda path, proc_root='/proc': True)
    monkeypatch.setattr(check_vslam, 'reference_path', lambda: str(tmp_path / 'ref.yaml'))
    assert check_vslam.main([str(db)]) == 1
    assert 'Ctrl+C' in capsys.readouterr().out
```

- [ ] **Step 2: Run — expect FAIL** (`ModuleNotFoundError: check_vslam`).

- [ ] **Step 3: Implement** `scripts/check_vslam.py`:

```python
#!/usr/bin/env python3
"""ตรวจแผนที่ของโจทย์ V-SLAM ว่าผ่านกี่ด่าน

    ros2 run rospider_gazebo check_vslam.py myvslam          # ROSpider/maps/vslam/myvslam.db
    ros2 run rospider_gazebo check_vslam.py ~/somewhere/m.db

Copies the database to a temp folder (rtabmap-export must not touch the participant's file),
exports its 2D grid with rtabmap-export, reads its graph, and scores both against
maps/vslam_challenge.yaml; see rospider_gazebo/vslam_check.py.
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile

from rospider_gazebo import maps, slam_check, vslam_check


def reference_path():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('rospider_gazebo'), 'maps', 'vslam_challenge.yaml')


def export_grid(db_path, work_dir, run=subprocess.run):
    """rtabmap-export's 2D grid of `db_path` as a map yaml in `work_dir`, or None (reason printed)."""
    try:
        run(['rtabmap-export', '--map', '--output', 'grid', '--output_dir', work_dir, db_path],
            check=True, capture_output=True, text=True, timeout=300)
    except FileNotFoundError:
        print('ไม่พบคำสั่ง rtabmap-export - ติดตั้ง ros-jazzy-rtabmap ก่อน')
        return None
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as err:
        print(f'rtabmap-export สร้างแผนที่จากไฟล์นี้ไม่ได้ ({err})')
        return None
    path = os.path.join(work_dir, 'grid.yaml')
    return path if os.path.isfile(path) else None


def main(argv=None):
    parser = argparse.ArgumentParser(description='ตรวจแผนที่ของโจทย์ V-SLAM')
    parser.add_argument('map', help='ชื่อแผนที่ใน ROSpider/maps/vslam (เช่น myvslam) หรือ path ของไฟล์ .db')
    parser.add_argument('--reference', default=None, help='แผนที่อ้างอิง (สำหรับวิทยากร)')
    args = parser.parse_args(argv)

    if os.sep in args.map or args.map.startswith('~'):
        workspace_maps = os.path.dirname(os.path.abspath(os.path.expanduser(args.map)))
    else:
        workspace_maps = maps.workspace_maps_dir('vslam')
    path = maps.find_map(args.map, '.db', workspace_maps, os.getcwd())
    reference_file = args.reference or reference_path()
    if path is not None and vslam_check.db_in_use(path):
        print(f'แผนที่ {path} ยังเปิดอยู่ - ปิด launch ของโจทย์ก่อน (กด Ctrl+C แล้วรอจนปิดเสร็จ) '
              'แผนที่จะถูกบันทึกตอนปิด')
        return 1
    reference = slam_check.load_map(reference_file)
    learner, closures = None, None
    if path is None:
        print(f"ไม่พบแผนที่ '{args.map}' (หาใน {workspace_maps} และในโฟลเดอร์ปัจจุบัน)")
    else:
        print(f'ตรวจแผนที่ {path}')
        with tempfile.TemporaryDirectory(prefix='check_vslam_') as work:
            copy = os.path.join(work, 'map.db')
            shutil.copyfile(path, copy)
            grid = export_grid(copy, work)
            if grid is None:
                return 1
            try:
                learner = slam_check.load_map(grid)
                poses, links = vslam_check.read_graph(copy)
                closures = vslam_check.closure_errors(poses, links)
            except (slam_check.MapLoadError, vslam_check.sqlite3.Error) as err:
                print(f'อ่านแผนที่ไม่ได้: {err}')
                return 1
    levels = vslam_check.evaluate(learner, reference, closures, missing=path is None)
    print(vslam_check.format_report(levels))
    return 0 if all(level.status == 'pass' for level in levels) else 1


if __name__ == '__main__':
    sys.exit(main())
```

Note: in `test_database_still_open_is_refused` the refusal must happen before the reference is loaded (the test's reference file does not exist) — the order above does that.

- [ ] **Step 4: Run — expect PASS:** `PYTHONPATH=. python3 -m pytest test/test_check_vslam.py -q`. Register `test_check_vslam` in `CMakeLists.txt`; add `scripts/check_vslam.py` to the first `install(PROGRAMS ...)` list; add the `package.xml` dependency.

- [ ] **Step 5: `tools/drive_route.py`** (instructors: record the answer-key map; used by Tasks 7-9). Turn to face each waypoint, then walk forward (camera looks where it goes), end facing +x at the origin:

```python
#!/usr/bin/env python3
"""Drive the exercise room's route like a participant with teleop: turn to face each waypoint,
walk to it, return to the start facing +x (the revisit gives V-SLAM its loop closure).

    python3 tools/drive_route.py            # both rooms (~300 s)
    python3 tools/drive_route.py --room-a   # room A only
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

ROUTE = [(0.5, 0.55), (1.5, 0.55), (1.5, -1.6), (2.5, -1.6), (3.6, -1.6), (3.6, -0.3), (3.5, 0.4),
         (3.5, 1.5), (3.5, 0.4), (2.9, -0.4), (2.4, -1.5), (1.5, -1.6), (1.5, 0.55), (0.5, 0.55),
         (0.5, -1.5), (-0.6, -1.5), (-0.6, 0.5), (0.3, 1.5), (0.0, 0.0)]
ROOM_A = [(0.5, -1.5), (-0.6, -1.5), (-0.6, 0.5), (0.3, 1.5), (0.0, 0.0)]
SPEED, TURN, TOLERANCE, LEG_TIMEOUT = 0.15, 0.4, 0.06, 60.0


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--room-a', action='store_true')
    route = ROOM_A if parser.parse_args().room_a else ROUTE
    rclpy.init()
    node = rclpy.create_node('drive_route')
    pose = {}

    def on_odom(msg):
        q = msg.pose.pose.orientation
        pose.update(x=msg.pose.pose.position.x, y=msg.pose.pose.position.y,
                    yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    node.create_subscription(Odometry, '/odom', on_odom, 10)
    pub = node.create_publisher(Twist, '/controller/cmd_vel', 1)
    while 'x' not in pose:
        rclpy.spin_once(node, timeout_sec=0.1)

    def turn_to(heading):
        end = time.time() + 30.0
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            error = wrap(heading - pose['yaw'])
            if abs(error) < 0.05:
                break
            twist = Twist()
            twist.angular.z = max(-TURN, min(TURN, 1.5 * error))
            pub.publish(twist)
        pub.publish(Twist())

    start = time.time()
    for gx, gy in route:
        turn_to(math.atan2(gy - pose['y'], gx - pose['x']))
        end = time.time() + LEG_TIMEOUT
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            dx, dy = gx - pose['x'], gy - pose['y']
            distance = math.hypot(dx, dy)
            if distance < TOLERANCE:
                break
            error = wrap(math.atan2(dy, dx) - pose['yaw'])
            twist = Twist()
            twist.linear.x = min(SPEED, distance) * max(0.0, math.cos(error))
            twist.angular.z = max(-TURN, min(TURN, 1.5 * error))
            pub.publish(twist)
        else:
            print(f'leg to ({gx}, {gy}) timed out at ({pose["x"]:.2f}, {pose["y"]:.2f})')
    turn_to(0.0)
    pub.publish(Twist())
    print(f'route done in {time.time() - start:.0f} s', flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

`python3 -m py_compile tools/drive_route.py` → no output.

---

### Task 7: `vslam.launch.py` changes and `vslam_challenge.launch.py`; record the reference

**Files:**
- Modify: `launch/vslam.launch.py`, `rospider_gazebo/maps.py`
- Create: `launch/vslam_challenge.launch.py`, `maps/vslam_challenge.{pgm,yaml}`
- Test: `test/test_maps.py`

**Interfaces:**
- Consumes: `vslam_params.mapping_params_file`, `challenge_params.ensure_user_file`, `lifecycle_activate.py NODE...` (Task 2).
- Produces: `maps.localization_copy(db_path) -> str`; `vslam.launch.py` args `vslam_params` (default `''` = `config/vslam.yaml`) and `arm_pose` (`init|horizontal`, default `horizontal`); `vslam_challenge.launch.py` args `params`, `map` (default `myvslam`), `gui`, `rviz`.

- [ ] **Step 1: Failing test** (append to `test/test_maps.py`):

```python
def test_localization_copy_leaves_the_map_untouched(tmp_path):
    db = tmp_path / 'mine.db'
    db.write_bytes(b'original')
    copy = maps.localization_copy(str(db))
    assert copy != str(db) and open(copy, 'rb').read() == b'original'
    open(copy, 'wb').write(b'changed by rtabmap')
    assert db.read_bytes() == b'original'


def test_localization_copy_of_a_missing_map_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        maps.localization_copy(str(tmp_path / 'none.db'))
```

(`import pytest` at the top if the file lacks it.)

- [ ] **Step 2: Run — expect FAIL**, then implement in `rospider_gazebo/maps.py`:

```python
def localization_copy(db_path):
    """A temp copy of an RTAB-Map database to localize on. rtabmap in localization mode writes
    its 2D grid back into the database on shutdown, with ghost walls from that run (camera poses
    lag the arm's TF); every later run then loads them as the map -- measured: the answer key
    then failed G2 with 'no path' in 3 of 4 runs. The participant's file is never opened."""
    import shutil
    import tempfile

    if not os.path.isfile(db_path):
        raise FileNotFoundError(db_path)
    copy = os.path.join(tempfile.mkdtemp(prefix='vslam_localize_'), os.path.basename(db_path))
    shutil.copyfile(db_path, copy)
    return copy
```

Run `test_maps.py` → PASS.

- [ ] **Step 3: `launch/vslam.launch.py`.** In `launch_setup`:
  - `vslam_params = LaunchConfiguration('vslam_params').perform(context) or os.path.join(pkg, 'config', 'vslam.yaml')`
  - Gazebo include: `'arm_pose': LaunchConfiguration('arm_pose')` (instead of the literal `'horizontal'`); keep the comment "The camera must look ahead" and add "(vslam_challenge sets init on purpose)".
  - In localization mode: `database = maps.localization_copy(database)` before building the node (import `localization_copy` with `resolve_map`), and `overrides['map_always_update'] = False` next to `Mem/IncrementalMemory`, with the comment "map_always_update adds this run's views to /map at their (lagging) poses: ghost walls in the planner's map".
  - Nav2 include: `'autostart': 'false'`, and append after it:

```python
        # Nav2's lifecycle_manager waits forever when a change_state reply is lost (seen in ~1 of
        # 12 launches); the activator re-reads each node's state instead. Same order as
        # nav2_bringup/launch/navigation_launch.py's lifecycle_nodes.
        actions.append(Node(
            package='rospider_gazebo', executable='lifecycle_activate.py', output='screen',
            arguments=list(NAV2_NODES) + ['--timeout', '180'],
        ))
```

  with, at module level, `NAV2_NODES = ('controller_server', 'smoother_server', 'planner_server', 'route_server', 'behavior_server', 'velocity_smoother', 'collision_monitor', 'bt_navigator', 'waypoint_follower', 'docking_server')` — first confirm it matches: `sed -n '/lifecycle_nodes = \[/,/\]/p' /opt/ros/jazzy/share/nav2_bringup/launch/navigation_launch.py`.
  - The `LogInfo` for localization says `'loading (a copy of)'`.
  - Declare: `DeclareLaunchArgument('vslam_params', default_value='', description='RTAB-Map parameters (default: config/vslam.yaml)')`, `DeclareLaunchArgument('arm_pose', default_value='horizontal', choices=['init', 'horizontal'])`.

- [ ] **Step 4: `launch/vslam_challenge.launch.py`** — same structure as `slam_challenge.launch.py`:

```python
"""`ros2 launch rospider_gazebo vslam_challenge.launch.py map:=myvslam` -- the V-SLAM mapping exercise.

The maze room (worlds/slam_challenge.sdf) with vslam.launch.py (camera only), run on
config/vslam_challenge.yaml (three values deliberately wrong), edited in the source tree and
merged over config/vslam.yaml by rospider_gazebo/vslam_params.py. Drive through both rooms
and back to the start, close the launch (Ctrl+C: RTAB-Map writes the database), then
`ros2 run rospider_gazebo check_vslam.py <map>`.

  params  another participant file (default: config/vslam_challenge.yaml); instructors pass
          config/vslam_challenge_solved.yaml's path
  map     the new map's name, kept as <workspace>/maps/vslam/<map>.db (overwritten)
  gui, rviz  as vslam.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from rospider_gazebo import challenge_params, vslam_params


def launch_setup(context):
    pkg = get_package_share_directory('rospider_gazebo')
    template = os.path.join(pkg, 'config', 'vslam_challenge.yaml')
    user = os.path.expanduser(LaunchConfiguration('params').perform(context)) or template
    created = challenge_params.ensure_user_file(template, user)
    user = os.path.realpath(user)       # under --symlink-install: the source copy to edit
    try:
        merged, arm_pose = vslam_params.mapping_params_file(
            os.path.join(pkg, 'config', 'vslam.yaml'), user)
    except challenge_params.ChallengeConfigError as err:
        raise RuntimeError(f'ไฟล์โจทย์ใช้ไม่ได้: {err}') from None
    note = ('สร้างไฟล์โจทย์ใหม่ที่' if created else 'ใช้ไฟล์โจทย์') + f' {user}'
    return [
        LogInfo(msg=f'[vslam_challenge] {note} -- แก้ไฟล์นี้แล้ว launch ใหม่; '
                    'ขับให้ทั่วแล้วกลับจุดเริ่ม จากนั้นปิด launch (Ctrl+C) ก่อนตรวจ'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'vslam.launch.py')),
            launch_arguments={
                'world': 'slam_challenge',
                'map': LaunchConfiguration('map'),
                'vslam_params': merged,
                'arm_pose': arm_pose,
                'localization': 'false',
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
            }.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value='',
                              description="the participant's values (default: config/vslam_challenge.yaml)"),
        DeclareLaunchArgument('map', default_value='myvslam'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
```

- [ ] **Step 5: Build and run all unit tests.** From `ROSpider/`: build (Global Constraints), then `colcon test --packages-select rospider_gazebo --event-handlers console_direct+ > /tmp/claude-1000/<scratch>/colcon_test.log 2>&1; colcon test-result --verbose | tail -3`. Expected: 0 failures.

- [ ] **Step 6: Record the answer-key map (sim).**

```bash
simenv; cd $W
setsid ros2 launch rospider_gazebo vslam_challenge.launch.py rviz:=false map:=vslam_answer \
  params:=$W/src/simulations/rospider_gazebo/config/vslam_challenge_solved.yaml > <scratch>/rec.log 2>&1 &
PGID=$!
for i in $(seq 1 90); do timeout 3 ros2 topic echo --once /map --no-arr >/dev/null 2>&1 && break; sleep 1; done
python3 src/simulations/rospider_gazebo/tools/drive_route.py
stop $PGID
rtabmap-export --map --output vslam_challenge --output_dir src/simulations/rospider_gazebo/maps maps/vslam/vslam_answer.db
python3 -c "from rospider_gazebo import vslam_check as v; p,l=v.read_graph('maps/vslam/vslam_answer.db'); e=v.closure_errors(p,l); print(len(e), v._wrong(e))"
```

Expected: `route done in ~300 s`; `maps/vslam_challenge.pgm/.yaml` written (~122 x 102 cells); closures ≥ 5, wrong 0. If wrong > 0, the room still aliases: stop and report (do not raise thresholds). Also confirm the log shows `arm_pose` horizontal (`grep -c "joint4" <scratch>/rec.log` is not the test — check `/joint_states` is not needed; the solved run's walls prove it) and that the grid looks like the room (render it to PNG and look).

- [ ] **Step 7: Check the answer key scores itself** (rebuild so the reference is installed): `ros2 run rospider_gazebo check_vslam.py vslam_answer` → `ผ่านครบทุกด่าน!` (100%, 100%, 1.0x, 0 wrong).

---

### Task 8: Measure the mapping exercise in the sim

**Files:** none new (values may change in `config/vslam_challenge.yaml` / `rospider_gazebo/vslam_check.py` constants only by a ledgered ruling, per the spec's rule).

Each run: fresh launch with a scratch params file (copy of `config/vslam_challenge.yaml` with the listed changes), `tools/drive_route.py`, `stop`, `check_vslam.py <map>`.

| Run | File | Expected result |
|---|---|---|
| M1 | shipped starting file | level 1 fails (known < 1 m²) |
| M2 | `Grid/RangeMax "5.0"` fixed | level 2 fails on walls (~34%) |
| M3 | + `camera_view: ahead` | level 3 fails (wrong closures ≥ 1, ratio > 1.2) |
| M4 | + `Vis/MinInliers "20"` (= answer key) | all pass |
| M5 | answer key again (`params:=...solved`) | all pass |
| M6 | answer key, `tools/drive_route.py --room-a` | level 2 fails (coverage) |

- [ ] **Step 1: M1-M6**, one at a time; record known m², coverage, walls, closures, wrong, ratio for each in the ledger.
- [ ] **Step 2:** If M3 passes level 3 (no wrong closure that run), repeat M3 twice. If it passes in 2 of 3, lower `Vis/MinInliers` in the starting file to `"3"` and re-run M3 twice; ledger the ruling.
- [ ] **Step 3:** If any other row differs from "Expected", follow the spec: replace the wrong value with another from the same file (not the threshold), re-measure, ledger it.
- [ ] **Step 4:** Put the measured numbers into `vslam_check.py`'s docstring (replacing the probe numbers) and the spec's table.
- [ ] **Step 5:** Record M1-M4's wall-clock times (launch to check) to confirm a pass fits ~25 minutes.

---

### Task 9: `vslam_nav_challenge.launch.py`; measure it

**Files:**
- Create: `launch/vslam_nav_challenge.launch.py`

**Interfaces:**
- Consumes: `vslam_params.nav_params_file`, `vslam_params.require_map`, `maps.workspace_maps_dir('vslam')`, `vslam.launch.py` (Task 7), `check_nav.py` (Task 3).

- [ ] **Step 1: Write the launch file:**

```python
"""`ros2 launch rospider_gazebo vslam_nav_challenge.launch.py map:=myvslam` -- navigating on V-SLAM.

Loads the participant's own map from the V-SLAM mapping exercise (<workspace>/maps/vslam/<map>.db;
vslam.launch.py localizes on a temp copy, so the file is never changed) and runs Nav2 with
Regulated Pure Pursuit on config/vslam_nav2_params.yaml, with the participant's values from
config/vslam_nav_challenge.yaml (two deliberately wrong) merged by rospider_gazebo/vslam_params.py.
Score with `ros2 run rospider_gazebo check_nav.py` (the Nav2 exercise's course). The robot must
start where the map was started (the spawn point): relocalization from elsewhere is not reliable.

  params  another participant file (default: config/vslam_nav_challenge.yaml)
  map     the V-SLAM map's name or a path to a .db
  gui, rviz  as vslam.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from rospider_gazebo import challenge_params, maps, vslam_params


def launch_setup(context):
    pkg = get_package_share_directory('rospider_gazebo')
    template = os.path.join(pkg, 'config', 'vslam_nav_challenge.yaml')
    user = os.path.expanduser(LaunchConfiguration('params').perform(context)) or template
    created = challenge_params.ensure_user_file(template, user)
    user = os.path.realpath(user)
    value = LaunchConfiguration('map').perform(context)
    try:
        database = vslam_params.require_map(value, maps.workspace_maps_dir('vslam'), os.getcwd())
        merged, arm_pose = vslam_params.nav_params_file(
            os.path.join(pkg, 'config', 'vslam_nav2_params.yaml'), user)
    except challenge_params.ChallengeConfigError as err:
        raise RuntimeError(str(err)) from None
    note = ('สร้างไฟล์โจทย์ใหม่ที่' if created else 'ใช้ไฟล์โจทย์') + f' {user}'
    return [
        LogInfo(msg=f'[vslam_nav_challenge] {note} -- แก้ไฟล์นี้แล้ว launch ใหม่; แผนที่ {database}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'vslam.launch.py')),
            launch_arguments={
                'world': 'slam_challenge',
                'map': database,
                'localization': 'true',
                'params_file': merged,
                'arm_pose': arm_pose,
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
            }.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=''),
        DeclareLaunchArgument('map', default_value='myvslam'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
```

Build. `ros2 launch rospider_gazebo vslam_nav_challenge.launch.py map:=nothere rviz:=false` → exits at once with the Thai "ไม่พบแผนที่ V-SLAM 'nothere' ... vslam_challenge" message, no Gazebo window.

- [ ] **Step 2: Measure** (map `vslam_answer` from Task 7; each run: fresh launch, wait for `ros2 lifecycle get /bt_navigator` = `active [3]`, `ros2 run rospider_gazebo check_nav.py`, `stop`; confirm `md5sum maps/vslam/vslam_answer.db` is unchanged after every run):

| Run | File | Expected |
|---|---|---|
| N1 | shipped starting file | level 1 fails (G1 aborted/timeout) |
| N2 | `camera_view: ahead` | level 2 fails (`no_path` to G2) |
| N3 | + `robot_radius: 0.15` | level 3 fails (`over_limit`) |
| N4 | + `max_vel_x: 0.15` (= answer key) | all pass, 150-180 s |
| N5 | answer key again | all pass |

- [ ] **Step 3:** If N2 does not give `no_path` (the V-SLAM map's doorway is wider), try `robot_radius` 0.33, then 0.35, one run each; keep the first that blocks G2 while N3's value 0.15 still passes; ledger the ruling and update the file, the spec table and the Thai comment if needed.
- [ ] **Step 4:** In every run check the log for `not active after` from `lifecycle_activate.py` (it must never appear) and that nothing prints "Deactivating" within 60 s of activation (a broken bond would do that). If it appears, stop and report.
- [ ] **Step 5:** Record the measured numbers in the spec's table and in the header of `launch/vslam_nav_challenge.launch.py`.

---

### Task 10: Docs, full suite, hand-off

**Files:**
- Modify: `ROSpider/SIMULATION.md` (V-SLAM section), `CLAUDE.md` (Simulation section)

- [ ] **Step 1: `SIMULATION.md`.** Under the V-SLAM section, a "โจทย์" block in the style of the SLAM and Nav2 ones:

```markdown
**โจทย์ V-SLAM (สร้างแผนที่ด้วยกล้อง)**

    ros2 launch rospider_gazebo vslam_challenge.launch.py map:=myvslam

1. แก้ค่าในไฟล์โจทย์ `src/simulations/rospider_gazebo/config/vslam_challenge.yaml` แล้วปิด-เปิด launch ใหม่
2. ขับหุ่นด้วย teleop ให้ทั่วทั้งสองห้อง แล้วกลับมาที่จุดเริ่ม
3. ปิด launch (Ctrl+C) รอจนปิดเสร็จ แผนที่ถูกบันทึกตอนนี้
4. ตรวจ: `ros2 run rospider_gazebo check_vslam.py myvslam`

ด่าน: 1 มีแผนที่ · 2 เห็นห้องและผนัง · 3 แผนที่ไม่เบี้ยว (จำที่เดิมได้และไม่จำผิดที่)

**โจทย์นำทางด้วย V-SLAM** (ใช้แผนที่จากโจทย์ข้างบน)

    ros2 launch rospider_gazebo vslam_nav_challenge.launch.py map:=myvslam

แก้ `config/vslam_nav_challenge.yaml` แล้วตรวจด้วย `ros2 run rospider_gazebo check_nav.py` (เส้นทางเดียวกับโจทย์ Nav2)
วิทยากร: แผนที่เฉลยสร้างด้วย `params:=<path ของ vslam_challenge_solved.yaml>` + `python3 src/simulations/rospider_gazebo/tools/drive_route.py`
```

- [ ] **Step 2: `CLAUDE.md`.** Add a bullet after the Nav2 exercise bullet: the V-SLAM exercises (files, levels, measured values from Tasks 8-9), the face textures and why, `rtabmap-export` over `map_saver_cli`, localization on a copy + `map_always_update` false and why (ghost walls), Nav2 brought up by `lifecycle_activate.py`, wrong closures judged against exact sim odometry, `check_nav.py`'s resend. Update the V-SLAM bullet's `localization:=true` sentence accordingly and the SLAM-exercise bullet's world description (faces).

- [ ] **Step 3: Full suite.** Build, `colcon test --packages-select rospider_gazebo` (log to scratch), `colcon test-result --verbose | tail -3` → 0 failures; `cd src/simulations/rospider_gazebo && PYTHONPATH=. python3 -m pytest test -q` → all pass. Confirm no sim processes remain (`pgrep -x rtabmap; pgrep -f "gz sim"` → nothing) and `git status` shows only the intended files (restore both participant files to the shipped values if a run edited them; `maps/vslam/*.db` stays git-ignored).

- [ ] **Step 4: Ask the user** (no commit without asking): report the measured tables, every ruling, and ask whether to commit (one commit for the feature) and push.
