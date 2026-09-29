# Final Mission Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A closing workshop exercise. The participant writes `config/mission_challenge.yaml` (a
`speed` and a list of steps); `check_mission.py` runs it on the maze room's full stack (Nav2,
AprilTag tracking, depth-camera grasping) and scores three levels: reached room B, holding the blue
block, the blue block on the yellow pad.

**Architecture:**
- Two pure modules, both unit tested:
  - `mission_plan.py`: the file → typed steps, or a Thai refusal
  - `mission_check.py`: the scene constants, and the facts a run records → levels and a Thai report
- One runner/checker script drives the existing nodes through their services: Nav2 actions,
  `apriltag_track ~/set_running` plus a live `target_tag`, `track_and_grab ~/pick`, and
  `pick_and_place ~/place`.
- The Nav2 client code moves out of `check_nav.py` into `rospider_gazebo/nav_client.py`, so both
  checkers share it.
- A generated world adds two tag stations and a pad to the maze room.

**Tech Stack:** ROS 2 Jazzy, Gazebo Harmonic, Nav2 (RPP), rclpy, pytest.

**Spec:** `docs/superpowers/specs/2026-09-29-mission-challenge-design.md`

## Global Constraints

- Run everything from `ROSpider/` with `source /opt/ros/jazzy/setup.bash`, `need_compile=True`.
  Build with `colcon build --packages-select rospider_gazebo --symlink-install`, then
  `source install/local_setup.bash`.
- Unit tests: `cd ROSpider/src/simulations/rospider_gazebo && python3 -m pytest -q test`.
  Package tests: `colcon test --packages-select rospider_gazebo && colcon test-result --verbose`.
- Test sims run in their own partition so they never touch the user's:
  `export GZ_PARTITION=missionplan ROS_DOMAIN_ID=68`. Stop them by that partition, never with a
  bare `pkill -f` pattern: that once killed the agent's own shell.
- Run live sim measurements one at a time, never two in parallel: they share the CPU and the
  real-time factor.
- Participant-facing text (hints, errors, file comments, SIMULATION.md) is Thai.
- Hints describe what happened and never name a key.
- `speed` is allowed from 0.05 to 0.20 m/s, and is not scored.
- Levels:
  - 1: robot x > 2.1 at some point
  - 2: blue block z > 0.15 after a `pick`
  - 3: blue block at rest with z < 0.05, within the pad tolerance of the pad centre (0.40, 0.55)
- **No commits.** The user commits only when they ask (standing preference). Leave work uncommitted
  at the end of each task.
- The participant files in `config/` hold the user's played values right now: `line_challenge`,
  `vslam_challenge`, `grasp_challenge`, `shape_challenge`. Never overwrite or `git checkout`
  them.

## Review Focus

1. **The file changes between checks without a relaunch.** The runner reads `mission_challenge.yaml`
   at every run, never a cached copy. The file's own header says a relaunch is only for putting
   the robot and blocks back.
2. **Ctrl+C mid-step.** Every subsystem the runner switched on is switched off (Nav2 goal cancelled,
   `apriltag_track` off, `track_and_grab` stopped) and the robot stands still. Covered by the
   runner's `finally` path in Task 7, and verified live in Task 7 step 6.
3. **A step order that is legal but silly.**
   - `pick` before any `go_to_tag`, `place` with nothing held, or two `pick` in a row must end with
     a clear Thai reason, never hang.
   - Every wait has a sim-time limit and a wall-time backstop (Task 7).
4. **A block dragged at startup.** The grasp welds form the instant a block spawns.
   `pick_and_place` must still be firing its startup detaches then, or the arm drags both blocks
   off their pedestals across the room.
   - The launch spawns the blocks in the same timer as `pick_and_place` (Task 3).
   - The checker refuses to start if a block is not on its pedestal (Task 7).
5. **The idle `cmd_vel` silence.** After Task 1 nothing may publish on `/controller/cmd_vel` while
   idle. Checked live in Task 3, and re-checked after the full stack runs in Task 7.

---

## File map

| File | Task | Responsibility |
|---|---|---|
| `scripts/apriltag_track.py` | 1 | quiet when off; live `target_tag`; re-pose when switched on |
| `rospider_gazebo/mission_check.py` | 2, 6 | scene constants (2); facts → levels + Thai report (6) |
| `tools/make_mission_world.py`, `worlds/mission_challenge.sdf` | 2 | the scene |
| `test/test_mission_world.py` | 2 | world matches the constants |
| `models/tag_station_3/`, `worlds/textures/tag_3.png` | 2 | station 3 (already on disk, untracked) |
| `launch/mission_challenge.launch.py` | 3 | the stack |
| `rospider_gazebo/nav_client.py` | 4 | Nav2 client shared by `check_nav.py` and `check_mission.py` |
| `scripts/check_nav.py` | 4 | uses `nav_client` |
| `rospider_gazebo/mission_plan.py`, `test/test_mission_plan.py` | 5 | file → steps |
| `rospider_gazebo/grasp_check.py` | 6 | `_side` becomes public `side` |
| `test/test_mission_check.py` | 6 | levels and report |
| `scripts/check_mission.py` | 7 | runner + checker |
| `config/mission_challenge.yaml`, `config/mission_challenge_solved.yaml` | 7 | draft and answer key |
| `CMakeLists.txt` | 2, 5, 6, 7 | install `check_mission.py`; register tests |
| `ROSpider/SIMULATION.md`, `CLAUDE.md`, spec "As built" | 8 | docs |

All paths below are relative to `ROSpider/src/simulations/rospider_gazebo/` unless they start
with `ROSpider/`, `docs/` or `CLAUDE.md`.

---

### Task 1: apriltag_track: quiet when off, live target_tag, re-pose on start

**Files:**
- Modify: `scripts/apriltag_track.py`

**Interfaces:**
- Produces:
  - `/apriltag_track` publishes nothing on `/controller/cmd_vel` while `following` is false,
    except one zero twist at the moment it is switched off.
  - Parameter `target_tag` can be set at run time with `SetParameters` on `/apriltag_track`.
  - `~/set_running true` re-sends `LOOK_POSE`, because another node may have moved the arm.

There is no unit test harness for node scripts in this package (logic that needs tests lives in
`rospider_gazebo/`). This change is three small branches, verified live in step 4.

- [ ] **Step 1: Stop the per-frame stop while off**

In `process()`, replace

```python
        if self.tag is None or not self.following:
            self.stop()
            self.pid_yaw.clear()
            self.pid_distance.clear()
            return frame
```

with

```python
        if not self.following:
            return frame                # off: /controller/cmd_vel belongs to someone else
        if self.tag is None:
            self.stop()
            self.pid_yaw.clear()
            self.pid_distance.clear()
            return frame
```

`set_running_callback` already sends one `self.stop()` when switched off.

- [ ] **Step 2: Re-pose when switched on**

In `set_running_callback`, before `response.success = True`:

```python
        if self.following:
            # Another node (track_and_grab, pick_and_place) may have moved the arm since start-up.
            self.servos.set_servo_position(1.0, LOOK_POSE)
```

- [ ] **Step 3: Make target_tag live**

Add at the top: `from rcl_interfaces.msg import SetParametersResult`. At the end of `__init__`:

```python
        self.add_on_set_parameters_callback(self._on_parameters)
```

Add the method:

```python
    def _on_parameters(self, parameters):
        """target_tag may change at run time (check_mission.py's go_to_tag steps)."""
        for parameter in parameters:
            if parameter.name == 'target_tag':
                self.target_tag = int(parameter.value)
                self.tag = None
                self.pid_yaw.clear()
                self.pid_distance.clear()
        return SetParametersResult(successful=True)
```

Also add one line to the module docstring after "run only one of the two.":

```
Off (start:=false or ~/set_running false) it publishes nothing on /controller/cmd_vel, so Nav2 or
track_and_grab can drive; target_tag can be changed at run time with `ros2 param set`.
```

- [ ] **Step 4: Verify live**

Build. Then in one shell (the test partition):

```bash
ros2 launch rospider_gazebo apriltag_challenge.launch.py gui:=true > /tmp/claude-1000/-home-earth157-entech-hiwonder-ros2-ws/63adbe42-6d1d-46e3-8879-b8b6aaa74920/scratchpad/mission/t1.log 2>&1 &
```

After 35 s, check each of these:

- `timeout 12 ros2 topic hz /controller/cmd_vel`
  Expected: no rate lines, only "WARNING: topic ... does not appear to be published yet" or
  nothing at all.
- `ros2 param set /apriltag_track target_tag 2`, then
  `ros2 param get /apriltag_track target_tag`
  Expected: `Integer value is: 2`.
- Stop the launch by partition. Then run the AprilTag answer key once with the existing checker
  (`scratchpad/tag/crun.sh solved <solved yaml path>`).
  Expected: `ผ่านครบทุกด่าน!` in about 15 s, as the AprilTag spec records.

- [ ] **Step 5: Unit suite still green**

Run: `python3 -m pytest -q test`
Expected: all pass (376 before this plan).

---

### Task 2: Scene constants, world generator, world

**Files:**
- Create: `rospider_gazebo/mission_check.py` (constants only in this task)
- Create: `tools/make_mission_world.py`
- Create: `worlds/mission_challenge.sdf` (generated)
- Create: `test/test_mission_world.py`
- Include: `models/tag_station_3/model.sdf`, `worlds/textures/tag_3.png` (on disk already)
- Modify: `CMakeLists.txt` (register `test_mission_world`)

**Interfaces:**
- Produces, in `rospider_gazebo/mission_check.py`:
  - `STATIONS: dict[int, tuple[float, float, float]]`: tag → (x, y, yaw rad) of the station model
  - `BLOCK_OF_TAG = {2: 'red', 3: 'blue'}`
  - `TARGET_TAG = 3`, `TARGET_COLOR = 'blue'`
  - `PEDESTAL_OFFSET: float`: metres from the pedestal centre toward the robot
  - `BLOCK_Z = 0.105`
  - `block_spawn(color) -> (x, y, z)`
  - `PAD = (0.40, 0.55)`, `PAD_SIZE = 0.12`, `PAD_TOL = 0.06`
  - `ROOM_B_X = 2.1`, `LIFTED_Z = 0.15`, `FLOOR_Z = 0.05`
  - `STOP_DISTANCE: float`: apriltag_track's standoff for a park at the pick spot (Task 3 measures it)
- Produces: `tools/make_mission_world.build() -> str`, the world text.

- [ ] **Step 1: Write the failing test**

`test/test_mission_world.py`:

```python
"""worlds/mission_challenge.sdf is tools/make_mission_world.py's output for mission_check's constants."""
import importlib.util
import math
import pathlib
import xml.etree.ElementTree as ET

from rospider_gazebo import mission_check

PKG = pathlib.Path(__file__).resolve().parents[1]


def _generator():
    spec = importlib.util.spec_from_file_location('make_mission_world',
                                                  PKG / 'tools' / 'make_mission_world.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _models(text):
    world = ET.fromstring(text[text.index('<sdf'):]).find('world')
    return {m.get('name'): m for m in world.findall('model')}


def test_shipped_world_is_the_generators_output():
    assert (PKG / 'worlds' / 'mission_challenge.sdf').read_text() == _generator().build()


def test_stations_and_pad_sit_where_the_checker_expects():
    models = _models(_generator().build())
    for tag, (x, y, yaw) in mission_check.STATIONS.items():
        pose = [float(v) for v in models[f'tag_station_{tag}'].find('pose').text.split()]
        assert pose[0] == x and pose[1] == y and math.isclose(pose[5], yaw, abs_tol=1e-4)
    pad = [float(v) for v in models['mission_pad'].find('pose').text.split()]
    assert (pad[0], pad[1]) == mission_check.PAD


def test_the_maze_room_is_unchanged():
    maze = _models((PKG / 'worlds' / 'slam_challenge.sdf').read_text())
    mission = _models(_generator().build())
    for name in maze:
        assert ET.tostring(maze[name]) == ET.tostring(mission[name])


def test_each_block_spawns_on_its_own_station_toward_the_robot():
    for tag, color in mission_check.BLOCK_OF_TAG.items():
        x, y, yaw = mission_check.STATIONS[tag]
        bx, by, bz = mission_check.block_spawn(color)
        assert math.isclose(math.hypot(bx - x, by - y), mission_check.PEDESTAL_OFFSET)
        assert bz == mission_check.BLOCK_Z
```

- [ ] **Step 2: Run to verify it fails**

Run: `python3 -m pytest -q test/test_mission_world.py`
Expected: FAIL, `ImportError: cannot import name 'mission_check'` (or `ModuleNotFoundError`).

- [ ] **Step 3: Write the constants**

`rospider_gazebo/mission_check.py`:

```python
"""Score the final mission (docs/superpowers/specs/2026-09-29-mission-challenge-design.md).

The scene's constants live here, so tools/make_mission_world.py, mission_challenge.launch.py and
scripts/check_mission.py share one source. The world frame is the map frame (the reference map
was recorded from the spawn at the origin) and the sim's odometry frame. Pure Python, unit tested.
"""

import math

#: tag id -> station model pose (x, y, yaw rad). Both face -x, into room B, against its east wall.
STATIONS = {2: (3.75, 1.0, math.pi), 3: (3.75, -0.3, math.pi)}
BLOCK_OF_TAG = {2: 'red', 3: 'blue'}
TARGET_TAG = 3
TARGET_COLOR = 'blue'
#: A block sits this far from its pedestal's centre toward the robot, and this high (on the 8 cm
#: pedestal). Task 3 of the plan measures the offset with STOP_DISTANCE.
PEDESTAL_OFFSET = 0.035
BLOCK_Z = 0.105
#: apriltag_track's standoff that parks the robot with the block where pick-here grabs it
#: (0.20-0.27 m ahead of base_footprint). Measured in the plan's Task 3.
STOP_DISTANCE = 0.27
#: The yellow pad in room A (visual only), its size, and how far from its centre a block still
#: counts as on it (the centre over the pad).
PAD = (0.40, 0.55)
PAD_SIZE = 0.12
PAD_TOL = 0.06
#: Room B: east of the corridor wall at x = 2.0.
ROOM_B_X = 2.1
#: A held block's centre is at ~0.22 m; one resting on the floor at 0.025 m.
LIFTED_Z = 0.15
FLOOR_Z = 0.05


def block_spawn(color):
    """Where the block of `color` is spawned: on its station's pedestal, toward the robot."""
    tag = next(t for t, c in BLOCK_OF_TAG.items() if c == color)
    x, y, yaw = STATIONS[tag]
    return (x + math.cos(yaw) * PEDESTAL_OFFSET, y + math.sin(yaw) * PEDESTAL_OFFSET, BLOCK_Z)
```

- [ ] **Step 4: Write the generator**

`tools/make_mission_world.py`:

```python
#!/usr/bin/env python3
"""Write worlds/mission_challenge.sdf: worlds/slam_challenge.sdf plus the two tag stations and the
yellow pad of rospider_gazebo/mission_check.py (the final mission's scene). The blocks are
dynamic, so the launch spawns them.

    PYTHONPATH=. python3 tools/make_mission_world.py
"""
import pathlib

from rospider_gazebo import mission_check, stations

PKG = pathlib.Path(__file__).resolve().parents[1]
HEADER = '''<!-- The final mission's scene (docs/superpowers/specs/2026-09-29-mission-challenge-design.md).
     Generated by tools/make_mission_world.py from worlds/slam_challenge.sdf and
     rospider_gazebo/mission_check.py; edit those. -->
'''


def _station(tag, x, y, yaw):
    sdf = stations.station_sdf(tag)
    model = sdf[sdf.index('<model'):sdf.rindex('</model>') + len('</model>')]
    model = model.replace('../../worlds/textures/', 'textures/')
    return '    ' + model.replace('<static>true</static>',
                                  f'<pose>{x} {y} 0 0 0 {yaw:.5f}</pose>\n    <static>true</static>', 1)


def _pad():
    x, y = mission_check.PAD
    size = mission_check.PAD_SIZE
    return f'''    <model name="mission_pad">
      <pose>{x} {y} 0.001 0 0 0</pose>
      <static>true</static>
      <link name="link"><visual name="v"><geometry><box><size>{size} {size} 0.002</size></box></geometry>
        <material><ambient>0.95 0.85 0.1 1</ambient><diffuse>0.95 0.85 0.1 1</diffuse></material></visual></link>
    </model>'''


def build():
    maze = (PKG / 'worlds' / 'slam_challenge.sdf').read_text()
    body = maze[maze.index('<sdf'):maze.rindex('  </world>')]
    body = body.replace('<world name="slam_challenge">', '<world name="mission_challenge">', 1)
    extra = [_station(tag, *pose) for tag, pose in mission_check.STATIONS.items()] + [_pad()]
    return ('<?xml version="1.0"?>\n' + HEADER + body + '\n'.join(extra) + '\n  </world>\n</sdf>\n')


def main():
    (PKG / 'worlds' / 'mission_challenge.sdf').write_text(build())


if __name__ == '__main__':
    main()
```

If `slam_challenge.sdf`'s `<world name=...>` differs, read it first with
`grep -n "<world" worlds/slam_challenge.sdf` and match the replace to it.

- [ ] **Step 5: Generate and run the tests**

Run: `PYTHONPATH=. python3 tools/make_mission_world.py && python3 -m pytest -q test/test_mission_world.py`
Expected: 4 passed.

- [ ] **Step 6: Register the test**

Add to `CMakeLists.txt`, after the `test_grasp_check` entry:

```cmake
  ament_add_pytest_test(test_mission_world test/test_mission_world.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
```

---

### Task 3: The launch, and the measurements the spec asks for first

**Files:**
- Create: `launch/mission_challenge.launch.py`
- Modify: `rospider_gazebo/mission_check.py` (`STOP_DISTANCE`, `PEDESTAL_OFFSET`, and possibly
  `PAD_TOL`, from the measurements)
- Regenerate: `worlds/mission_challenge.sdf`, if a station constant changes
- Scratch (not shipped): `…/scratchpad/mission/*.sh`, `*.py`

**Interfaces:**
- Consumes: `mission_check.STATIONS`, `block_spawn`, `STOP_DISTANCE` (Task 2);
  `nav_params.merged_params_file(nav2_path, user_path) -> str` (existing).
- Produces:
  - `ros2 launch rospider_gazebo mission_challenge.launch.py [map:=] [gui:=] [rviz:=]` starts:
    - Gazebo with `mission_challenge`, and Nav2 with map `slam_challenge` on the Nav2 answer key
    - `color_detect`, `apriltag_detect`, and `pick_and_place` (`auto_start` false)
    - `apriltag_track` (`start` false, `stop_distance` = `STOP_DISTANCE`, `show` false)
    - the `track_and_grab` window (`walk` false, `auto_place` false)
    - blocks `pick_cube_red` and `pick_cube_blue` at `block_spawn()`

- [ ] **Step 1: Write the launch**

`launch/mission_challenge.launch.py`:

```python
"""`ros2 launch rospider_gazebo mission_challenge.launch.py` -- the final mission.

The maze room with two tag stations and a yellow pad (worlds/mission_challenge.sdf), Nav2 on the
reference map with the Nav2 exercise's answer key, and every node the mission's steps drive:
color_detect, apriltag_detect, apriltag_track (off until a go_to_tag step), pick_and_place and the
track_and_grab window. The participant's config/mission_challenge.yaml is read by
`ros2 run rospider_gazebo check_mission.py` at every run, not here; relaunch only to put the robot
and the blocks back. See docs/superpowers/specs/2026-09-29-mission-challenge-design.md.

  map        your own map from the SLAM exercise (a name in ROSpider/maps, or a path);
             default: the reference map maps/slam_challenge
  gui, rviz  as navigation.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rospider_gazebo import mission_check, nav_params


def _demo(pkg, arguments):
    """vision_demo.launch.py in a scope of its own: launch arguments leak between includes."""
    return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'vision_demo.launch.py')),
        launch_arguments=arguments.items())])


def launch_setup(context):
    pkg = get_package_share_directory('rospider_gazebo')
    merged = nav_params.merged_params_file(os.path.join(pkg, 'config', 'nav2_params.yaml'),
                                           os.path.join(pkg, 'config', 'nav_challenge_solved.yaml'))
    # The participant's own SLAM map, or the reference (navigation.launch.py resolves a name).
    map_value = (LaunchConfiguration('map').perform(context)
                 or os.path.join(pkg, 'maps', 'slam_challenge.yaml'))
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'navigation.launch.py')),
        launch_arguments={'world': 'mission_challenge',
                          'map': map_value,
                          'params_file': merged,
                          'gui': LaunchConfiguration('gui'),
                          'rviz': LaunchConfiguration('rviz')}.items())
    blocks = []
    for color in mission_check.BLOCK_OF_TAG.values():
        x, y, z = mission_check.block_spawn(color)
        blocks.append(Node(
            package='ros_gz_sim', executable='create', name=f'spawn_{color}', output='screen',
            arguments=['-file', os.path.join(pkg, 'models', f'pick_cube_{color}', 'model.sdf'),
                       '-name', f'pick_cube_{color}', '-x', str(x), '-y', str(y), '-z', str(z)]))
    sim_time = {'use_sim_time': True}
    detectors = [
        Node(package='rospider_gazebo', executable='color_detect.py', name='color_detect',
             output='screen',
             parameters=[os.path.join(pkg, 'config', 'color_detect.yaml'), sim_time]),
        Node(package='rospider_gazebo', executable='apriltag_detect.py', name='apriltag_detect',
             output='screen',
             parameters=[os.path.join(pkg, 'config', 'apriltag.yaml'), sim_time]),
        # The blocks weld to the hand the instant they spawn; pick_and_place's start-up
        # detaches must still be firing then, so it starts in the same timer, after them.
        Node(package='rospider_gazebo', executable='pick_and_place.py', name='pick_and_place',
             output='screen',
             parameters=[os.path.join(pkg, 'config', 'pick_place.yaml'),
                         dict(sim_time, auto_start=False)]),
    ]
    windows = [
        _demo(pkg, {'demo': 'apriltag_track', 'detector': 'none', 'start': 'false', 'show': 'false',
                    'target_tag': str(mission_check.TARGET_TAG),
                    'stop_distance': str(mission_check.STOP_DISTANCE),
                    'speed_limit': '0.05', 'turn_limit': '0.2'}),
        _demo(pkg, {'demo': 'track_and_grab', 'detector': 'none', 'walk': 'false',
                    'auto_place': 'false'}),
    ]
    return [navigation,
            TimerAction(period=5.0, actions=blocks + detectors),
            TimerAction(period=10.0, actions=windows)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='',
                              description='your own map from the SLAM exercise (default: the reference)'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
```

- [ ] **Step 2: Build and bring it up once**

Build, then in the test partition:
`ros2 launch rospider_gazebo mission_challenge.launch.py > scratchpad/mission/l1.log 2>&1 &`

After 45 s, check each of these:

- `grep -c "ERROR\|Traceback" l1.log`
  Expected: 0, or only errors that `nav_challenge.launch.py` also prints. Compare with
  `scratchpad/navlaunch_final_solved.log`.
- `gz model -m pick_cube_blue -p` and `gz model -m pick_cube_red -p`
  Expected: each at its `block_spawn()` position ±0.01, z 0.105. Not moved: the startup weld was
  released.
- `timeout 12 ros2 topic hz /controller/cmd_vel`
  Expected: nothing published (Review Focus 5).
- `gz stats -d 10`, and read the real-time factor.
  Expected: RTF ≥ 0.5. Below that, relaunch with `rviz:=false`, record both numbers, and keep
  `rviz` defaulting to true only if RTF with it is ≥ 0.5.

If a block has moved, the weld was not released: put the blocks in a `TimerAction(period=4.0)` of
their own, and keep `pick_and_place` at 5.0. Then repeat this step.

- [ ] **Step 3: Measure the station park**

Write `scratchpad/mission/park.py`. It:

- sets `/apriltag_track` `target_tag` to 3
- drives the robot in front of station 3 by sending Nav2 a goal at (3.2, -0.3, yaw 0)
- calls `/apriltag_track/set_running true` and waits until the odometry stays within 5 mm for 3 s
  (limit 60 s sim)
- calls it false
- calls `/track_and_grab/set_running false`, waits 2 s, calls `/track_and_grab/pick` `blue`, and
  waits up to 60 s for `/pick_and_place/state` == `CARRY`
- prints: the robot's pose, its distance from the block's spawn, CARRY yes/no, and
  `gz model -m pick_cube_blue -p`

Use `scratchpad/tag/watch.py` (odometry and stillness) and `scratchpad/grasp/explore.py` (state
subscription with TRANSIENT_LOCAL, `gz model`) as the pattern.

Run it after a fresh launch for `STOP_DISTANCE` 0.27, 0.24 and 0.30. Pass it through a scratch
copy of `mission_check.py` or a `stop_distance:=` edit of the launch's dict.

- Expected: at least one value lifts the blue block with the robot 0.25-0.32 m from it (the
  grasping exercise measured 0.29 m base-to-block for a clean pick).
- Choose the middle of the values that pick.
- If none picks, change `PEDESTAL_OFFSET` in 0.02 m steps (−0.035 … +0.035) at the best
  `STOP_DISTANCE` until one does. Regenerate the world when a station constant changes.

Accept when the chosen pair picks in 3 of 3 fresh launches. Write the pair into
`mission_check.py`. Record every run in the spec's "As built" section (Task 8 writes that section;
keep the numbers in `scratchpad/mission/measurements.md` until then).

- [ ] **Step 4: Measure the carried block and Nav2**

With the block held (continue from a step-3 run that ended in CARRY), send a Nav2 goal back to
(0.56, 0.55, yaw 180°) with `nav_client` (or with `ros2 action send_goal`, if Task 4 is not done
yet).

Expected: the goal succeeds and `/local_costmap/costmap` shows no obstacle attached to the robot
(look in RViz).

If the LiDAR marks the held block and the goal aborts, add `<visibility_flags>2</visibility_flags>`
to the `<visual>` of `models/pick_cube_red/model.sdf` and `models/pick_cube_blue/model.sdf`. The
LiDAR's `visibility_mask` already excludes bit 2 for the robot, see `gazebo.launch.py`. Then
measure again, and check the colour camera still sees the blocks: step 3 must still pick.

- [ ] **Step 5: Measure the live speed**

A fresh launch. With `rcl_interfaces/srv/SetParameters`:

- set `/controller_server` `FollowPath.desired_linear_vel`
- read `/velocity_smoother` `max_velocity` with `GetParameters`, replace index 0, and set it back

Do this at 0.10, drive from the spawn to (-0.4, -1.3) (the Nav2 exercise's G1, 1.4 m away in
room A), and record the mean odometry speed over the goal. Repeat at 0.20 from a fresh launch.

Expected: the two mean speeds differ by at least 0.06 m/s, so the change takes effect. If
`velocity_smoother` refuses the set (result `successful: false`), record its reason. The runner
then sets only `desired_linear_vel`, and the spec's "As built" says so.

- [ ] **Step 6: Unit suite still green**

Run: `python3 -m pytest -q test`
Expected: all pass.

---

### Task 4: nav_client: the Nav2 client shared by both checkers

**Files:**
- Create: `rospider_gazebo/nav_client.py`
- Modify: `scripts/check_nav.py`

**Interfaces:**
- Produces: `class Nav2Client(node)` with:
  - `sim_now() -> float`
  - `wait(future, wall_timeout)`
  - `ready() -> bool`
  - `active() -> bool`
  - `plan_to(x, y, yaw=0.0) -> 'ok' | 'no_path' | 'rejected'`
  - `drive(name, x, y, timeout, over_limit_after=inf, yaw=0.0) -> GoalResult`
  - `cancel()`
  - `yaw` is in radians.

This is a move, not a rewrite: `check_nav.py` must behave exactly as before.

- [ ] **Step 1: Move the class**

Create `rospider_gazebo/nav_client.py` holding what is now `check_nav.Checker`, except `main`:

- the imports it needs
- `SERVER_WAIT`, `ACTIVE_WAIT`, `NAV2_NODES`, `STATUS`
- every method, renamed to `Nav2Client`, taking an existing `node` instead of creating one

Two changes on the way:

```python
    def pose(self, x, y, yaw=0.0):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        return msg
```

Also thread `yaw` through `plan_to(x, y, yaw=0.0)` and
`drive(self, name, x, y, timeout, over_limit_after=math.inf, yaw=0.0)`.

Module docstring:

```python
"""Nav2's planner and navigator as scripts/check_nav.py and scripts/check_mission.py use them.

Plans each goal first (a blocked doorway is `no_path` at once instead of after Nav2's recoveries),
resends a goal whose response was lost (nav_check.send_with_retry), times on the sim clock with a
wall-clock backstop, and cancels the goal in flight on request (Ctrl+C). Needs a spinning-capable
rclpy node; not unit tested (it is a thin action client), verified by the two checkers' live runs.
"""
```

- [ ] **Step 2: Use it from check_nav.py**

In `check_nav.py`:

- keep `main()` and its text as they are
- replace the class with

```python
from rospider_gazebo.nav_client import Nav2Client
```

- in `main()`:

```python
    node = rclpy.create_node('check_nav', parameter_overrides=[Parameter('use_sim_time', value=True)])
    checker = Nav2Client(node)
```

- in `finally:`, use `node.destroy_node()`
- remove the imports `check_nav.py` no longer uses

- [ ] **Step 3: Verify check_nav unchanged**

Run: `python3 -m pytest -q test` → all pass.

Live, in the test partition:

- `ros2 launch rospider_gazebo nav_challenge.launch.py params:=<path to config/nav_challenge_solved.yaml>`
- after 40 s, `ros2 run rospider_gazebo check_nav.py`

Expected: `ผ่านครบทุกด่าน!`, total ~200-220 s, as the Nav2 spec records.

---

### Task 5: mission_plan: the participant's file → steps

**Files:**
- Create: `rospider_gazebo/mission_plan.py`
- Create: `test/test_mission_plan.py`
- Modify: `CMakeLists.txt` (register `test_mission_plan`)

**Interfaces:**
- Consumes: `challenge_params.ChallengeConfigError`, and
  `grasp_check.parse_point(text, source) -> (x, y, z)` (raises `ChallengeConfigError` naming
  'place_point').
- Produces:
  - `SPEED_RANGE = (0.05, 0.20)`, `COLORS = ('red', 'green', 'blue')`,
    `KINDS = ('go_to', 'go_to_tag', 'pick', 'place')`
  - `class Step(NamedTuple): kind: str; value: object`, where value is
    - `go_to` → `(x, y, yaw_rad)` floats
    - `go_to_tag` → `int`
    - `pick` → `str`
    - `place` → `(x, y, z)` floats
  - `parse(data, source) -> (speed: float, steps: list[Step])`
  - `load(path) -> (speed, steps)`
  - `describe(step) -> str`, a Thai one-liner

- [ ] **Step 1: Write the failing tests**

`test/test_mission_plan.py`:

```python
import math
import pathlib

import pytest
import yaml
from rospider_gazebo import mission_plan
from rospider_gazebo.challenge_params import ChallengeConfigError
from rospider_gazebo.mission_plan import Step

PKG = pathlib.Path(__file__).resolve().parents[1]
GOOD = {'speed': 0.15, 'steps': [{'go_to': [3.2, 0.35, 0]}, {'go_to_tag': 3}, {'pick': 'blue'},
                                 {'go_to': [0.56, 0.55, 180]}, {'place': '0.16 0.0 0.035'}]}


def _parse(data):
    return mission_plan.parse(data, 'm.yaml')


def test_a_good_file_becomes_typed_steps():
    speed, steps = _parse(GOOD)
    assert speed == 0.15
    assert steps[0] == Step('go_to', (3.2, 0.35, 0.0))
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


@pytest.mark.parametrize('step, number', [
    ({'goto': [1, 1, 0]}, 'goto'),                 # misspelled step
    ({'go_to': [1, 1]}, 'go_to'),                  # two numbers
    ({'go_to': ['a', 1, 0]}, 'go_to'),
    ({'go_to_tag': 'three'}, 'go_to_tag'),
    ({'go_to_tag': True}, 'go_to_tag'),
    ({'pick': 'yellow'}, 'pick'),
    ({'place': '0.16 0.0'}, 'place_point'),
    ({'go_to': [1, 1, 0], 'pick': 'red'}, 'ขั้นที่ 1'),   # two keys in one step
    ('pick blue', 'ขั้นที่ 1'),                     # not a mapping
])
def test_bad_steps_are_refused_with_the_step_number(step, number):
    with pytest.raises(ChallengeConfigError, match=number) as err:
        _parse({'speed': 0.15, 'steps': [step]})
    assert 'ขั้นที่ 1' in str(err.value)


def test_empty_steps_are_refused():
    with pytest.raises(ChallengeConfigError, match='steps'):
        _parse({'speed': 0.15, 'steps': []})


def test_describe_is_thai_and_names_the_values():
    speed, steps = _parse(GOOD)
    assert '3.20' in mission_plan.describe(steps[0]) and '180' in mission_plan.describe(steps[3])
    assert '3' in mission_plan.describe(steps[1])
    assert 'blue' in mission_plan.describe(steps[2])


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
```

- [ ] **Step 2: Run to verify they fail**

Run: `python3 -m pytest -q test/test_mission_plan.py`
Expected: FAIL at import (`cannot import name 'mission_plan'`).

- [ ] **Step 3: Write the module**

`rospider_gazebo/mission_plan.py`:

```python
"""The final mission's file (config/mission_challenge.yaml) -> a speed and a list of steps.

Refuses, with a Thai message naming the step's number, anything the runner could not execute:
unknown keys or steps, wrong types, a colour that does not exist, a place that is not three
numbers. It checks nothing about whether the plan makes sense -- that is the exercise.
Pure Python, unit tested.
"""

import math
from typing import NamedTuple

import yaml
from rospider_gazebo.challenge_params import ChallengeConfigError
from rospider_gazebo.grasp_check import parse_point

SPEED_RANGE = (0.05, 0.20)
COLORS = ('red', 'green', 'blue')
KINDS = ('go_to', 'go_to_tag', 'pick', 'place')


class Step(NamedTuple):
    kind: str
    value: object


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _step(index, item, source):
    where = f'{source} ขั้นที่ {index}'
    if not isinstance(item, dict) or len(item) != 1:
        raise ChallengeConfigError(
            f'{where}: แต่ละขั้นต้องเป็นคำสั่งเดียว เช่น "- pick: blue" (ได้ {item!r})')
    (kind, value), = item.items()
    if kind not in KINDS:
        raise ChallengeConfigError(f"{where}: ไม่รู้จักคำสั่ง '{kind}' (มี {', '.join(KINDS)})")
    if kind == 'go_to':
        if not (isinstance(value, list) and len(value) == 3 and all(map(_number, value))):
            raise ChallengeConfigError(f"{where}: 'go_to' ต้องเป็น [x, y, องศาที่หัน] เช่น [3.2, 0.35, 0]")
        x, y, yaw = (float(v) for v in value)
        return Step(kind, (x, y, math.radians(yaw)))
    if kind == 'go_to_tag':
        if not (isinstance(value, int) and not isinstance(value, bool) and value >= 0):
            raise ChallengeConfigError(f"{where}: 'go_to_tag' ต้องเป็นหมายเลขป้าย เช่น 3")
        return Step(kind, value)
    if kind == 'pick':
        if value not in COLORS:
            raise ChallengeConfigError(f"{where}: 'pick' ต้องเป็นสี {', '.join(COLORS)} (ได้ {value!r})")
        return Step(kind, value)
    if not isinstance(value, str):
        raise ChallengeConfigError(f"{where}: 'place_point' ต้องเป็นข้อความในเครื่องหมายคำพูด เช่น '0.16 0.0 0.035'")
    return Step(kind, parse_point(value, where))


def parse(data, source):
    """(speed, steps) from the file's mapping."""
    if not isinstance(data, dict):
        raise ChallengeConfigError(f'{source}: ต้องมี speed: และ steps: เหมือนไฟล์ตั้งต้น')
    unknown = [key for key in data if key not in ('speed', 'steps')]
    if unknown:
        raise ChallengeConfigError(f"{source}: ไม่รู้จัก '{unknown[0]}' (มีแค่ speed และ steps)")
    speed = data.get('speed')
    if not _number(speed) or not SPEED_RANGE[0] <= speed <= SPEED_RANGE[1]:
        raise ChallengeConfigError(
            f"{source}: 'speed' ต้องเป็นตัวเลข {SPEED_RANGE[0]}-{SPEED_RANGE[1]} (เมตร/วินาที) ได้ {speed!r}")
    steps = data.get('steps')
    if not isinstance(steps, list) or not steps:
        raise ChallengeConfigError(f"{source}: 'steps' ต้องเป็นรายการคำสั่งอย่างน้อยหนึ่งขั้น")
    return float(speed), [_step(i, item, source) for i, item in enumerate(steps, 1)]


def load(path):
    """(speed, steps) from the file at `path`."""
    try:
        with open(path) as handle:
            data = yaml.safe_load(handle)
    except yaml.YAMLError as err:
        mark = getattr(err, 'problem_mark', None)
        where = f' บรรทัด {mark.line + 1}' if mark is not None else ''
        raise ChallengeConfigError(f'{path}{where}: รูปแบบ YAML ผิด') from err
    except OSError as err:
        raise ChallengeConfigError(f'{path}: เปิดไฟล์ไม่ได้ ({err})') from err
    return parse(data, path)


def describe(step):
    """The step as one Thai line, for the runner's progress output."""
    if step.kind == 'go_to':
        x, y, yaw = step.value
        return f'go_to เดินไป ({x:.2f}, {y:.2f}) หันไป {math.degrees(yaw):.0f} องศา'
    if step.kind == 'go_to_tag':
        return f'go_to_tag เดินไปหยุดหน้าป้ายหมายเลข {step.value}'
    if step.kind == 'pick':
        return f'pick หยิบบล็อกสี {step.value}'
    x, y, z = step.value
    return f'place วางที่ ({x:.2f}, {y:.2f}, {z:.3f}) นับจากตัวหุ่น'
```

- [ ] **Step 4: Run to verify they pass**

Run: `python3 -m pytest -q test/test_mission_plan.py`
Expected: all pass. The two shipped-file cases are skipped until Task 7.

- [ ] **Step 5: Register the test**

Add `test_mission_plan` to `CMakeLists.txt`, next to `test_mission_world`, in the same form.

---

### Task 6: mission_check: facts → levels and the Thai report

**Files:**
- Modify: `rospider_gazebo/mission_check.py` (add `evaluate`, `format_report`)
- Modify: `rospider_gazebo/grasp_check.py` (`_side` → `side`, used by both)
- Create: `test/test_mission_check.py`
- Modify: `CMakeLists.txt` (register `test_mission_check`)

**Interfaces:**
- Consumes: the constants from Task 2; `grasp_check.side(robot, point) -> str`;
  `slam_check.Level`.
- Produces: `evaluate(facts) -> [Level, Level, Level]` and
  `format_report(levels, facts) -> str`, where `facts` is a dict:
  - `in_room_b: bool`
  - `robot: (x, y, yaw)` at the end
  - `parked_tag: int | None`: the last `go_to_tag` that parked
  - `lifted: list[str]`: colours lifted by `pick` steps
  - `blocks: {'red': (x, y, z) | None, 'blue': ...}` at the end
  - `failure: str | None`: the Thai reason the run stopped early
  - `failed_step: int | None`: the 1-based number of that step

- [ ] **Step 1: Make grasp_check's side public**

In `grasp_check.py`, rename `def _side(` to `def side(` and its two calls in `format_report`. Run
`python3 -m pytest -q test/test_grasp_check.py`. Expected: 10 passed.

- [ ] **Step 2: Write the failing tests**

`test/test_mission_check.py`:

```python
import math

from rospider_gazebo import mission_check as mc

FORBIDDEN = ('speed', 'go_to', 'go_to_tag', 'pick', 'place', 'steps')
PAD_ROBOT = (0.56, 0.55, math.pi)          # the answer key's return pose, facing the pad


def _facts(**over):
    facts = {'in_room_b': True, 'robot': PAD_ROBOT, 'parked_tag': 3, 'lifted': ['blue'],
             'blocks': {'red': mc.block_spawn('red'), 'blue': (0.41, 0.56, 0.025)},
             'failure': None, 'failed_step': None}
    facts.update(over)
    return facts


def _run(**over):
    facts = _facts(**over)
    levels = mc.evaluate(facts)
    return [level.status for level in levels], mc.format_report(levels, facts)


def test_answer_key_passes():
    statuses, text = _run()
    assert statuses == ['pass'] * 3 and 'ผ่านครบทุกด่าน' in text


def test_never_in_room_b_fails_level_one_and_says_where_it_ended():
    statuses, text = _run(in_room_b=False, robot=(0.6, 1.5, 0.0), parked_tag=None, lifted=[],
                          blocks={'red': mc.block_spawn('red'), 'blue': mc.block_spawn('blue')},
                          failure='มองไม่เห็นป้ายหมายเลข 2', failed_step=2)
    assert statuses == ['fail', 'skip', 'skip']
    assert '0.60' in text and 'ห้อง B' in text and 'ขั้นที่ 2' in text


def test_parking_at_the_red_station_fails_level_two_and_names_the_tag():
    statuses, text = _run(parked_tag=2, lifted=[],
                          blocks={'red': mc.block_spawn('red'), 'blue': mc.block_spawn('blue')},
                          failure='หยิบบล็อกสี blue ไม่ได้', failed_step=3)
    assert statuses == ['pass', 'fail', 'skip']
    assert 'หมายเลข 2' in text and 'แดง' in text


def test_lifting_red_fails_level_two_and_says_so():
    statuses, text = _run(parked_tag=2, lifted=['red'])
    assert statuses == ['pass', 'fail', 'skip'] and 'สีแดง' in text


def test_blue_put_down_at_the_spawn_fails_level_three_with_distance_and_side():
    statuses, text = _run(robot=(0.0, 0.0, 0.0), blocks={'red': mc.block_spawn('red'),
                                                         'blue': (0.16, 0.0, 0.025)})
    assert statuses == ['pass', 'pass', 'fail']
    assert '0.60' in text                       # hypot(0.40 - 0.16, 0.55 - 0.0)


def test_blue_still_held_is_not_on_the_pad():
    statuses, _text = _run(blocks={'red': mc.block_spawn('red'), 'blue': (0.40, 0.55, 0.22)})
    assert statuses[2] == 'fail'


def test_unreadable_block_pose_fails_level_three_plainly():
    statuses, text = _run(blocks={'red': None, 'blue': None})
    assert statuses[2] == 'fail' and 'Gazebo' in text


def test_hints_name_no_key():
    cases = [dict(in_room_b=False, lifted=[], parked_tag=None),
             dict(parked_tag=2, lifted=[]), dict(parked_tag=2, lifted=['red']),
             dict(blocks={'red': None, 'blue': (0.16, 0.0, 0.025)})]
    for over in cases:
        text = _run(**over)[1]
        hints = [line for line in text.splitlines() if 'คำใบ้' in line]
        assert not any(key in line for key in FORBIDDEN for line in hints)
```

- [ ] **Step 3: Run to verify they fail**

Run: `python3 -m pytest -q test/test_mission_check.py`
Expected: FAIL with `AttributeError: module 'rospider_gazebo.mission_check' has no attribute 'evaluate'`.

- [ ] **Step 4: Write evaluate and format_report**

Append to `mission_check.py`, and add the imports at the top:
`from rospider_gazebo.grasp_check import side` and `from rospider_gazebo.slam_check import Level`.

```python
COLOUR_THAI = {'red': 'แดง', 'green': 'เขียว', 'blue': 'น้ำเงิน'}
_TITLES = {1: 'ไปถึงห้อง B', 2: 'ถือบล็อกสีน้ำเงิน', 3: 'บล็อกสีน้ำเงินอยู่บนแผ่นเหลือง'}


def evaluate(facts):
    """The three levels for the facts one run recorded."""
    if not facts['in_room_b']:
        return [Level(1, 'fail', facts), Level(2, 'skip', {'after': 1}), Level(3, 'skip', {'after': 1})]
    if TARGET_COLOR not in facts['lifted']:
        return [Level(1, 'pass', facts), Level(2, 'fail', facts), Level(3, 'skip', {'after': 2})]
    block = facts['blocks'].get(TARGET_COLOR)
    on_pad = (block is not None and block[2] < FLOOR_Z
              and math.dist(block[:2], PAD) <= PAD_TOL)
    return [Level(1, 'pass', facts), Level(2, 'pass', facts),
            Level(3, 'pass' if on_pad else 'fail', facts)]


def format_report(levels, facts):
    """Thai text: the step that stopped the run (if one did), then the levels with hints."""
    lines = []
    if facts['failure']:
        lines.append(f'ภารกิจหยุดที่ขั้นที่ {facts["failed_step"]}: {facts["failure"]}')
    for level in levels:
        head = f'ด่าน {level.number} {_TITLES[level.number]}:'
        if level.status == 'skip':
            lines.append(f'{head} ยังไม่ตรวจ (ผ่านด่าน {level.numbers["after"]} ก่อน)')
        elif level.status == 'pass':
            lines.append(f'{head} ผ่าน')
        else:
            lines.extend(_failure(level.number, head, facts))
    if all(level.status == 'pass' for level in levels):
        lines.append('ผ่านครบทุกด่าน!')
    return '\n'.join(lines)


def _failure(number, head, facts):
    x, y, _ = facts['robot']
    if number == 1:
        return [f'{head} ไม่ผ่าน - หุ่นไม่เคยเข้าห้อง B (สุดท้ายอยู่ที่ ({x:.2f}, {y:.2f}))',
                '  คำใบ้: ห้อง B อยู่ทางไหนของแผนที่? ดูพิกัดใน RViz แล้วเทียบกับจุดหมายแรก']
    if number == 2:
        others = [c for c in facts['lifted'] if c != TARGET_COLOR]
        if others:
            text = f'หุ่นหยิบบล็อกสี{COLOUR_THAI[others[-1]]}'
        elif facts['parked_tag'] is not None:
            colour = BLOCK_OF_TAG.get(facts['parked_tag'])
            there = f' ซึ่งมีบล็อกสี{COLOUR_THAI[colour]}' if colour else ''
            text = f'หุ่นไปหยุดหน้าป้ายหมายเลข {facts["parked_tag"]}{there} แล้วหยิบบล็อกสีน้ำเงินไม่ได้'
        else:
            text = 'หุ่นยังไม่ได้ไปหยุดหน้าป้ายไหนเลย จึงไม่มีบล็อกให้หยิบในระยะแขน'
        return [f'{head} ไม่ผ่าน - {text}',
                '  คำใบ้: บล็อกสีน้ำเงินอยู่บนแท่นหน้าป้ายหมายเลขไหน? หุ่นหยิบได้แค่ของที่อยู่ตรงหน้า']
    block = facts['blocks'].get(TARGET_COLOR)
    if block is None:
        return [f'{head} ไม่ผ่าน - อ่านตำแหน่งบล็อกจาก Gazebo ไม่ได้']
    if block[2] >= FLOOR_Z:
        return [f'{head} ไม่ผ่าน - บล็อกยังไม่ได้วางลงพื้น (สูง {block[2]:.2f} ม.)',
                '  คำใบ้: หลังเดินกลับแล้ว ได้สั่งวางหรือยัง']
    gap = math.dist(block[:2], PAD)
    return [f'{head} ไม่ผ่าน - บล็อกห่างกลางแผ่น {gap:.2f} ม. (ต้องไม่เกิน {PAD_TOL:.2f}); '
            f'แผ่นอยู่{side(facts["robot"], PAD)}ของหุ่น บล็อกไปอยู่{side(facts["robot"], block)}',
            '  คำใบ้: จุดวางนับจากตัวหุ่น - ตอนวางหุ่นยืนตรงไหนและหันไปทางไหน']
```

- [ ] **Step 5: Run to verify they pass**

Run: `python3 -m pytest -q test/test_mission_check.py test/test_mission_world.py test/test_grasp_check.py`
Expected: all pass.

- [ ] **Step 6: Register the test**

Add `test_mission_check` to `CMakeLists.txt` in the same form.

---

### Task 7: check_mission.py, the draft and the answer key, live verification

**Files:**
- Create: `scripts/check_mission.py` (executable)
- Create: `config/mission_challenge.yaml`, `config/mission_challenge_solved.yaml`
- Modify: `CMakeLists.txt` (install `scripts/check_mission.py`)

**Interfaces:**
- Consumes:
  - `mission_plan.load/describe/Step` (Task 5)
  - `mission_check.*` (Tasks 2, 6)
  - `nav_client.Nav2Client` (Task 4)
  - `grasp_check.parse_gz_pose`
  - services: `/apriltag_track/set_running` (SetBool), `/apriltag_track/set_parameters`
    (SetParameters), `/track_and_grab/{set_walk,set_auto_place,set_running}` (SetBool),
    `/track_and_grab/pick` (SetString), `/pick_and_place/place` (SetString)
  - topics: `/pick_and_place/state` (latched String), `/apriltag_detect/apriltag_info`
    (ApriltagsInfo), `/odom`
  - Nav2 params: `/controller_server` `FollowPath.desired_linear_vel`, `/velocity_smoother`
    `max_velocity` (Task 3 step 5 decides whether the second is set)

- [ ] **Step 1: Write the files the runner reads**

`config/mission_challenge.yaml`:

```yaml
# ภารกิจปิดท้าย: บล็อกสีน้ำเงินวางอยู่บนแท่นหน้าป้าย AprilTag หมายเลข 3 ในห้อง B
# ให้หุ่นไปหยิบมา แล้ววางบนแผ่นสีเหลืองในห้อง A (กลางแผ่นอยู่ที่ x = 0.40, y = 0.55 บนแผนที่)
# ในห้อง B มีแท่นอีกแท่น (ป้ายหมายเลข 2) ที่มีบล็อกสีแดง อย่าหยิบผิด
#
# ไฟล์นี้เพื่อนร่างไว้ รันได้แต่ยังมีจุดผิด - แก้แล้วตรวจใหม่ได้เลย (ตัวตรวจอ่านไฟล์นี้ทุกครั้ง)
# ตรวจ: ros2 run rospider_gazebo check_mission.py
# ก่อนตรวจรอบใหม่ ปิด-เปิด launch ใหม่ หุ่นและบล็อกจะได้กลับที่เดิม
# อยากเริ่มใหม่: git checkout -- ไฟล์นี้
#
# คำสั่งที่ใช้ได้:
#   go_to: [x, y, องศา]   Nav2 เดินไปจุด (x, y) บนแผนที่ แล้วหันไปทางนั้น (0 = +x, 90 = +y)
#   go_to_tag: N          เดินตามป้าย AprilTag หมายเลข N ไปหยุดตรงหน้า (ระยะที่แขนหยิบของบนแท่นได้)
#   pick: สี              หยิบบล็อกสีนั้น (red, green, blue) ที่อยู่ตรงหน้า แล้วถือไว้
#   place: 'x y z'        วางบล็อกที่ถืออยู่ นับจากตัวหุ่น: x ไปข้างหน้า, y ไปทางซ้าย, z ขึ้นบน (เมตร)

# ความเร็วเดินตอน go_to (เมตร/วินาที, 0.05-0.20)
speed: 0.15
steps:
  - go_to: [0.6, 1.5, 0]
  - go_to_tag: 2
  - pick: blue
  - go_to: [0.0, 0.0, 0]
  - place: '0.16 0.0 0.035'
```

`config/mission_challenge_solved.yaml`:

```yaml
# เฉลยภารกิจปิดท้าย (สำหรับวิทยากร): cp ไฟล์นี้ทับ mission_challenge.yaml แล้วตรวจ
speed: 0.15
steps:
  - go_to: [3.2, 0.35, 0]
  - go_to_tag: 3
  - pick: blue
  - go_to: [0.56, 0.55, 180]
  - place: '0.16 0.0 0.035'
```

The room-B waypoint (3.2, 0.35) and the return pose come from Task 3's measurements. If Task 3
moved a station, recompute:

- waypoint: 0.55 m in front of the midpoint of the two stations
- return: `PAD` + 0.16 m toward the doorway, facing the pad (180°)

- [ ] **Step 2: Write the runner**

`scripts/check_mission.py`:

```python
#!/usr/bin/env python3
"""ตรวจภารกิจปิดท้าย: รันไฟล์ภารกิจทีละขั้น แล้วบอกผลทีละด่าน

    ros2 run rospider_gazebo check_mission.py [path/to/mission.yaml]

Reads config/mission_challenge.yaml (or the given path) afresh at every run, checks the stack is up
and the robot and blocks are where a fresh launch puts them, then runs the steps in order through
the nodes mission_challenge.launch.py starts (rospider_gazebo/mission_plan.py describes the step
types). It stops at the first failed step, switches everything it switched on back off (on
Ctrl+C too), reads where the blocks came to rest, and scores the run with
rospider_gazebo/mission_check.py. Times are on the sim clock with wall-clock backstops.
"""

import math
import os
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from interfaces.msg import ApriltagsInfo
from interfaces.srv import SetString
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rospider_gazebo import grasp_check, mission_check, mission_plan
from rospider_gazebo.challenge_params import ChallengeConfigError
from rospider_gazebo.nav_client import Nav2Client
from std_msgs.msg import String
from std_srvs.srv import SetBool

GO_TO_LIMIT = 240.0
TAG_LIMIT = 60.0
TAG_UNSEEN = 20.0         # no sight of the tag this long after switching on: give up
STILL_S = 3.0
PICK_LIMIT = 60.0
PLACE_LIMIT = 40.0
SETTLE = 2.0
WALL_FACTOR = 5.0         # a sim-time limit becomes this many wall seconds at most
FREE = ('IDLE', 'DONE')
STATUS_THAI = {'no_path': 'Nav2 หาทางไปไม่ได้ (จุดหมายอยู่ในผนังหรือนอกแผนที่?)',
               'aborted': 'Nav2 ไปไม่ถึงแล้วยอมแพ้', 'timeout': 'เดินไม่ถึงภายในเวลา',
               'rejected': 'Nav2 ไม่รับเป้าหมาย', 'canceled': 'ถูกยกเลิก'}


class StepFailed(Exception):
    """A step could not finish; the message is the Thai reason."""


def default_file():
    share = get_package_share_directory('rospider_gazebo')
    return os.path.realpath(os.path.join(share, 'config', 'mission_challenge.yaml'))


def gz_pose(model):
    try:
        out = subprocess.run(['gz', 'model', '-m', model, '-p'], capture_output=True, text=True,
                             timeout=30).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None
    return grasp_check.parse_gz_pose(out)


class Mission:
    def __init__(self):
        self.node = rclpy.create_node('check_mission',
                                      parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.nav = Nav2Client(self.node)
        n = self.node
        self.tag_run = n.create_client(SetBool, '/apriltag_track/set_running')
        self.tag_params = n.create_client(SetParameters, '/apriltag_track/set_parameters')
        self.grab_walk = n.create_client(SetBool, '/track_and_grab/set_walk')
        self.grab_auto = n.create_client(SetBool, '/track_and_grab/set_auto_place')
        self.grab_run = n.create_client(SetBool, '/track_and_grab/set_running')
        self.grab_pick = n.create_client(SetString, '/track_and_grab/pick')
        self.place_srv = n.create_client(SetString, '/pick_and_place/place')
        self.ctrl_set = n.create_client(SetParameters, '/controller_server/set_parameters')
        self.smooth_get = n.create_client(GetParameters, '/velocity_smoother/get_parameters')
        self.smooth_set = n.create_client(SetParameters, '/velocity_smoother/set_parameters')
        self.map_get = n.create_client(GetParameters, '/map_server/get_parameters')
        self.own_map = False
        self.odom = None
        self.odom_wall = None
        self.state = None
        self.tags_seen = {}         # tag id -> sim time last seen
        self.facts = {'in_room_b': False, 'robot': (0.0, 0.0, 0.0), 'parked_tag': None,
                      'lifted': [], 'blocks': {}, 'failure': None, 'failed_step': None}
        n.create_subscription(Odometry, '/odom', self._on_odom, 10)
        n.create_subscription(String, '/pick_and_place/state', self._on_state,
                              QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        n.create_subscription(ApriltagsInfo, '/apriltag_detect/apriltag_info', self._on_tags, 1)
        self.switched_on = set()

    # ---------------------------------------------------------------- plumbing

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.odom_wall = time.monotonic()
        self.facts['robot'] = self.odom
        if self.odom[0] > mission_check.ROOM_B_X:
            self.facts['in_room_b'] = True

    def _on_state(self, msg):
        self.state = msg.data

    def _on_tags(self, msg):
        now = self.nav.sim_now()
        for tag in msg.data:
            self.tags_seen[tag.id] = now

    def spin(self, seconds):
        """Spin for `seconds` of sim time (wall backstop)."""
        start, wall_end = self.nav.sim_now(), time.monotonic() + seconds * WALL_FACTOR + 5
        while self.nav.sim_now() - start < seconds and time.monotonic() < wall_end:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def call(self, client, request, timeout=5.0, tries=3):
        for _ in range(tries):
            if not client.wait_for_service(timeout_sec=2.0):
                continue
            answer = self.nav.wait(client.call_async(request), timeout)
            if answer is not None:
                return answer
        return None

    def set_bool(self, client, on):
        request = SetBool.Request()
        request.data = on
        return self.call(client, request)

    def set_param(self, client, name, value):
        param = ParameterMsg(name=name)
        if isinstance(value, int):
            param.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
        elif isinstance(value, float):
            param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
        else:
            param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                         double_array_value=list(value))
        answer = self.call(client, SetParameters.Request(parameters=[param]))
        return answer is not None and all(r.successful for r in answer.results)

    # ---------------------------------------------------------------- set-up

    def ready(self):
        if not (self.nav.ready() and self.nav.active()):
            return 'ไม่พบ Nav2 - เปิด ros2 launch rospider_gazebo mission_challenge.launch.py แล้วรอให้ RViz ขึ้นแผนที่'
        for client in (self.tag_run, self.grab_pick, self.place_srv):
            if not client.wait_for_service(timeout_sec=60.0):
                return f'ไม่พบ {client.srv_name} - รอให้หน้าต่าง track_and_grab ขึ้นก่อนแล้วตรวจใหม่'
        end = time.monotonic() + 30.0
        while (self.odom is None or self.state is None) and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.odom is None or self.state is None:
            return 'ไม่ได้รับ /odom หรือสถานะของแขน - ปิด-เปิด launch ใหม่'
        answer = self.call(self.map_get, GetParameters.Request(names=['yaml_filename']))
        map_file = answer.values[0].string_value if answer is not None and answer.values else '?'
        self.own_map = os.path.basename(map_file) != 'slam_challenge.yaml'
        print(f'แผนที่: {map_file}', flush=True)
        if math.hypot(*self.odom[:2]) > 0.1:
            return 'หุ่นไม่ได้อยู่ที่จุดเริ่ม - ปิด-เปิด launch ใหม่ก่อนตรวจ'
        for color in mission_check.BLOCK_OF_TAG.values():
            pose = gz_pose(f'pick_cube_{color}')
            if pose is None or math.dist(pose, mission_check.block_spawn(color)) > 0.02:
                return f'บล็อกสี {color} ไม่ได้อยู่บนแท่น - ปิด-เปิด launch ใหม่ก่อนตรวจ'
        return None

    def set_speed(self, speed):
        self.set_param(self.ctrl_set, 'FollowPath.desired_linear_vel', speed)
        answer = self.call(self.smooth_get, GetParameters.Request(names=['max_velocity']))
        if answer is not None and answer.values:
            limits = list(answer.values[0].double_array_value)
            limits[0] = speed
            self.set_param(self.smooth_set, 'max_velocity', limits)

    # ---------------------------------------------------------------- steps

    def go_to(self, value, speed):
        x, y, yaw = value
        self.set_speed(speed)
        self.switched_on.add('nav')
        result = self.nav.drive('go_to', x, y, GO_TO_LIMIT, yaw=yaw)
        self.switched_on.discard('nav')
        if result.status != 'succeeded':
            reason = STATUS_THAI.get(result.status, result.status)
            if result.status == 'no_path' and self.own_map:
                reason += ' - ใช้แผนที่ของตัวเองอยู่: แผนที่มีห้อง B และทางเดินครบไหม'
            raise StepFailed(reason)

    def go_to_tag(self, tag):
        if not self.set_param(self.tag_params, 'target_tag', tag):
            raise StepFailed('ตั้งหมายเลขป้ายให้ apriltag_track ไม่ได้')
        if self.set_bool(self.tag_run, True) is None:
            raise StepFailed('apriltag_track ไม่ตอบคำสั่ง')
        self.switched_on.add('tag')
        start = self.nav.sim_now()
        wall_end = time.monotonic() + TAG_LIMIT * WALL_FACTOR
        anchor, still_since = self.odom, start
        try:
            while True:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                now = self.nav.sim_now()
                if math.dist(self.odom[:2], anchor[:2]) > 0.005 or \
                        abs(math.remainder(self.odom[2] - anchor[2], math.tau)) > 0.02:
                    anchor, still_since = self.odom, now
                seen = self.tags_seen.get(tag)
                if seen is None and now - start > TAG_UNSEEN:
                    raise StepFailed(f'มองไม่เห็นป้ายหมายเลข {tag}')
                if seen is not None and now - seen < 1.0 and now - still_since > STILL_S:
                    self.facts['parked_tag'] = tag
                    return
                if now - start > TAG_LIMIT or time.monotonic() > wall_end:
                    raise StepFailed(f'ไปไม่ถึงหน้าป้ายหมายเลข {tag} ภายในเวลา')
        finally:
            self.set_bool(self.tag_run, False)
            self.switched_on.discard('tag')

    def pick(self, color):
        if self.state not in FREE:
            raise StepFailed('แขนยังไม่ว่าง (ถือบล็อกอยู่หรือเปล่า?)')
        self.set_bool(self.grab_walk, False)
        self.set_bool(self.grab_auto, False)
        self.set_bool(self.grab_run, False)       # camera down to the look pose
        self.spin(2.0)
        answer = self.call(self.grab_pick, SetString.Request(data=color))
        if answer is None or not answer.success:
            raise StepFailed(f'track_and_grab ไม่รับคำสั่งหยิบ ({getattr(answer, "message", "ไม่ตอบ")})')
        self.switched_on.add('grab')
        start, wall_end = self.nav.sim_now(), time.monotonic() + PICK_LIMIT * WALL_FACTOR
        while self.state != 'CARRY':
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.nav.sim_now() - start > PICK_LIMIT or time.monotonic() > wall_end:
                self.set_bool(self.grab_run, False)
                self.switched_on.discard('grab')
                raise StepFailed(f'หยิบบล็อกสี {color} ไม่ได้ (ไม่อยู่ในระยะแขน หรือมองไม่เห็น)')
        self.switched_on.discard('grab')
        for c in mission_check.BLOCK_OF_TAG.values():
            pose = gz_pose(f'pick_cube_{c}')
            if pose is not None and pose[2] > mission_check.LIFTED_Z:
                self.facts['lifted'].append(c)

    def place(self, point):
        if self.state != 'CARRY':
            raise StepFailed('ไม่ได้ถือบล็อกอยู่ จึงไม่มีอะไรให้วาง')
        answer = self.call(self.place_srv, SetString.Request(data=' '.join(f'{v:g}' for v in point)))
        if answer is None or not answer.success:
            raise StepFailed(f'แขนไม่รับคำสั่งวาง ({getattr(answer, "message", "ไม่ตอบ")})')
        start, wall_end = self.nav.sim_now(), time.monotonic() + PLACE_LIMIT * WALL_FACTOR
        seen_busy = False
        while not (seen_busy and self.state in FREE):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            seen_busy = seen_busy or self.state not in FREE + ('CARRY',)
            if self.nav.sim_now() - start > PLACE_LIMIT or time.monotonic() > wall_end:
                raise StepFailed('วางไม่เสร็จภายในเวลา (จุดวางอยู่นอกระยะแขนหรือเปล่า?)')

    # ---------------------------------------------------------------- run

    def run(self, speed, steps):
        for number, step in enumerate(steps, 1):
            print(f'ขั้น {number}/{len(steps)} {mission_plan.describe(step)} ...', flush=True)
            start = self.nav.sim_now()
            try:
                if step.kind == 'go_to':
                    self.go_to(step.value, speed)
                else:
                    getattr(self, step.kind)(step.value)
            except StepFailed as err:
                print(f'  -> ไม่สำเร็จ: {err}', flush=True)
                self.facts['failure'], self.facts['failed_step'] = str(err), number
                return
            print(f'  -> สำเร็จ ({self.nav.sim_now() - start:.0f} วินาที)', flush=True)

    def stop_all(self):
        """Switch off whatever is still on; safe to call twice."""
        if 'nav' in self.switched_on:
            self.nav.cancel()
        self.set_bool(self.tag_run, False)
        # pick_and_place is left to finish a motion it has started: stopping the arm mid-grasp
        # leaves the block welded at an odd angle.
        self.set_bool(self.grab_run, False)
        self.switched_on.clear()

    def finish(self):
        self.spin(SETTLE)
        self.facts['blocks'] = {c: gz_pose(f'pick_cube_{c}')
                                for c in mission_check.BLOCK_OF_TAG.values()}
        levels = mission_check.evaluate(self.facts)
        print(mission_check.format_report(levels, self.facts))
        return 0 if all(level.status == 'pass' for level in levels) else 1


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else default_file()
    try:
        speed, steps = mission_plan.load(path)
    except ChallengeConfigError as err:
        print(f'ไฟล์ภารกิจใช้ไม่ได้: {err}')
        return 1
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    mission = Mission()
    print(f'ตรวจภารกิจปิดท้ายจาก {path} (ความเร็ว {speed:g} ม./วินาที, {len(steps)} ขั้น)', flush=True)
    try:
        problem = mission.ready()
        if problem:
            print(problem)
            return 1
        mission.run(speed, steps)
        mission.stop_all()
        return mission.finish()
    except KeyboardInterrupt:
        try:
            mission.stop_all()
        except Exception:       # noqa: BLE001 -- shutting down anyway
            pass
        print('หยุดตรวจแล้ว (สั่งทุกอย่างหยุด)')
        return 130
    finally:
        mission.node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
```

If Task 3 step 5 found that `velocity_smoother` refuses the set, drop the three
`smooth_*`/`limits` lines from `set_speed`.

`chmod +x scripts/check_mission.py`. Add it to `install(PROGRAMS …)` in `CMakeLists.txt` after
`scripts/check_grasp.py`. Build.

- [ ] **Step 3: Unit suite, and the shipped files load**

Run: `python3 -m pytest -q test`
Expected: all pass, and the two shipped-file cases of `test_mission_plan` now run instead of
being skipped.

- [ ] **Step 4: The answer key, three times**

Create `scratchpad/mission/crun.sh`. It:

- takes `<name> <mission yaml>`
- starts `mission_challenge.launch.py` in the test partition, waits 60 s
- runs `ros2 run rospider_gazebo check_mission.py <yaml>` under a 900 s timeout, printing wall seconds
- stops the launch by partition

Model it on `scratchpad/grasp/crun.sh`, passing the mission file as the argument instead of
`params:=`.

Run it with the solved file three times, sequentially.

Expected each time: every step `สำเร็จ`, then `ผ่านครบทุกด่าน!`, in under 5 minutes of wall
time.

If the place misses the pad, first compare the measured block rest with `PAD`:

- a constant offset: correct the answer key's return pose, not `PAD_TOL`
- a spread wider than `PAD_TOL`: record it, and ask the user before widening the tolerance

- [ ] **Step 5: The draft, then each fix in order**

Three more files in the scratchpad: draft + room-B waypoint fixed; + `go_to_tag: 3`; + return
pose fixed (= the answer key). Run the draft and the first two.

Expected:

| File | Result |
|---|---|
| draft | level 1 fails; the run stops at step 2 with `มองไม่เห็นป้ายหมายเลข 2` |
| + waypoint | level 2 fails, naming tag 2 and the red block |
| + tag | level 3 fails with the block ~0.6 m from the pad |

Record each result.

- [ ] **Step 6: Ctrl+C, the speed knob, and an own map**

- A fresh launch, the answer key. Press Ctrl+C (send `kill -INT` to the checker) during step 1.
  Expected:
  - `หยุดตรวจแล้ว`
  - the robot stops within 2 s (odometry)
  - `timeout 12 ros2 topic hz /controller/cmd_vel` then shows nothing
- The answer key with `speed: 0.20`. Expected: passes. Record the wall time next to the 0.15 runs.
- An own map: `mission_challenge.launch.py map:=<scratchpad>/ref.yaml`. `scratchpad/ref.*` is a map
  recorded with the SLAM exercise's answer key during its measurements; if it is missing, record
  one with `slam_challenge.launch.py params:=<solved>` and `map_saver_cli`. Then the answer key.
  Expected: `แผนที่: …/ref.yaml`, then passes.

---

### Task 8: Docs, final suite, and hand-back

**Files:**
- Modify: `ROSpider/SIMULATION.md` (a `## ภารกิจปิดท้าย` section before `## ปัญหาที่พบบ่อย`)
- Modify: `CLAUDE.md` (a bullet after the 3D Shape Recognition bullet)
- Modify: `docs/superpowers/specs/2026-09-29-mission-challenge-design.md` (an "As built" section
  with every measurement from Tasks 3 and 7, and any decision taken on the way)

- [ ] **Step 1: SIMULATION.md**

Insert before `## ปัญหาที่พบบ่อย`:

````markdown
## ภารกิจปิดท้าย (30 นาที)

รวมหลายหัวข้อไว้ในภารกิจเดียว: Nav2 พาข้ามไปห้อง B, เดินตามป้าย AprilTag ไปหยุดหน้าแท่นที่ถูก, หยิบบล็อกด้วยกล้องความลึก แล้วนำกลับมาวางบนแผ่นสีเหลืองในห้อง A ค่าจูนของแต่ละระบบใช้ค่าเฉลยให้แล้ว สิ่งที่ต้องทำคือเขียนลำดับขั้นในไฟล์ภารกิจให้ถูก (ไฟล์ที่ให้มาเพื่อนร่างไว้ มีจุดผิด)

```bash
ros2 launch rospider_gazebo mission_challenge.launch.py   # ฉาก + Nav2 + RViz + หน้าต่าง track_and_grab
ros2 run rospider_gazebo check_mission.py                 # รันไฟล์ภารกิจทีละขั้นแล้วบอกผลทีละด่าน
```

แก้ `src/simulations/rospider_gazebo/config/mission_challenge.yaml` (อ่านโจทย์และคำสั่งที่ใช้ได้ที่หัวไฟล์) แล้วตรวจได้เลย ผ่านแล้วอยากท้าทายขึ้น: ใช้แผนที่ของตัวเองจากโจทย์ SLAM ด้วย `mission_challenge.launch.py map:=<ชื่อแผนที่>` ก่อนตรวจรอบใหม่ให้ปิด-เปิด launch ใหม่ หุ่นและบล็อกจะได้กลับที่เดิม ดูพิกัดบนแผนที่ได้จาก RViz (เลื่อนเมาส์บนแผนที่ มุมล่างซ้ายบอก x, y) — ด่าน: 1 ไปถึงห้อง B · 2 ถือบล็อกสีน้ำเงิน · 3 บล็อกสีน้ำเงินอยู่บนแผ่นเหลือง
````

Before writing the RViz hint, check it in RViz 2 Jazzy: does the status bar show the cursor's map
coordinates? If it does not, say instead: `ใช้ปุ่ม Publish Point แล้วดูพิกัดใน terminal ที่รัน
ros2 topic echo /clicked_point`.

- [ ] **Step 2: CLAUDE.md bullet**

One bullet, in the style of the other exercise bullets, covering:

- the scene constants' home (`mission_check.py`) and the generated world
- the launch's composition, and why blocks and `pick_and_place` share a timer
- `apriltag_track`'s quiet-when-off and live `target_tag`
- `nav_client.py` shared by `check_nav.py` and `check_mission.py`
- the step types
- the levels
- the draft's three mistakes and what each run measured
- `speed` set live on Nav2 (and whether `velocity_smoother` took it)
- the measured park (`STOP_DISTANCE`, `PEDESTAL_OFFSET`)
- whether the carried block needed `visibility_flags`
- the check's wall time

- [ ] **Step 3: Final suite**

Run: `colcon build --packages-select rospider_gazebo --symlink-install && colcon test --packages-select rospider_gazebo && colcon test-result --test-result-base build/rospider_gazebo`
Expected: `0 errors, 0 failures` (407 + the new tests).

Also confirm that no test sim is left in the `missionplan` partition.

- [ ] **Step 4: Hand back, without committing**

- Report to the user in Thai, briefly: what was built, the measured results, and any deviations
  (with their rulings).
- List the files to commit: the new files in the File map, and `models/tag_station_3/`,
  `worlds/textures/tag_3.png`. Leave out the user's played participant files, `.vscode/`,
  `ROSpider/maps/room1.*`, `tag_station_4` and `tag_4.png`.
- Ask whether to commit and push.
