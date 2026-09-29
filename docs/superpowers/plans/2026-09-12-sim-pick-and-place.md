# Sim Pick-and-Place Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the ROSpider Gazebo simulation detect three coloured cubes with its RGB-D camera and move them, one at a time, onto a drop marker using its 5-DOF arm.

**Architecture:** Three decoupled pieces. `color_detect.py` does 2D HSV detection and publishes `interfaces/ObjectsInfo` on `/yolo/object_detect`, the same topic and type Hiwonder's YOLO node uses. `arm_ik.py` is a pure-maths module holding exact forward kinematics and a closed-form inverse — MoveIt is not used, because `position_only_ik: true` makes it unable to control grasp orientation. `pick_and_place.py` is a state machine that projects detections to 3D, latches them in `base_link`, drives the arm through `arm_controller`/`gripper_controller`, and welds the cube to the hand with Gazebo's `DetachableJoint`.

**Tech Stack:** ROS 2 Jazzy, Python 3.12, Gazebo Sim 8 (Harmonic), `ros_gz_bridge`, `ros_gz_sim`, `gz_ros2_control`, `joint_trajectory_controller`, OpenCV 5.0, NumPy, `cv_bridge`, `tf2_ros`, pytest via `ament_cmake_pytest`.

**Spec:** `ROSpider/docs/superpowers/specs/2026-09-12-sim-pick-and-place-design.md`

## Global Constraints

- All paths are relative to `ROSpider/`. Build from `ROSpider/`, never from the workspace root.
- `export need_compile=True` before any launch; launch files read it with `os.environ[...]` and raise `KeyError` without it.
- Everything lands in `src/simulations/rospider_gazebo`. Hiwonder's upstream packages are not modified.
- `worlds/rospider_room.sdf` must not change. It backs `maps/rospider_room.*` and the Nav2 demo's AMCL initial pose.
- Arm geometry constants, all in metres, from `rospider_description/urdf/arm.urdf.xacro`: `L1 = 0.080000`, `L2 = 0.080563`, `L3 = 0.129632`, shoulder at `(0.065006, 0.0, 0.125610)` in `base_link`, `base_link` `0.116091` above `base_footprint`.
- Joint limits: `joint1` ±π; `joint2`–`joint5` ±2.09 rad. Gripper `r_joint` ∈ [-0.66, 0.75], open `0.5066`, closed `-0.6288`.
- Anything needing a Gazebo link on the hand must name **`link5`**. `gripper_link` and `end_effector_link` are lumped away by their fixed joints.
- `arm_ik.py` works exclusively in the `base_link` frame.
- Build and test commands:
  - `colcon build --packages-select rospider_gazebo --symlink-install`
  - `colcon test --packages-select rospider_gazebo && colcon test-result --verbose`

---

### Task 1: Exact forward kinematics

**Files:**
- Create: `src/simulations/rospider_gazebo/rospider_gazebo/arm_ik.py`
- Create: `src/simulations/rospider_gazebo/test/test_arm_ik.py`
- Modify: `src/simulations/rospider_gazebo/CMakeLists.txt`
- Modify: `src/simulations/rospider_gazebo/package.xml`

**Interfaces:**
- Consumes: nothing.
- Produces: `JOINT_NAMES: tuple[str, ...]`, `JOINT_LIMITS: tuple[tuple[float, float], ...]`, `SHOULDER: np.ndarray` shape (3,), `L1`, `L2`, `L3`, `MAX_REACH`, `WRIST_REACH`, `BASE_LINK_HEIGHT` (all `float`), and `forward_kinematics(q: Sequence[float]) -> tuple[np.ndarray, float]` returning the `end_effector_link` origin in `base_link` and the approach pitch in radians (positive = pointing down).

- [ ] **Step 1: Write the failing test**

Create `src/simulations/rospider_gazebo/test/test_arm_ik.py`:

```python
import math

import numpy as np

from rospider_gazebo import arm_ik


def test_zero_pose_matches_urdf_chain():
    position, pitch = arm_ik.forward_kinematics((0.0, 0.0, 0.0, 0.0, 0.0))
    np.testing.assert_allclose(
        position, (0.062415239, 0.004078388, 0.415684349), atol=1e-7)
    assert math.isclose(pitch, -1.5563673267948994, abs_tol=1e-9)


def test_init_pose_tilts_52_degrees_down():
    # controller/config/init_pose.yaml; CLAUDE.md records this pose as "camera
    # tilted 52 deg down at the floor".
    _, pitch = arm_ik.forward_kinematics((0.0, 0.628, -1.927, -1.194, 0.0))
    assert math.isclose(math.degrees(pitch), 52.012, abs_tol=0.01)


def test_constants_match_the_urdf():
    assert math.isclose(arm_ik.MAX_REACH, 0.290195, abs_tol=1e-6)
    assert math.isclose(arm_ik.WRIST_REACH, 0.160563, abs_tol=1e-6)
    assert arm_ik.JOINT_NAMES == (
        'joint1', 'joint2', 'joint3', 'joint4', 'joint5')
```

- [ ] **Step 2: Run test to verify it fails**

Run: `colcon build --packages-select rospider_gazebo --symlink-install && colcon test --packages-select rospider_gazebo --pytest-args -k arm_ik && colcon test-result --verbose`
Expected: FAIL with `ModuleNotFoundError: No module named 'rospider_gazebo.arm_ik'`

- [ ] **Step 3: Write the forward kinematics**

Create `src/simulations/rospider_gazebo/rospider_gazebo/arm_ik.py`:

```python
"""Closed-form kinematics for the ROSpider 5-DOF arm, in the base_link frame.

Hiwonder ships its arm IK as arm_kinematics/*.so, which is aarch64-only, and
MoveIt's config sets position_only_ik so it cannot constrain a grasp's
orientation. Everything here is derived from rospider_description's URDF.

Joint axes: joint1 is +Z (yaw), joint2/3/4 are all -Y (a planar 3R arm), joint5
is +Z along its own link (roll). "Pitch" below is the approach angle of the hand:
0 points horizontally forward, +pi/2 points straight down.
"""

import math
from typing import Optional, Sequence

import numpy as np

JOINT_NAMES = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5')
JOINT_LIMITS = ((-math.pi, math.pi),) + ((-2.09, 2.09),) * 4

# base_link -> joint1 -> ... -> joint5, as (xyz, rpy, axis) straight from
# arm.urdf.xacro. The lateral terms are sub-millimetre but are kept so that
# forward_kinematics is exact and can be used to refine the idealised solution.
_CHAIN = (
    ((0.065006, 0.0, 0.082583), (0.0, 0.0, 0.0), (0.00040398, 0.0, 1.0)),
    ((-2.7885e-05, 4.5188e-04, 0.043027), (0.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
    ((0.0, -1.5386e-04, 0.08), (0.0, 0.0, 1.2291e-05), (0.0, -1.0, 0.0)),
    ((3.5733e-04, 1.0042e-03, 0.080556), (0.0, -0.014429, 0.0), (0.0, -1.0, 0.0)),
    ((-1.0511e-03, 2.7762e-03, 0.049547), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
)
_END_EFFECTOR_OFFSET = (0.0, 0.0, 0.08)   # link5 -> end_effector_link, fixed

SHOULDER = np.array([0.065006, 0.0, 0.125610])   # joint2 origin in base_link
L1 = 0.080000          # joint2 -> joint3
L2 = 0.080563          # joint3 -> joint4
L3 = 0.129632          # joint4 -> end_effector_link (joint5 adds no length)
MAX_REACH = L1 + L2 + L3
WRIST_REACH = L1 + L2
BASE_LINK_HEIGHT = 0.116091   # base_footprint -> base_link, base.urdf.xacro:10


def _rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _axis_rotation(axis: Sequence[float], angle: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def _transform(xyz, rpy, axis, angle) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = _rpy(*rpy) @ _axis_rotation(axis, angle)
    m[:3, 3] = xyz
    return m


def forward_kinematics(q: Sequence[float]):
    """End effector origin in base_link, and the hand's approach pitch."""
    m = np.eye(4)
    for (xyz, rpy, axis), angle in zip(_CHAIN, q):
        m = m @ _transform(xyz, rpy, axis, angle)
    offset = np.eye(4)
    offset[:3, 3] = _END_EFFECTOR_OFFSET
    m = m @ offset
    approach = m[:3, 2]     # the hand points along its own +Z
    return m[:3, 3], -math.asin(float(np.clip(approach[2], -1.0, 1.0)))
```

- [ ] **Step 4: Register the module and the test**

Edit `src/simulations/rospider_gazebo/CMakeLists.txt`. `ament_python_install_package(${PROJECT_NAME})` already installs `arm_ik.py` alongside `maps.py`, so only the test registration is new. Insert immediately before `ament_package()`:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # APPEND_ENV puts the package source on PYTHONPATH so `from rospider_gazebo
  # import arm_ik` resolves without depending on the install space being sourced.
  ament_add_pytest_test(test_arm_ik test/test_arm_ik.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()
```

Edit `src/simulations/rospider_gazebo/package.xml`, adding after the `<buildtool_depend>` lines:

```xml
  <test_depend>ament_cmake_pytest</test_depend>
  <test_depend>python3-pytest</test_depend>
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `colcon build --packages-select rospider_gazebo --symlink-install && colcon test --packages-select rospider_gazebo --pytest-args -k arm_ik && colcon test-result --verbose`
Expected: PASS, 3 tests

- [ ] **Step 6: Commit**

```bash
git add ROSpider/src/simulations/rospider_gazebo/rospider_gazebo/arm_ik.py \
        ROSpider/src/simulations/rospider_gazebo/test/test_arm_ik.py \
        ROSpider/src/simulations/rospider_gazebo/CMakeLists.txt \
        ROSpider/src/simulations/rospider_gazebo/package.xml
git commit -m "feat(sim): add exact forward kinematics for the ROSpider arm

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: Closed-form inverse kinematics and grasp planning

**Files:**
- Modify: `src/simulations/rospider_gazebo/rospider_gazebo/arm_ik.py`
- Modify: `src/simulations/rospider_gazebo/test/test_arm_ik.py`

**Interfaces:**
- Consumes: everything Task 1 produced.
- Produces:
  - `reach_limit(z: float) -> float` — bare-chain horizontal reach from the shoulder axis at `base_link` height `z`.
  - `tool_axis(pitch: float, yaw: float) -> np.ndarray` shape (3,) — unit approach direction.
  - `inverse_kinematics(target: Sequence[float], pitch: float, roll: float = 0.0) -> Optional[np.ndarray]` — five joint angles, or `None`. Returns the elbow branch with the larger joint-limit margin.
  - `GraspPlan` — `NamedTuple` with fields `pitch: float` (radians), `approach: np.ndarray` shape (5,), `grasp: np.ndarray` shape (5,), `margin: float` (radians).
  - `plan_grasp(target, pitches_deg: Sequence[float], back_off: float, roll: float = 0.0) -> Optional[GraspPlan]`.

- [ ] **Step 1: Write the failing tests**

Append to `src/simulations/rospider_gazebo/test/test_arm_ik.py`:

```python
CUBE_Z = 0.105 - arm_ik.BASE_LINK_HEIGHT      # cube centre, base_link frame
PITCHES = (90.0, 80.0, 70.0, 60.0, 50.0, 40.0, 30.0)
LAYOUT = {                                    # spec section 5.1
    'red': (0.235, 0.07),
    'green': (0.235, 0.0),
    'blue': (0.235, -0.07),
    'drop': (0.160, 0.0),
}


def test_round_trip_over_the_pedestal_workspace():
    checked = 0
    for x in np.arange(0.16, 0.28, 0.02):
        for y in np.arange(-0.09, 0.091, 0.03):
            for pitch_deg in (60.0, 75.0, 90.0):
                target = np.array([x, y, CUBE_Z])
                pitch = math.radians(pitch_deg)
                q = arm_ik.inverse_kinematics(target, pitch)
                if q is None:
                    continue
                position, actual = arm_ik.forward_kinematics(q)
                assert np.linalg.norm(position - target) < 1e-3
                assert abs(actual - pitch) < math.radians(0.5)
                checked += 1
    assert checked > 50, f'only {checked} poses solved; the workspace moved'


def test_out_of_reach_returns_none():
    far = np.array([arm_ik.SHOULDER[0] + arm_ik.MAX_REACH + 0.02, 0.0, CUBE_Z])
    assert all(arm_ik.inverse_kinematics(far, math.radians(p)) is None
               for p in range(-80, 81, 5))
    assert arm_ik.inverse_kinematics(arm_ik.SHOULDER, math.radians(90)) is None


def test_reach_limit_brackets_the_solvable_set():
    limit = arm_ik.reach_limit(CUBE_Z)
    assert math.isclose(limit, 0.255980, abs_tol=1e-5)
    inside = np.array([arm_ik.SHOULDER[0] + limit - 0.002, 0.0, CUBE_Z])
    outside = np.array([arm_ik.SHOULDER[0] + limit + 0.002, 0.0, CUBE_Z])
    assert any(arm_ik.inverse_kinematics(inside, math.radians(p)) is not None
               for p in range(-80, 81, 1))
    assert all(arm_ik.inverse_kinematics(outside, math.radians(p)) is None
               for p in range(-80, 81, 1))


def test_solutions_respect_joint_limits():
    q = arm_ik.inverse_kinematics((0.235, 0.0, CUBE_Z), math.radians(80))
    assert q is not None
    for value, (low, high) in zip(q, arm_ik.JOINT_LIMITS):
        assert low <= value <= high


def test_scene_layout_is_graspable():
    for name, (x, y) in LAYOUT.items():
        plan = arm_ik.plan_grasp((x, y, CUBE_Z), PITCHES, back_off=0.04)
        assert plan is not None, f'{name} has no workable approach'
        assert plan.margin > math.radians(10.0), (
            f'{name} sits {math.degrees(plan.margin):.1f} deg from a joint limit')
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `colcon test --packages-select rospider_gazebo --pytest-args -k arm_ik && colcon test-result --verbose`
Expected: FAIL with `AttributeError: module 'rospider_gazebo.arm_ik' has no attribute 'inverse_kinematics'`

- [ ] **Step 3: Write the inverse kinematics**

Append to `src/simulations/rospider_gazebo/rospider_gazebo/arm_ik.py`:

```python
class GraspPlan(NamedTuple):
    """An approach pitch plus the two arm poses a grasp needs."""

    pitch: float
    approach: np.ndarray
    grasp: np.ndarray
    margin: float


def reach_limit(z: float) -> float:
    """Horizontal reach from the shoulder axis at base_link height z.

    This is the bare chain, with the hand free to point anywhere. A grasp fixes
    the approach direction and reaches considerably less; use plan_grasp.
    """
    drop = abs(z - SHOULDER[2])
    if drop >= MAX_REACH:
        return 0.0
    return math.sqrt(MAX_REACH * MAX_REACH - drop * drop)


def tool_axis(pitch: float, yaw: float) -> np.ndarray:
    """Unit vector the hand points along, for an approach pitch and a yaw."""
    return np.array([math.cos(pitch) * math.cos(yaw),
                     math.cos(pitch) * math.sin(yaw),
                     -math.sin(pitch)])


def _limit_margin(q: Sequence[float]) -> float:
    return min(min(high - value, value - low)
               for value, (low, high) in zip(q, JOINT_LIMITS))


def _seed_branches(target: np.ndarray, pitch: float, roll: float):
    """Both elbow solutions of the idealised planar model, or None."""
    delta = target - SHOULDER
    q1 = math.atan2(delta[1], delta[0])
    rho = math.hypot(delta[0], delta[1])
    height = delta[2]

    # Angles below are measured from +Z towards the radial direction, so the
    # last link's direction is phi = pi/2 + pitch.
    phi = math.pi / 2.0 + pitch
    wrist_rho = rho - L3 * math.sin(phi)
    wrist_height = height - L3 * math.cos(phi)

    cosine = ((wrist_rho ** 2 + wrist_height ** 2 - L1 * L1 - L2 * L2)
              / (2.0 * L1 * L2))
    if abs(cosine) > 1.0:
        return None

    branches = []
    for sign in (-1.0, 1.0):
        elbow = sign * math.acos(max(-1.0, min(1.0, cosine)))
        shoulder = (math.atan2(wrist_rho, wrist_height)
                    - math.atan2(L2 * math.sin(elbow), L1 + L2 * math.cos(elbow)))
        # URDF joints 2-4 turn about -Y, so their values are the negated
        # about-+Y angles used above.
        branches.append(np.array([q1, -shoulder, -elbow,
                                  -(phi - (shoulder + elbow)), roll]))
    return branches


def _refine(q: np.ndarray, target: np.ndarray, pitch: float) -> np.ndarray:
    """Newton steps on the exact chain, clearing the idealisation's error."""
    q = np.array(q, dtype=float)
    for _ in range(20):
        position, actual = forward_kinematics(q)
        residual = np.array([position[0] - target[0],
                             position[1] - target[1],
                             position[2] - target[2],
                             actual - pitch])
        if np.max(np.abs(residual)) < 1e-10:
            break
        jacobian = np.zeros((4, 4))
        for column in range(4):
            nudged = q.copy()
            nudged[column] += 1e-6
            moved, moved_pitch = forward_kinematics(nudged)
            jacobian[:, column] = np.array([moved[0] - position[0],
                                            moved[1] - position[1],
                                            moved[2] - position[2],
                                            moved_pitch - actual]) / 1e-6
        q[:4] += np.linalg.lstsq(jacobian, -residual, rcond=None)[0]
    return q


def inverse_kinematics(target: Sequence[float], pitch: float,
                       roll: float = 0.0) -> Optional[np.ndarray]:
    """Joint angles reaching target with the hand at the given approach pitch.

    Returns the elbow branch with the larger joint-limit margin. Which branch
    is chosen matters: taking the first valid one leaves the pedestal grasps
    under a degree from a limit, while choosing by margin leaves tens of
    degrees.
    """
    target = np.asarray(target, dtype=float)
    branches = _seed_branches(target, pitch, roll)
    if branches is None:
        return None

    best = None
    for seed in branches:
        q = _refine(seed, target, pitch)
        position, actual = forward_kinematics(q)
        if np.linalg.norm(position - target) > 1e-4:
            continue
        if abs(actual - pitch) > 1e-4:
            continue
        if any(not low <= value <= high
               for value, (low, high) in zip(q, JOINT_LIMITS)):
            continue
        margin = _limit_margin(q)
        if best is None or margin > best[0]:
            best = (margin, q)
    return None if best is None else best[1]


def plan_grasp(target: Sequence[float], pitches_deg: Sequence[float],
               back_off: float, roll: float = 0.0) -> Optional[GraspPlan]:
    """Best approach pitch for which grasp and pre-grasp both solve.

    The pre-grasp backs off along the tool axis rather than straight up: at
    these heights a vertical hover needs more than the L1+L2 sub-chain has.
    """
    target = np.asarray(target, dtype=float)
    yaw = math.atan2(target[1] - SHOULDER[1], target[0] - SHOULDER[0])

    best = None
    for pitch_deg in pitches_deg:
        pitch = math.radians(pitch_deg)
        grasp = inverse_kinematics(target, pitch, roll)
        if grasp is None:
            continue
        approach = inverse_kinematics(
            target - back_off * tool_axis(pitch, yaw), pitch, roll)
        if approach is None:
            continue
        margin = min(_limit_margin(grasp), _limit_margin(approach))
        if best is None or margin > best.margin:
            best = GraspPlan(pitch, approach, grasp, margin)
    return best
```

Add `NamedTuple` to the typing import at the top of the file:

```python
from typing import NamedTuple, Optional, Sequence
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `colcon test --packages-select rospider_gazebo --pytest-args -k arm_ik && colcon test-result --verbose`
Expected: PASS, 8 tests

- [ ] **Step 5: Commit**

```bash
git add ROSpider/src/simulations/rospider_gazebo/rospider_gazebo/arm_ik.py \
        ROSpider/src/simulations/rospider_gazebo/test/test_arm_ik.py
git commit -m "feat(sim): add closed-form arm IK and grasp planning

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: Scene models and the pick-and-place launch file

**Files:**
- Create: `src/simulations/rospider_gazebo/models/pick_pedestal/model.sdf`
- Create: `src/simulations/rospider_gazebo/models/pick_cube_red/model.sdf`
- Create: `src/simulations/rospider_gazebo/models/pick_cube_green/model.sdf`
- Create: `src/simulations/rospider_gazebo/models/pick_cube_blue/model.sdf`
- Create: `src/simulations/rospider_gazebo/models/drop_marker/model.sdf`
- Create: `src/simulations/rospider_gazebo/launch/pick_place.launch.py`
- Modify: `src/simulations/rospider_gazebo/CMakeLists.txt`

**Interfaces:**
- Consumes: `gazebo.launch.py`'s existing arguments `world`, `gui`, `arm_pose`, `x`, `y`, `yaw`.
- Produces: Gazebo models named exactly `pick_cube_red`, `pick_cube_green`, `pick_cube_blue`, each with a single link named `link` — Task 4's `DetachableJoint` entries reference these names. Also `pick_place.launch.py` with arguments `world`, `gui`, `auto_start`.

- [ ] **Step 1: Write the pedestal model**

Create `src/simulations/rospider_gazebo/models/pick_pedestal/model.sdf`:

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="pick_pedestal">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>0.14 0.22 0.08</size></box></geometry>
        <material>
          <ambient>0.75 0.70 0.60 1</ambient>
          <diffuse>0.75 0.70 0.60 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry><box><size>0.14 0.22 0.08</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
```

- [ ] **Step 2: Write the three cube models**

Create `src/simulations/rospider_gazebo/models/pick_cube_red/model.sdf`. Inertia is that of a uniform 0.05 m cube of mass 0.05 kg: `I = m * s^2 / 6 = 0.05 * 0.0025 / 6 = 2.0833e-05`.

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="pick_cube_red">
    <link name="link">
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>2.0833e-05</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>2.0833e-05</iyy><iyz>0</iyz><izz>2.0833e-05</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
        <material>
          <ambient>0.9 0.05 0.05 1</ambient>
          <diffuse>0.9 0.05 0.05 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
    </link>
  </model>
</sdf>
```

Create `models/pick_cube_green/model.sdf` and `models/pick_cube_blue/model.sdf` as exact copies with the model name changed to `pick_cube_green` / `pick_cube_blue` and the two colour lines changed to `0.05 0.8 0.05 1` and `0.05 0.1 0.9 1` respectively.

- [ ] **Step 3: Write the drop marker**

Create `src/simulations/rospider_gazebo/models/drop_marker/model.sdf`:

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="drop_marker">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>0.05 0.05 0.004</size></box></geometry>
        <material>
          <ambient>0.95 0.85 0.1 1</ambient>
          <diffuse>0.95 0.85 0.1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

- [ ] **Step 4: Write the launch file**

Create `src/simulations/rospider_gazebo/launch/pick_place.launch.py`. `color_detect` and `pick_and_place` are added in Tasks 5 and 6; this task delivers the scene.

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Objects are spawned at run time rather than written into rospider_room.sdf:
# that world backs maps/rospider_room.* and the Nav2 demo's AMCL initial pose.
# Poses are in the world frame with the robot at the origin, so they are also
# base_footprint coordinates. See spec section 5.1.
SCENE = (
    ('pick_pedestal', (0.200, 0.0, 0.040)),
    ('pick_cube_red', (0.235, 0.07, 0.105)),
    ('pick_cube_green', (0.235, 0.0, 0.105)),
    ('pick_cube_blue', (0.235, -0.07, 0.105)),
    ('drop_marker', (0.160, 0.0, 0.082)),
)


def generate_launch_description():
    pkg = get_package_share_directory('rospider_gazebo')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'arm_pose': 'init',
        }.items(),
    )

    spawns = [
        Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{name}',
            output='screen',
            arguments=[
                '-file', os.path.join(pkg, 'models', name, 'model.sdf'),
                '-name', name,
                '-x', str(pose[0]), '-y', str(pose[1]), '-z', str(pose[2]),
            ],
        )
        for name, pose in SCENE
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg, 'worlds', 'rospider_room.sdf')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        gazebo,
        # The robot and its controllers need to exist before the cubes land on
        # the pedestal, or they drop through a world that is still loading.
        TimerAction(period=5.0, actions=spawns),
    ])
```

- [ ] **Step 5: Install the models**

Edit `src/simulations/rospider_gazebo/CMakeLists.txt`, adding `models` to the existing directory install:

```cmake
install(DIRECTORY config launch maps models rviz urdf worlds
  DESTINATION share/${PROJECT_NAME})
```

- [ ] **Step 6: Build and run the scene**

Run:

```bash
colcon build --packages-select rospider_gazebo --symlink-install
source install/local_setup.bash
export need_compile=True
ros2 launch rospider_gazebo pick_place.launch.py
```

Expected: Gazebo shows the robot with a pedestal in front of it, three coloured cubes resting on top, and a yellow marker. The cubes stay at rest; they do not sink, slide off or fall through.

- [ ] **Step 7: Verify the layout against the spec**

With the simulation running, in a second shell:

```bash
source install/local_setup.bash
ros2 run tf2_ros tf2_echo base_footprint end_LF 2>&1 | head -20
ros2 run tf2_ros tf2_echo base_footprint end_RF 2>&1 | head -20
```

Expected: front feet near `(0.20, ±0.15)`. Confirm visually in Gazebo that the pedestal (spanning `x` 0.13–0.27, `|y| ≤ 0.11`) does not clip a leg and does not overlap `sim_skid_link` (`|x| ≤ 0.12`). If a foot lands inside the pedestal footprint, move the pedestal and the objects on it forward in `SCENE` and re-run `colcon test --packages-select rospider_gazebo` — `test_scene_layout_is_graspable` will fail if the new position leaves the workspace. Record the measured foot positions in the commit message.

- [ ] **Step 8: Commit**

```bash
git add ROSpider/src/simulations/rospider_gazebo/models \
        ROSpider/src/simulations/rospider_gazebo/launch/pick_place.launch.py \
        ROSpider/src/simulations/rospider_gazebo/CMakeLists.txt
git commit -m "feat(sim): spawn a pick-and-place scene without touching the world

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: Attach and detach through DetachableJoint

**Files:**
- Modify: `src/simulations/rospider_gazebo/urdf/rospider_gazebo.urdf.xacro`
- Modify: `src/simulations/rospider_gazebo/config/gz_bridge.yaml`

**Interfaces:**
- Consumes: the Gazebo model names from Task 3.
- Produces: ROS topics `/grasp/red/attach`, `/grasp/red/detach`, and the same for `green` and `blue`, all `std_msgs/msg/Empty`, bridged `ROS_TO_GZ`.

- [ ] **Step 1: Add the plugins to the robot description**

Edit `src/simulations/rospider_gazebo/urdf/rospider_gazebo.urdf.xacro`. Inside the same `<gazebo>` block that holds `gz_ros2_control-system` and `gz-sim-velocity-control-system`, after the `OdometryPublisher` plugin, add:

```xml
    <!-- Grasping is a joint attachment, not friction: the gripper is small, has
         five mimic fingers and would drop a 5 cm cube. parent_link must be link5
         because gripper_joint and end_effector_joint are fixed, so sdformat lumps
         gripper_link and end_effector_link into link5. child_model is fixed at
         load time, hence one plugin per cube. suppress_child_warning is needed
         because pick_place.launch.py spawns the cubes after the robot. -->
    <xacro:macro name="grasp_joint" params="colour">
      <plugin filename="gz-sim-detachable-joint-system"
              name="gz::sim::systems::DetachableJoint">
        <parent_link>link5</parent_link>
        <child_model>pick_cube_${colour}</child_model>
        <child_link>link</child_link>
        <attach_topic>/grasp/${colour}/attach</attach_topic>
        <detach_topic>/grasp/${colour}/detach</detach_topic>
        <suppress_child_warning>true</suppress_child_warning>
      </plugin>
    </xacro:macro>
    <xacro:grasp_joint colour="red"/>
    <xacro:grasp_joint colour="green"/>
    <xacro:grasp_joint colour="blue"/>
```

- [ ] **Step 2: Bridge the six topics**

Append to `src/simulations/rospider_gazebo/config/gz_bridge.yaml`:

```yaml
# Grasping: pick_and_place.py welds a cube to link5 and releases it again. See
# the DetachableJoint plugins in urdf/rospider_gazebo.urdf.xacro.
- ros_topic_name: /grasp/red/attach
  gz_topic_name: /grasp/red/attach
  ros_type_name: std_msgs/msg/Empty
  gz_type_name: gz.msgs.Empty
  direction: ROS_TO_GZ

- ros_topic_name: /grasp/red/detach
  gz_topic_name: /grasp/red/detach
  ros_type_name: std_msgs/msg/Empty
  gz_type_name: gz.msgs.Empty
  direction: ROS_TO_GZ

- ros_topic_name: /grasp/green/attach
  gz_topic_name: /grasp/green/attach
  ros_type_name: std_msgs/msg/Empty
  gz_type_name: gz.msgs.Empty
  direction: ROS_TO_GZ

- ros_topic_name: /grasp/green/detach
  gz_topic_name: /grasp/green/detach
  ros_type_name: std_msgs/msg/Empty
  gz_type_name: gz.msgs.Empty
  direction: ROS_TO_GZ

- ros_topic_name: /grasp/blue/attach
  gz_topic_name: /grasp/blue/attach
  ros_type_name: std_msgs/msg/Empty
  gz_type_name: gz.msgs.Empty
  direction: ROS_TO_GZ

- ros_topic_name: /grasp/blue/detach
  gz_topic_name: /grasp/blue/detach
  ros_type_name: std_msgs/msg/Empty
  gz_type_name: gz.msgs.Empty
  direction: ROS_TO_GZ
```

- [ ] **Step 3: Verify the two behaviours the spec flags as unproven**

Build and launch:

```bash
colcon build --packages-select rospider_gazebo --symlink-install
source install/local_setup.bash
export need_compile=True
ros2 launch rospider_gazebo pick_place.launch.py
```

In a second shell, check the startup behaviour first — spec section 5.2 warns Gazebo may create the joint as soon as the child model appears:

```bash
source install/local_setup.bash
ros2 topic pub --once /grasp/red/detach std_msgs/msg/Empty "{}"
```

Expected: the red cube sits on the pedestal, unmoved. If it was hanging from the gripper before this command, the detach-on-startup in Task 6 is required; note that in the commit message.

Then test attach with the arm parked, which puts `link5` well away from the cube:

```bash
ros2 topic pub --once /grasp/red/attach std_msgs/msg/Empty "{}"
```

Expected: the cube stays exactly where it is. If it teleports to the hand, record the offset — the spec's fallback is to position the hand so the offset becomes the intended grasp offset.

Finally confirm the weld holds by moving the arm:

```bash
ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [joint1, joint2, joint3, joint4, joint5],
    points: [{positions: [0.0, 0.3, -1.2, -0.8, 0.0],
              time_from_start: {sec: 2}}]}'
```

Expected: the cube travels with the hand. Then `ros2 topic pub --once /grasp/red/detach std_msgs/msg/Empty "{}"` and the cube falls.

- [ ] **Step 4: Commit**

```bash
git add ROSpider/src/simulations/rospider_gazebo/urdf/rospider_gazebo.urdf.xacro \
        ROSpider/src/simulations/rospider_gazebo/config/gz_bridge.yaml
git commit -m "feat(sim): weld cubes to the gripper with DetachableJoint

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: HSV colour detector

**Files:**
- Create: `src/simulations/rospider_gazebo/scripts/color_detect.py`
- Create: `src/simulations/rospider_gazebo/config/color_detect.yaml`
- Modify: `src/simulations/rospider_gazebo/CMakeLists.txt`
- Modify: `src/simulations/rospider_gazebo/package.xml`
- Modify: `src/simulations/rospider_gazebo/launch/pick_place.launch.py`

**Interfaces:**
- Consumes: `/depth_cam/rgb/image_raw`.
- Produces: `/yolo/object_detect` (`interfaces/msg/ObjectsInfo`) where each `ObjectInfo` has `class_name` in `{'red', 'green', 'blue'}`, `box = [x1, y1, x2, y2]` in pixels, `score = 1.0`, `width`/`height` of the source image, and `angle` in degrees from `cv2.minAreaRect`. Also `/color_detect/image_result` (`sensor_msgs/msg/Image`, `bgr8`).

- [ ] **Step 1: Write the config**

Create `src/simulations/rospider_gazebo/config/color_detect.yaml`:

```yaml
# HSV bounds for the simulated cubes. OpenCV hue is 0-179. Red wraps the hue
# origin, so it carries two ranges that are OR-ed together.
color_detect:
  ros__parameters:
    min_area_px: 300
    kernel_px: 5
    colors: ['red', 'green', 'blue']
    red:
      lower: [0, 120, 80, 170, 120, 80]
      upper: [10, 255, 255, 179, 255, 255]
    green:
      lower: [40, 100, 60]
      upper: [85, 255, 255]
    blue:
      lower: [95, 120, 60]
      upper: [130, 255, 255]
```

- [ ] **Step 2: Write the node**

Create `src/simulations/rospider_gazebo/scripts/color_detect.py`:

```python
#!/usr/bin/env python3
"""HSV cube detector for the simulation.

Publishes interfaces/ObjectsInfo on /yolo/object_detect, the same topic and
message competition/yolo_node.py uses on the real robot, so a YOLO node can
replace this one without the pick node changing.
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from interfaces.msg import ObjectInfo, ObjectsInfo
from rclpy.node import Node
from sensor_msgs.msg import Image

DRAW_BGR = {'red': (0, 0, 255), 'green': (0, 255, 0), 'blue': (255, 0, 0)}


class ColorDetectNode(Node):

    def __init__(self):
        super().__init__('color_detect',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.bridge = CvBridge()
        self.min_area = int(self.get_parameter('min_area_px').value)
        kernel_px = int(self.get_parameter('kernel_px').value)
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_px, kernel_px))

        self.ranges = {}
        for color in self.get_parameter('colors').value:
            lower = list(self.get_parameter(f'{color}.lower').value)
            upper = list(self.get_parameter(f'{color}.upper').value)
            self.ranges[color] = [
                (np.array(lower[i:i + 3], dtype=np.uint8),
                 np.array(upper[i:i + 3], dtype=np.uint8))
                for i in range(0, len(lower), 3)
            ]

        self.objects_pub = self.create_publisher(
            ObjectsInfo, '/yolo/object_detect', 1)
        self.image_pub = self.create_publisher(Image, '~/image_result', 1)
        self.create_subscription(
            Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.get_logger().info(
            f'watching for {sorted(self.ranges)} on /depth_cam/rgb/image_raw')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        height, width = frame.shape[:2]

        result = ObjectsInfo()
        for color, ranges in self.ranges.items():
            mask = None
            for lower, upper in ranges:
                band = cv2.inRange(hsv, lower, upper)
                mask = band if mask is None else cv2.bitwise_or(mask, band)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if cv2.contourArea(contour) < self.min_area:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                info = ObjectInfo()
                info.class_name = color
                info.box = [int(x), int(y), int(x + w), int(y + h)]
                info.score = 1.0
                info.width = int(width)
                info.height = int(height)
                info.angle = int(cv2.minAreaRect(contour)[2])
                result.objects.append(info)

                cv2.rectangle(frame, (x, y), (x + w, y + h),
                              DRAW_BGR[color], 2)
                cv2.putText(frame, color, (x, max(0, y - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, DRAW_BGR[color], 1)

        self.objects_pub.publish(result)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))


def main():
    rclpy.init()
    node = ColorDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 3: Install the script and declare the dependencies**

Edit `src/simulations/rospider_gazebo/CMakeLists.txt`, extending the existing `install(PROGRAMS ...)`:

```cmake
install(PROGRAMS scripts/sim_gait.py scripts/color_detect.py
  DESTINATION lib/${PROJECT_NAME})
```

Edit `src/simulations/rospider_gazebo/package.xml`, adding to the `<exec_depend>` block:

```xml
  <exec_depend>interfaces</exec_depend>
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>std_srvs</exec_depend>
  <exec_depend>tf2_geometry_msgs</exec_depend>
  <exec_depend>python3-opencv</exec_depend>
```

- [ ] **Step 4: Add the node to the launch file**

Edit `src/simulations/rospider_gazebo/launch/pick_place.launch.py`. Add after the `spawns` list comprehension:

```python
    detector = Node(
        package='rospider_gazebo',
        executable='color_detect.py',
        name='color_detect',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'color_detect.yaml'),
                    {'use_sim_time': True}],
    )
```

and add `detector` to the `TimerAction`'s action list: `actions=spawns + [detector]`.

- [ ] **Step 5: Build and verify detection**

Run:

```bash
colcon build --packages-select rospider_gazebo --symlink-install
source install/local_setup.bash
export need_compile=True
ros2 launch rospider_gazebo pick_place.launch.py
```

In a second shell:

```bash
source install/local_setup.bash
ros2 topic echo /yolo/object_detect --once
```

Expected: three `objects`, one each with `class_name` `red`, `green`, `blue`, boxes roughly 20–30 px wide, centred near `u` 162 / 323 / 485 and `v` ≈ 400 at the default `init` arm pose (spec section 5.1 projects the cubes to those pixels; `v` moves to ≈305 once Task 6 sets the look pose). Also view `ros2 run rqt_image_view rqt_image_view /color_detect/image_result` and confirm the boxes sit on the cubes. If a colour is missed, widen its range in `config/color_detect.yaml`.

- [ ] **Step 6: Commit**

```bash
git add ROSpider/src/simulations/rospider_gazebo/scripts/color_detect.py \
        ROSpider/src/simulations/rospider_gazebo/config/color_detect.yaml \
        ROSpider/src/simulations/rospider_gazebo/CMakeLists.txt \
        ROSpider/src/simulations/rospider_gazebo/package.xml \
        ROSpider/src/simulations/rospider_gazebo/launch/pick_place.launch.py
git commit -m "feat(sim): add an HSV cube detector on /yolo/object_detect

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: Pick-and-place state machine

**Files:**
- Create: `src/simulations/rospider_gazebo/scripts/pick_and_place.py`
- Create: `src/simulations/rospider_gazebo/config/pick_place.yaml`
- Modify: `src/simulations/rospider_gazebo/CMakeLists.txt`
- Modify: `src/simulations/rospider_gazebo/launch/pick_place.launch.py`

**Interfaces:**
- Consumes: `arm_ik.plan_grasp`, `arm_ik.forward_kinematics`, `arm_ik.JOINT_NAMES`, `arm_ik.GraspPlan` from Task 2; `/yolo/object_detect` from Task 5; `/grasp/<colour>/{attach,detach}` from Task 4.
- Produces: `/arm_controller/joint_trajectory`, `/gripper_controller/joint_trajectory`, services `/pick_and_place/start` (`interfaces/srv/SetString`) and `/pick_and_place/stop` (`std_srvs/srv/Trigger`).

- [ ] **Step 1: Write the config**

Create `src/simulations/rospider_gazebo/config/pick_place.yaml`:

```yaml
pick_and_place:
  ros__parameters:
    # Look pose: joint4 is tuned so the three cubes land near the image centre
    # (v ~= 305 of 480) rather than at the bottom edge as they do at joint4=-1.194.
    look_pose: [0.0, 0.628, -1.927, -1.35, 0.0]

    # Every pitch is evaluated; the one whose grasp and pre-grasp have the
    # largest joint-limit margin wins. See spec section 4.3.
    approach_pitches_deg: [90.0, 80.0, 70.0, 60.0, 50.0, 40.0, 30.0]
    approach_distance: 0.04

    # Offsets along base_link z. The detector sees the cube's top face, so the
    # grasp offset is negative to reach the middle of a 0.05 m cube.
    grasp_z_offset: -0.025
    release_z_offset: 0.03

    gripper_open: 0.5066        # rospider.srdf group_state "open"
    grasp_joint_value: 0.1      # partial close: the weld holds the cube, so a
                                # full close would just penetrate it

    # Drop point in base_footprint; converted to base_link once at startup.
    drop_point: [0.160, 0.0, 0.105]

    colors: ['red', 'green', 'blue']
    stable_frames: 5
    joint_tolerance: 0.02
    move_duration: 2.0
    settle_time: 1.0
    state_timeout: 15.0
    depth_window_px: 5
    auto_start: true
```

- [ ] **Step 2: Write the node**

Create `src/simulations/rospider_gazebo/scripts/pick_and_place.py`:

```python
#!/usr/bin/env python3
"""Pick coloured cubes off the pedestal and move them to the drop marker.

Look-then-move: the camera rides on link4, so moving the arm moves the camera
off the target and inside the depth sensor's 0.15 m near clip. The node detects
from one fixed look pose, latches the target in base_link, and only then moves.
"""

import math
from enum import Enum

import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from interfaces.msg import ObjectsInfo
from interfaces.srv import SetString
from rclpy.node import Node
from rospider_gazebo import arm_ik
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class State(Enum):
    IDLE = 'IDLE'
    LOOK = 'LOOK'
    LOCALIZE = 'LOCALIZE'
    PRE_GRASP = 'PRE_GRASP'
    DESCEND = 'DESCEND'
    GRASP = 'GRASP'
    LIFT = 'LIFT'
    TO_DROP = 'TO_DROP'
    LOWER = 'LOWER'
    RELEASE = 'RELEASE'
    RETREAT = 'RETREAT'
    DONE = 'DONE'


class PickAndPlaceNode(Node):

    def __init__(self):
        super().__init__('pick_and_place',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.bridge = CvBridge()
        p = self.get_parameter
        self.look_pose = np.array(p('look_pose').value, dtype=float)
        self.pitches = list(p('approach_pitches_deg').value)
        self.approach_distance = float(p('approach_distance').value)
        self.grasp_z_offset = float(p('grasp_z_offset').value)
        self.release_z_offset = float(p('release_z_offset').value)
        self.gripper_open = float(p('gripper_open').value)
        self.grasp_joint_value = float(p('grasp_joint_value').value)
        self.drop_point = np.array(p('drop_point').value, dtype=float)
        self.colors = list(p('colors').value)
        self.stable_frames = int(p('stable_frames').value)
        self.joint_tolerance = float(p('joint_tolerance').value)
        self.move_duration = float(p('move_duration').value)
        self.settle_time = float(p('settle_time').value)
        self.state_timeout = float(p('state_timeout').value)
        self.depth_window = int(p('depth_window_px').value)

        # drop_point arrives in base_footprint; arm_ik works in base_link.
        self.drop_point[2] -= arm_ik.BASE_LINK_HEIGHT

        self.state = State.IDLE
        self.state_entered = self.get_clock().now()
        self.goal = None
        self.target_color = None
        self.plan = None
        self.latched = None
        self.remaining = list(self.colors)
        self.detections = []
        self.streak = 0
        self.joint_state = {}
        self.depth_image = None
        self.intrinsics = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 1)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 1)
        self.attach_pubs = {
            c: self.create_publisher(Empty, f'/grasp/{c}/attach', 1)
            for c in self.colors}
        self.detach_pubs = {
            c: self.create_publisher(Empty, f'/grasp/{c}/detach', 1)
            for c in self.colors}

        self.create_subscription(
            ObjectsInfo, '/yolo/object_detect', self.objects_callback, 1)
        self.create_subscription(
            Image, '/depth_cam/depth/image_raw', self.depth_callback, 1)
        self.create_subscription(
            CameraInfo, '/depth_cam/depth/camera_info', self.info_callback, 1)
        self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)

        self.create_service(SetString, '~/start', self.start_callback)
        self.create_service(Trigger, '~/stop', self.stop_callback)

        # Gazebo builds a DetachableJoint as soon as it finds the child model,
        # so clear all three before doing anything else.
        self.release_all()
        self.create_timer(0.1, self.tick)
        if bool(p('auto_start').value):
            self.begin(self.colors)

    # ---------------------------------------------------------------- inputs

    def objects_callback(self, msg):
        self.detections = [(o.class_name, o.box) for o in msg.objects]

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

    def info_callback(self, msg):
        self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def joint_callback(self, msg):
        self.joint_state.update(zip(msg.name, msg.position))

    def start_callback(self, request, response):
        wanted = [request.data] if request.data else list(self.colors)
        self.begin([c for c in wanted if c in self.colors])
        response.success = True
        response.message = f'picking {self.remaining}'
        return response

    def stop_callback(self, _request, response):
        self.remaining = []
        self.enter(State.DONE)
        response.success = True
        response.message = 'stopped'
        return response

    # --------------------------------------------------------------- helpers

    def begin(self, colors):
        self.remaining = list(colors)
        self.enter(State.LOOK)

    def enter(self, state):
        self.get_logger().info(f'{self.state.value} -> {state.value}')
        self.state = state
        self.state_entered = self.get_clock().now()
        self.streak = 0

    def elapsed(self):
        return (self.get_clock().now() - self.state_entered).nanoseconds * 1e-9

    def send_arm(self, q):
        self.goal = np.asarray(q, dtype=float)
        msg = JointTrajectory()
        msg.joint_names = list(arm_ik.JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in self.goal]
        point.time_from_start.sec = int(self.move_duration)
        point.time_from_start.nanosec = int(
            (self.move_duration % 1.0) * 1e9)
        msg.points.append(point)
        self.arm_pub.publish(msg)

    def send_gripper(self, value):
        msg = JointTrajectory()
        msg.joint_names = ['r_joint']
        point = JointTrajectoryPoint()
        point.positions = [float(value)]
        point.time_from_start.sec = 1
        msg.points.append(point)
        self.gripper_pub.publish(msg)

    def arrived(self):
        if self.goal is None:
            return False
        for name, want in zip(arm_ik.JOINT_NAMES, self.goal):
            if name not in self.joint_state:
                return False
            if abs(self.joint_state[name] - want) > self.joint_tolerance:
                return False
        return True

    def release_all(self):
        for pub in self.detach_pubs.values():
            pub.publish(Empty())

    def abandon(self, reason):
        self.get_logger().warn(f'{self.target_color}: {reason}; skipping')
        self.release_all()
        self.send_gripper(self.gripper_open)
        if self.target_color in self.remaining:
            self.remaining.remove(self.target_color)
        self.target_color = None
        self.enter(State.LOOK)

    def localize(self, box):
        """Box centroid + depth -> a point in base_link, or None."""
        if self.depth_image is None or self.intrinsics is None:
            return None
        u = int((box[0] + box[2]) / 2)
        v = int((box[1] + box[3]) / 2)
        half = self.depth_window // 2
        patch = self.depth_image[max(0, v - half):v + half + 1,
                                 max(0, u - half):u + half + 1]
        patch = patch[np.isfinite(patch) & (patch > 0.0)]
        if patch.size == 0:
            return None
        depth = float(np.median(patch))

        fx, fy, cx, cy = self.intrinsics
        camera_point = np.array([(u - cx) * depth / fx,
                                 (v - cy) * depth / fy,
                                 depth])
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', 'depth_cam_frame', rclpy.time.Time())
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(f'no transform: {exc}')
            return None

        t = tf.transform.translation
        r = tf.transform.rotation
        rotation = _quaternion_matrix(r.x, r.y, r.z, r.w)
        return rotation @ camera_point + np.array([t.x, t.y, t.z])

    def plan_for(self, point):
        return arm_ik.plan_grasp(
            point, self.pitches, self.approach_distance)

    # ----------------------------------------------------------- state machine

    def tick(self):
        if self.state in (State.IDLE, State.DONE):
            return
        if (self.state not in (State.LOOK,)
                and self.elapsed() > self.state_timeout):
            self.abandon(f'timed out in {self.state.value}')
            return

        handler = getattr(self, f'_on_{self.state.name.lower()}')
        handler()

    def _on_look(self):
        if not self.remaining:
            self.send_arm(self.look_pose)
            self.get_logger().info('all cubes placed')
            self.enter(State.DONE)
            return

        # Publish once, not every tick: a joint_trajectory_controller restarts
        # the trajectory on each message, so republishing at 10 Hz would keep
        # the arm perpetually 2 s from its goal and arrived() would never hold.
        if self.goal is None or not np.array_equal(self.goal, self.look_pose):
            self.send_arm(self.look_pose)

        wanted = self.remaining[0]
        # Checked before arrived(), so a stuck arm gives up instead of hanging.
        if self.elapsed() > self.state_timeout:
            self.get_logger().warn(f'never saw {wanted}; skipping')
            self.remaining.remove(wanted)
            self.enter(State.LOOK)
            return
        if not self.arrived():
            return
        if any(name == wanted for name, _ in self.detections):
            self.streak += 1
        else:
            self.streak = 0
        if self.streak >= self.stable_frames:
            self.target_color = wanted
            self.enter(State.LOCALIZE)

    def _on_localize(self):
        box = next((b for name, b in self.detections
                    if name == self.target_color), None)
        if box is None:
            self.abandon('lost the target')
            return
        point = self.localize(box)
        if point is None:
            self.abandon('no usable depth')
            return
        point[2] += self.grasp_z_offset
        plan = self.plan_for(point)
        if plan is None:
            self.abandon(f'unreachable at {np.round(point, 3)}')
            return
        self.latched = point
        self.plan = plan
        self.get_logger().info(
            f'{self.target_color} at {np.round(point, 3)} '
            f'pitch {math.degrees(plan.pitch):.0f} deg '
            f'margin {math.degrees(plan.margin):.0f} deg')
        self.send_gripper(self.gripper_open)
        self.send_arm(plan.approach)
        self.enter(State.PRE_GRASP)

    def _on_pre_grasp(self):
        if self.arrived():
            self.send_arm(self.plan.grasp)
            self.enter(State.DESCEND)

    def _on_descend(self):
        if self.arrived():
            self.send_gripper(self.grasp_joint_value)
            self.attach_pubs[self.target_color].publish(Empty())
            self.enter(State.GRASP)

    def _on_grasp(self):
        if self.elapsed() > self.settle_time:
            self.send_arm(self.plan.approach)
            self.enter(State.LIFT)

    def _on_lift(self):
        if self.arrived():
            drop = self.drop_point.copy()
            drop[2] += self.release_z_offset
            plan = self.plan_for(drop)
            if plan is None:
                self.abandon('drop point unreachable')
                return
            self.plan = plan
            self.send_arm(plan.approach)
            self.enter(State.TO_DROP)

    def _on_to_drop(self):
        if self.arrived():
            self.send_arm(self.plan.grasp)
            self.enter(State.LOWER)

    def _on_lower(self):
        if self.arrived():
            self.detach_pubs[self.target_color].publish(Empty())
            self.send_gripper(self.gripper_open)
            self.enter(State.RELEASE)

    def _on_release(self):
        if self.elapsed() > self.settle_time:
            self.send_arm(self.plan.approach)
            self.enter(State.RETREAT)

    def _on_retreat(self):
        if self.arrived():
            self.get_logger().info(f'{self.target_color} placed')
            if self.target_color in self.remaining:
                self.remaining.remove(self.target_color)
            self.target_color = None
            self.enter(State.LOOK)


def _quaternion_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rclpy.init()
    node = PickAndPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 3: Install the script and launch it**

Edit `src/simulations/rospider_gazebo/CMakeLists.txt`:

```cmake
install(PROGRAMS scripts/sim_gait.py scripts/color_detect.py scripts/pick_and_place.py
  DESTINATION lib/${PROJECT_NAME})
```

Edit `src/simulations/rospider_gazebo/launch/pick_place.launch.py`, adding after `detector`:

```python
    picker = Node(
        package='rospider_gazebo',
        executable='pick_and_place.py',
        name='pick_and_place',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'pick_place.yaml'),
                    {'use_sim_time': True,
                     'auto_start': LaunchConfiguration('auto_start')}],
    )
```

and change the timer to `actions=spawns + [detector, picker]`.

- [ ] **Step 4: Verify localisation against ground truth**

Run:

```bash
colcon build --packages-select rospider_gazebo --symlink-install
source install/local_setup.bash
export need_compile=True
ros2 launch rospider_gazebo pick_place.launch.py auto_start:=false
```

In a second shell, trigger one cube and read the latched position from the log:

```bash
source install/local_setup.bash
ros2 service call /pick_and_place/start interfaces/srv/SetString "{data: 'green'}"
```

Expected: the node logs `green at [0.17 0.0 -0.036] pitch 80 deg margin 50 deg` or close to it. Convert to `base_footprint` by adding `0.116091` to z: the result must be within 1 cm of the spawn pose `(0.235, 0.0, 0.105)` less the 0.025 grasp offset, i.e. `(0.235, 0.0, 0.080)`. A larger error means the depth → camera → `base_link` chain is wrong; check `depth_cam_frame` in `ros2 run tf2_ros tf2_echo base_link depth_cam_frame` before touching the offsets.

- [ ] **Step 5: Run the full demo**

```bash
ros2 launch rospider_gazebo pick_place.launch.py
```

Expected: all three cubes end up on the drop marker, each within 3 cm of its centre, with no state timing out. Verify positions with:

```bash
ros2 topic echo /tf --once   # or read the cube poses in the Gazebo entity tree
```

- [ ] **Step 6: Commit**

```bash
git add ROSpider/src/simulations/rospider_gazebo/scripts/pick_and_place.py \
        ROSpider/src/simulations/rospider_gazebo/config/pick_place.yaml \
        ROSpider/src/simulations/rospider_gazebo/CMakeLists.txt \
        ROSpider/src/simulations/rospider_gazebo/launch/pick_place.launch.py
git commit -m "feat(sim): add the pick-and-place state machine

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 7: RViz view and documentation

**Files:**
- Create: `src/simulations/rospider_gazebo/rviz/pick_place.rviz`
- Modify: `src/simulations/rospider_gazebo/launch/pick_place.launch.py`
- Modify: `SIMULATION.md`
- Modify: `../CLAUDE.md` (workspace root)

**Interfaces:**
- Consumes: `/color_detect/image_result` from Task 5, the TF tree, and the robot model.
- Produces: nothing other tasks depend on.

- [ ] **Step 1: Build the RViz config**

Launch the demo, run `rviz2`, add a **RobotModel** display on `/robot_description`, a **TF** display, and an **Image** display on `/color_detect/image_result`, set the fixed frame to `base_footprint`, then save as `src/simulations/rospider_gazebo/rviz/pick_place.rviz`.

- [ ] **Step 2: Start RViz from the launch file**

Edit `src/simulations/rospider_gazebo/launch/pick_place.launch.py`, adding after `picker`:

```python
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', os.path.join(pkg, 'rviz', 'pick_place.rviz')],
        parameters=[{'use_sim_time': True}],
    )
```

and change the timer to `actions=spawns + [detector, picker, rviz]`.

- [ ] **Step 3: Document it in Thai in SIMULATION.md**

Add this section to `SIMULATION.md`, matching the Thai style of the sections already there. Fill the verification list from what you actually observed in Tasks 3-6, not from the spec's predictions.

```markdown
## หยิบและวางวัตถุ (pick and place)

หุ่นยืนอยู่กับที่ มองหาลูกบาศก์สีบนแท่นด้วยกล้อง RGB-D แล้วใช้แขนหยิบไปวางบนป้ายสีเหลือง
ทีละลูก การตรวจจับใช้ OpenCV แยกสีในปริภูมิ HSV และส่งผลออกทาง `/yolo/object_detect`
(`interfaces/ObjectsInfo`) ซึ่งเป็นหัวข้อและชนิดข้อความเดียวกับ `yolo_node.py` ของหุ่นจริง
จึงเปลี่ยนไปใช้ YOLO ภายหลังได้โดยไม่ต้องแก้โค้ดส่วนหยิบ

### รันทั้งชุด

```bash
export need_compile=True
ros2 launch rospider_gazebo pick_place.launch.py
```

### รันทีละสี

```bash
ros2 launch rospider_gazebo pick_place.launch.py auto_start:=false
ros2 service call /pick_and_place/start interfaces/srv/SetString "{data: 'red'}"
ros2 service call /pick_and_place/stop std_srvs/srv/Trigger
```

### ปรับค่า

- ช่วงสี HSV: `config/color_detect.yaml` ดูผลได้จากภาพ `/color_detect/image_result` ใน RViz
- ท่ามอง (`look_pose`), มุมเข้าหยิบ, ตำแหน่งจุดวาง: `config/pick_place.yaml`
- ตำแหน่งแท่นและลูกบาศก์: ตัวแปร `SCENE` ใน `launch/pick_place.launch.py`
  ถ้าย้ายตำแหน่ง ต้องแก้ `LAYOUT` ใน `test/test_arm_ik.py` ให้ตรงกัน แล้วรัน
  `colcon test --packages-select rospider_gazebo` เพื่อยืนยันว่ายังอยู่ในระยะที่แขนเอื้อมถึง

### ตรวจสอบว่าทำงานถูกต้อง

1. แท่นไม่ทับขาหุ่นและไม่ซ้อน `sim_skid_link` ลูกบาศก์วางนิ่งบนแท่น
2. `/color_detect/image_result` วาดกรอบครบทั้งสามสีจากท่ามอง
3. ตำแหน่ง 3 มิติที่ล็อกไว้ (ดูใน log ของ `pick_and_place`) ห่างจากตำแหน่ง spawn จริงไม่เกิน 1 ซม.
4. `/grasp/<สี>/attach` ทำให้ลูกบาศก์ติดมือ และ `detach` ทำให้ตก
5. ลูกบาศก์ทั้งสามลูกไปอยู่บนป้ายวาง ห่างจากจุดกลางไม่เกิน 3 ซม.
```

- [ ] **Step 4: Add it to CLAUDE.md**

In `../CLAUDE.md`, under the Simulation section's entry-point list, add `pick_place` to the `ros2 launch rospider_gazebo {...}.launch.py` line, and add a bullet recording the three decisions a future reader would otherwise re-litigate: MoveIt is not used because `position_only_ik: true` cannot constrain grasp orientation; grasping is a `DetachableJoint` weld on `link5` rather than friction; the scene is spawned at run time so `rospider_room.sdf` and its saved map stay valid.

- [ ] **Step 5: Run the full test suite**

Run: `colcon test --packages-select rospider_gazebo && colcon test-result --verbose`
Expected: PASS — 8 `arm_ik` tests plus the stock lint tests.

- [ ] **Step 6: Commit**

```bash
git add ROSpider/src/simulations/rospider_gazebo/rviz/pick_place.rviz \
        ROSpider/src/simulations/rospider_gazebo/launch/pick_place.launch.py \
        ROSpider/SIMULATION.md CLAUDE.md
git commit -m "docs(sim): document the pick-and-place demo

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

## Notes for the executor

- **Task 3 step 7 and Task 4 step 3 are gates, not formalities.** They test the two things the spec could not settle by reading code: where the feet actually are, and whether `DetachableJoint` welds a cube on spawn. Record what you observe in the commit message even when it matches the prediction.
- **If the layout has to move,** change `SCENE` in `pick_place.launch.py` and `drop_point` in `pick_place.yaml` together, then re-run `colcon test`. `test_scene_layout_is_graspable` reads the spec's layout constants directly, so update `LAYOUT` in `test_arm_ik.py` to match — that test failing after a move is the intended alarm, not a nuisance.
- **`grasp_joint_value: 0.1` is a guess.** The weld does the holding, so tune it purely on looks: the fingers should appear to touch the cube, not pass through it.
- **Do not add `moveit_py`.** `plan_grasp` replaces it, and `ros-jazzy-moveit-py` is not installed on this machine.
