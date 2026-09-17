# Sim pick-and-place: colour detection + arm grasp (phase 1)

Date: 2026-09-12
Status: approved design, ready for implementation planning
Scope: `ROSpider/src/simulations/rospider_gazebo`

## 1. Goal

Add an autonomous pick-and-place demo to the ROSpider Gazebo simulation: the robot
sees coloured cubes with its RGB-D camera, picks them up with its 5-DOF arm, and
moves them to a single drop zone. The robot does not walk — every object is placed
inside the arm's reach.

### Success criteria

Running one launch file brings up the simulation with a pedestal, three coloured
cubes and a drop marker, and the robot moves all three cubes onto the drop marker
without human input. Each cube ends up within 3 cm of the drop marker centre.

### Non-goals (phase 2 or later, separate specs)

- Walking to the object (locomotion + visual servoing + odometry drift).
- YOLO / neural detection. The detector interface is chosen so YOLO can replace the
  colour detector later without touching the grasp code, but no YOLO node is written.
- Running on real hardware. Design avoids gratuitous Gazebo coupling but does not
  pay any cost for hardware portability.
- Colour-sorted bins. One drop zone only.

## 2. Constraints discovered in the existing code

Every number below was read out of the repository, not assumed. They drive the design.

### 2.1 The arm is short — 0.29 m of chain

From `rospider_description/urdf/arm.urdf.xacro`, all offsets are pure z-translations
in each joint's own frame (lateral terms are all under 3 mm):

| segment | length (m) | source |
| --- | --- | --- |
| `base_link` → `joint1` | (0.065006, 0, 0.082583) | arm.urdf.xacro:42 |
| `joint1` → `joint2` (shoulder) | 0.043027 | arm.urdf.xacro:86 |
| `joint2` → `joint3` (`L1`) | 0.080000 | arm.urdf.xacro:135 |
| `joint3` → `joint4` (`L2`) | 0.080563 | arm.urdf.xacro:184 |
| `joint4` → `joint5` | 0.049632 | arm.urdf.xacro:233 |
| `joint5` → `end_effector_link` (fixed) | 0.080000 | arm.urdf.xacro:291 |

`joint5` rotates about its own link axis, so it adds no length: the last planar link
is `L3 = 0.049632 + 0.080000 = 0.129632`.

- **Maximum reach from the shoulder: `L1 + L2 + L3 = 0.290195 m`.**
- Shoulder position in `base_link`: `(0.065006, 0, 0.125610)`.
- `base_link` sits `0.116091 m` above `base_footprint` (`base.urdf.xacro:10`), so the
  shoulder is `0.241701 m` above the ground plane.

Reach at a given height `z` (in `base_footprint`), measured horizontally from the
shoulder axis:

```
r_max(z) = sqrt(0.290195² - (0.241701 - z)²)
```

At floor level (`z = 0.04`) this is only **0.209 m**, and reaching it needs the chain
almost fully extended — which is where IK is most fragile and where small errors
matter most. At pedestal height (`z = 0.105`) it is **0.256 m**. This is why objects
go on a pedestal.

**Grasp reach is much shorter than chain reach.** The formula above assumes the arm
may point the hand any way it likes. A grasp fixes the approach direction, which
fixes the last link, so the reachable set shrinks to what the 2R sub-chain
`L1 + L2 = 0.160563` can cover once the wrist centre has been placed. A purely
top-down grasp at pedestal height only reaches `x = 0.170`; a 60° approach reaches
`x = 0.260`.

Two consequences the design is built around:

- Approach direction is a **fallback list**, not a constant. The pick node tries
  progressively shallower approach pitches until IK solves, so near objects get a
  clean top-down grasp and far objects get an angled one.
- The pre-grasp pose backs off **along the tool axis**, not straight up. Hovering
  6 cm vertically above a cube is not reachable at any distance: it would need
  `hw = 0.169 > L1 + L2`.

### 2.2 MoveIt cannot control the grasp orientation

`robot_moveit_config/config/kinematics.yaml` sets `position_only_ik: true` for the
`arm` group. The arm has 5 DOF, so full 6-DOF pose IK is generally unsolvable and
Hiwonder disabled the orientation constraint. `/compute_ik` therefore returns
solutions that reach the requested point with the gripper at an arbitrary pitch —
useless for grasping. Turning orientation back on makes IK fail on most goals.

But the joint axes make this arm trivially solvable in closed form
(`arm.urdf.xacro`, `axis` tags):

| joint | axis | role |
| --- | --- | --- |
| `joint1` | `(0.000404, 0, 1)` ≈ +Z | yaw towards the target |
| `joint2`, `joint3`, `joint4` | `(0, -1, 0)` | **three coplanar pitch joints** |
| `joint5` | `(0, 0, 1)` (along the link) | gripper roll |

One yaw + a planar 3R arm + one roll. This is the same structure Hiwonder's
`arm_kinematics/*.so` solves, and those binaries are aarch64-only so they cannot be
used here anyway (see `CLAUDE.md`).

**Decision: write the IK. Do not use MoveIt in the pick pipeline.** MoveIt stays
available through `moveit.launch.py` for interactive RViz use.

### 2.3 The camera moves with the arm

`depth_camera.urdf.xacro:46` mounts `camera_connect_link` on **`link4`**. Joints 1–4
therefore move the camera. Reaching for a cube swings the camera off it, and the
depth near clip is 0.15 m (`rospider_gazebo.urdf.xacro:166`), so the cube is not
even measurable at grasp range.

**Decision: look-then-move.** Detect from one fixed "look" pose, convert the target
to the static `base_footprint` frame, latch it, and only then move the arm. No
visual servoing during the approach.

### 2.4 Fixed joints below `link5` are lumped away

`gripper_joint` and `end_effector_joint` are both `type="fixed"`
(`arm.urdf.xacro:281`, `arm.urdf.xacro:290`). sdformat merges fixed-joint children
into their parent, so `gripper_link` and `end_effector_link` do not exist as Gazebo
links.

**Anything that needs a Gazebo link on the hand must name `link5`.**

### 2.5 The existing cubes in the world cannot be picked up

`worlds/rospider_room.sdf` already has `red_cube`, `green_cube`, `blue_cube`, but
each is `<static>true</static>` and has a `<visual>` only — no `<collision>`, no
`<inertial>`. They are decoration.

They are also at `x = 0.55`, roughly 0.3 m beyond the arm's reach.

### 2.6 The world file must not change

`maps/rospider_room.yaml` was built by driving through `rospider_room.sdf`, and
`config/nav2_params.yaml` assumes AMCL starts at the spawn pose. Putting a pedestal
0.2 m in front of the spawn point would invalidate the saved map and break the
existing SLAM and Nav2 demos.

**Decision: spawn the pedestal, cubes and drop marker at runtime from the new launch
file via `ros_gz_sim create`. The world file is untouched.**

### 2.7 The robot's only collision geometry is a box at ground level

`gazebo.launch.py:33-35` deletes every `<collision>` that references a mesh, because
the STL collision meshes total ~1.5M triangles and ran the simulation at 2 % real
time. The legs and the body therefore have **no collision geometry at all**. The
only robot collision is `sim_skid_link` (`rospider_gazebo.urdf.xacro:14-30`): a
`0.24 x 0.18 x 0.02` box at `base_footprint`, spanning `x` in [-0.12, 0.12],
`y` in [-0.09, 0.09], `z` in [-0.011, 0.009].

So the pedestal cannot be hit by the legs. Its placement is constrained by:

- **Physics:** it must not intersect the skid box, so its front edge needs `x >= 0.13`.
- **Appearance:** the robot stands still in phase 1 with its front feet near
  `(0.20, +/-0.15)` (leg mounts at `base.urdf.xacro:137,149` plus ~0.23 m of leg), so
  keeping `|y| <= 0.11` avoids visible clipping through a leg.

Foot positions are derived from link offsets, not measured. The first implementation
task measures them from TF in the running simulation. Because nothing collides, a
mismatch is a cosmetic defect, not a physics failure.

## 3. Architecture

Three components with one job each, connected by existing message types.

```
/depth_cam/rgb/image_raw ──▶ color_detect.py ──▶ /yolo/object_detect  (interfaces/ObjectsInfo)
                                     └────────▶ ~/image_result       (annotated, for RViz)
                                                          │
/depth_cam/depth/image_raw ───────────────────┐           │
/depth_cam/depth/camera_info ─────────────────┤           │
                                              ▼           ▼
                                       pick_and_place.py  (state machine)
                                          │ depth → 3D point in depth_cam_frame
                                          │ tf2   → base_footprint, then latched
                                          │ arm_ik.py → five joint angles
                                          ├─▶ /arm_controller/joint_trajectory
                                          ├─▶ /gripper_controller/joint_trajectory
                                          └─▶ /grasp/<colour>/{attach,detach}  (std_msgs/Empty)
```

- **`color_detect.py`** is 2D only. It never sees depth, TF or the arm.
- **`arm_ik.py`** is a pure module: no ROS, no I/O, unit-testable.
- **`pick_and_place.py`** owns all the sequencing and all the frame maths.

Replacing `color_detect.py` with a YOLO node later means matching one published
topic and message type. Nothing else changes.

### 3.1 Why `interfaces/ObjectsInfo` and not a 3D message

`interfaces/msg/ObjectInfo.msg` carries `class_name`, `box`, `score`, `width`,
`height`, `angle` — 2D only. Hiwonder's own `competition/yolo_node.py` publishes
exactly this on `/yolo/object_detect`, and `competition/pick_and_place.py` does the
depth-to-3D projection itself on the consumer side.

Keeping that split means the sim detector is drop-in compatible with the real
robot's YOLO node, and the projection code lives in one place.

## 4. Component specifications

### 4.1 `rospider_gazebo/arm_ik.py` (new Python module)

Installed through the existing `ament_python_install_package(rospider_gazebo)`,
alongside `maps.py`.

Constants, taken from section 2.1, defined at module level.

**`forward_kinematics(q) -> (position, pitch)`**
Exact FK from the URDF numbers — including the small lateral offsets and the
`-0.014429 rad` pitch on `joint3 → joint4` — for `q = (q1..q5)`. Returns the
`end_effector_link` origin in `base_link` and the approach pitch of the hand.

**`inverse_kinematics(target_xyz, pitch, roll=0.0) -> q or None`**

1. `q1 = atan2(y, x)`.
2. Project the target into the `joint1` rotation plane, subtract the shoulder
   offset, and subtract `L3` along the requested approach direction to get the
   wrist centre.
3. Solve the planar 2R problem for `q2`, `q3` (elbow-up branch), then
   `q4 = pitch - q2 - q3` in the idealised model.
4. `q5 = roll`.
5. Refine with a few Gauss-Newton steps against the exact FK of step 1, so the
   ignored millimetre offsets and the 0.83° tilt do not survive into the result.
6. Return `None` if the target is out of reach or any joint leaves its URDF limits
   (`joint1 ±π`, `joint2..joint5 ±2.09`).

Sign conventions follow the URDF axes (`joint2..4` rotate about `-Y`). They are not
asserted in prose; the round-trip test in section 7 is what fixes them.

**`reach_limit(z) -> float`** — the `r_max(z)` formula from section 2.1, so callers
and configs can check a layout without duplicating the constants.

### 4.2 `scripts/color_detect.py` (new node)

| | |
| --- | --- |
| subscribes | `/depth_cam/rgb/image_raw` (`sensor_msgs/Image`) |
| publishes | `/yolo/object_detect` (`interfaces/ObjectsInfo`), `~/image_result` (`sensor_msgs/Image`) |
| params | from `config/color_detect.yaml` |

Per frame: BGR → HSV, one `cv2.inRange` per configured colour, morphological open
then close, `findContours`, discard contours below `min_area_px`, then for each
survivor emit an `ObjectInfo` with `class_name` set to the colour name, `box` as
`[x1, y1, x2, y2]`, `score` 1.0, `width`/`height` of the source image, and `angle`
from `cv2.minAreaRect`.

`~/image_result` is the input frame with boxes and labels drawn, for RViz and for
debugging HSV ranges.

Red wraps around the hue origin, so its config entry holds two ranges that are
OR-ed together.

### 4.3 `scripts/pick_and_place.py` (new node)

State machine, one target at a time:

| state | action | exit |
| --- | --- | --- |
| `IDLE` | wait for start | service call, or `auto_start` param |
| `LOOK` | drive the arm to the look pose; require the same colour present in `stable_frames` consecutive `ObjectsInfo` messages | detection stable → `LOCALIZE`; no detection for `look_timeout` → `DONE` |
| `LOCALIZE` | median depth over a window at the box centroid → camera-frame point via `camera_info` intrinsics → tf2 to `base_footprint` → **latch** | valid point → `PRE_GRASP`; invalid depth → `LOOK`, target skipped |
| `PRE_GRASP` | pick the approach pitch (below), IK to the grasp point backed off by `approach_distance` along the tool axis, gripper open | arrived → `DESCEND` |
| `DESCEND` | IK to the grasp point at the chosen pitch | arrived → `GRASP` |
| `GRASP` | gripper to `grasp_joint_value`, then publish attach for that colour | settle time elapsed → `LIFT` |
| `LIFT` | back to the `PRE_GRASP` pose | arrived → `TO_DROP` |
| `TO_DROP` | drop point backed off by `approach_distance` along its own tool axis | arrived → `LOWER` |
| `LOWER` | IK to the drop release point | arrived → `RELEASE` |
| `RELEASE` | publish detach, gripper open | settle time elapsed → `RETREAT` |
| `RETREAT` | back to the look pose; mark the colour done | arrived → `LOOK` |
| `DONE` | park at the look pose, log a summary | — |

**Choosing the approach pitch.** For each target the node evaluates every pitch in
`approach_pitches_deg` and keeps the one whose grasp and pre-grasp solutions have the
largest joint-limit margin. **Both** poses must solve, because a steep pitch can
reach a grasp point whose pre-grasp pose is above the 2R sub-chain's height ceiling
(section 2.1). If no pitch works the target is unreachable: log it, skip the colour,
return to `LOOK`.

Selecting by margin rather than by steepness matters more than it looks. The elbow
branch, not the pitch, dominates the margin: taking the first valid branch leaves the
cube grasps 0.9 deg from a joint limit, while choosing the branch and pitch by margin
leaves 44-50 deg. `inverse_kinematics` therefore returns the better of the two elbow
branches rather than the first one that validates.

The tool axis for pitch `p` and yaw `q1` is
`(cos p * cos q1, cos p * sin q1, -sin p)`, so backing off moves the hand up and
back towards the shoulder.

Rules that apply to every state:

- Each state has a timeout. On timeout: log, detach everything, open the gripper,
  return to `LOOK`, and add the current colour to a skip list so the demo makes
  progress instead of retrying forever.
- IK returning `None` is a normal outcome, not an error: log the target and reason,
  skip the colour, return to `LOOK`.
- "Arrived" means every commanded joint in `/joint_states` is within
  `joint_tolerance` of its goal. Trajectories are published on the controllers'
  topic interface with an explicit `time_from_start`; no action client is used,
  because `/joint_states` already gives everything the state machine needs.

Services: `~/start` (`interfaces/srv/SetString`; `data` is a colour name, or empty
for "every colour in turn") and `~/stop` (`std_srvs/srv/Trigger`).

On startup the node publishes detach on all three colours once, because Gazebo
creates a `DetachableJoint` as soon as it finds the child model (see 5.2).

### 4.4 Configuration

`config/color_detect.yaml` — HSV bounds per colour (red as two ranges),
`min_area_px`, morphology kernel size, publish rate for `~/image_result`.

`config/pick_place.yaml` — the look pose joint angles, `grasp_joint_value`, the
gripper open value, `joint_tolerance`, per-state timeouts, `stable_frames`, the drop
point in `base_footprint`, and the object layout used to spawn the scene.

Approach geometry, all in the pick node's maths only:

- `approach_pitches_deg` - the candidate list, default `[90, 80, 70, 60, 50, 40, 30]`.
  All are evaluated; the best joint-limit margin wins.
- `approach_distance` - how far the pre-grasp and lift poses back off along the tool
  axis. Default `0.04`. Verified reachable for every pose in section 5.1; `0.05`
  already puts the drop marker's pre-grasp out of reach.
- `grasp_z_offset` - added to the latched detection point's z. The detector sees the
  cube's top face, so this is negative: it drops the hand to the middle of the cube.
- `release_z_offset` - added to the drop point's z. Positive, a small gap so the cube
  falls the last few millimetres instead of being pushed into the pedestal.

The grasp itself is a joint attachment, so an error in these shows up as a visibly
wrong hand position rather than a failed grasp.

Gripper values come from the SRDF group states
(`robot_moveit_config/config/rospider.srdf:44-49`): open `r_joint = 0.5066`,
closed `-0.6288`, within the URDF limits `[-0.66, 0.75]`. Because the grasp is a
joint attachment rather than a friction contact, `grasp_joint_value` is tuned to a
partial close that looks like it is holding a 5 cm cube instead of penetrating it.

## 5. Simulation scene

### 5.1 Models spawned at runtime

New SDF models under `models/`, spawned by `pick_place.launch.py` with
`ros_gz_sim create`. All coordinates are in the world frame with the robot spawned
at the origin, so they equal `base_footprint` coordinates.

| model | geometry | pose | notes |
| --- | --- | --- | --- |
| `pick_pedestal` | box 0.14 x 0.22 x 0.08 | (0.200, 0, 0.04) | static, visual + collision. Spans `x` 0.13-0.27, `|y|` <= 0.11 (section 2.7) |
| `pick_cube_red` | box 0.05 cube | (0.235, +0.07, 0.105) | dynamic, visual + collision + inertial |
| `pick_cube_green` | box 0.05 cube | (0.235, 0, 0.105) | same |
| `pick_cube_blue` | box 0.05 cube | (0.235, -0.07, 0.105) | same |
| `drop_marker` | plate 0.05 x 0.05 x 0.004 | (0.160, 0, 0.082) | static, visual only |

Every model sits inside the pedestal footprint. Clearance between the drop marker
and the nearest cube is 25 mm.

**Reach check.** Chain reach is not the binding constraint (section 2.1), so the
check is the real one: at which pitch from `approach_pitches_deg` do both the grasp
pose and its 0.04 m backed-off pre-grasp pose solve? Computed against the exact
URDF kinematics, with cube centres at `base_link` z = -0.011091:

| target | position (`base_footprint`) | chosen pitch | grasp | pre-grasp / lift |
| --- | --- | --- | --- | --- |
| red cube | (0.235, +0.07, 0.105) | 70 deg | solves | solves |
| green cube | (0.235, 0, 0.105) | 80 deg | solves | solves |
| blue cube | (0.235, -0.07, 0.105) | 70 deg | solves | solves |
| drop point | (0.160, 0, 0.105) | 90 deg | solves | solves |

Worst joint-limit margin over all eight poses: 11.8 deg (the drop point); the six
cube poses are all above 44 deg.

**Camera check.** At the look pose (section 4.4 default: `joint2 = 0.628`,
`joint3 = -1.927`, `joint4 = -1.35`, others 0) the three cubes project to
`u` in [162, 484], `v` ~= 305 of a 640x480 image (`fx = fy = 467.7` from the sensor's
1.2 rad horizontal FOV) at a depth of 0.203 m, clear of the 0.15 m depth near clip.
All three are in frame with margin.

Positions and the look pose are computed, not observed. Implementation task 3
confirms them in the running simulation.

### 5.2 Grasping: `DetachableJoint`

`libgz-sim8-detachable-joint-system.so` is installed and exposes `attach_topic`,
`detach_topic`, `output_topic` and `suppress_child_warning`, so runtime re-attach
works.

Three plugin instances are added to the `<gazebo>` section of
`urdf/rospider_gazebo.urdf.xacro`, one per cube, because `child_model` is fixed at
load time:

```xml
<plugin filename="gz-sim-detachable-joint-system"
        name="gz::sim::systems::DetachableJoint">
  <parent_link>link5</parent_link>
  <child_model>pick_cube_red</child_model>
  <child_link>link</child_link>
  <attach_topic>/grasp/red/attach</attach_topic>
  <detach_topic>/grasp/red/detach</detach_topic>
  <suppress_child_warning>true</suppress_child_warning>
</plugin>
```

`parent_link` is `link5`, not `end_effector_link`, for the reason in section 2.4.
`suppress_child_warning` is required because the cubes are spawned after the robot.

Six new `ROS_TO_GZ` entries in `config/gz_bridge.yaml` map
`/grasp/{red,green,blue}/{attach,detach}` from `std_msgs/msg/Empty` to
`gz.msgs.Empty`.

Two behaviours to confirm during implementation, both with a defined fallback:

1. Gazebo creates the joint as soon as the child model appears. The node's
   detach-on-startup (4.3) is the countermeasure. If a cube is still welded to the
   hand at startup, the fallback is to spawn the cubes before the robot so the
   detach lands after both exist.
2. `DetachableJoint` should create the joint at the current relative pose without
   moving the child. If the cube teleports on attach, the fallback is to position
   the hand so the resulting offset is the intended grasp offset.

### 5.3 `launch/pick_place.launch.py`

Includes `gazebo.launch.py` with `arm_pose:=horizontal`, spawns the five models,
starts `color_detect` and `pick_and_place`, and opens RViz showing `~/image_result`,
the TF tree and the robot model.

Arguments: `gui` and `world` passed through to `gazebo.launch.py`, plus
`auto_start` (default `true`) which makes `pick_and_place` begin without a service
call.

## 6. Files

| file | change |
| --- | --- |
| `rospider_gazebo/arm_ik.py` | new — closed-form IK, exact FK, reach limit |
| `scripts/color_detect.py` | new — HSV detector |
| `scripts/pick_and_place.py` | new — state machine |
| `config/color_detect.yaml` | new |
| `config/pick_place.yaml` | new |
| `models/pick_pedestal/`, `models/pick_cube_{red,green,blue}/`, `models/drop_marker/` | new SDF models |
| `launch/pick_place.launch.py` | new |
| `rviz/pick_place.rviz` | new |
| `test/test_arm_ik.py` | new — unit tests |
| `urdf/rospider_gazebo.urdf.xacro` | edit — three `DetachableJoint` plugins |
| `config/gz_bridge.yaml` | edit — six attach/detach entries |
| `CMakeLists.txt` | edit — install `models/`, the two scripts, register the pytest test |
| `package.xml` | edit — add `interfaces`, `cv_bridge`, `sensor_msgs`, `std_msgs`, `std_srvs`, `tf2_geometry_msgs`, `python3-opencv`, `ament_cmake_pytest` |
| `SIMULATION.md` | edit — Thai how-to section, matching the existing style |
| `CLAUDE.md` (workspace root) | edit — add pick-and-place to the simulation section |

## 7. Testing

### Unit tests — the first functional tests in this repository

`test/test_arm_ik.py`, registered with `ament_add_pytest_test`. Today every package
has only the stock `test_flake8.py` / `test_pep257.py` / `test_copyright.py` lint
tests; `arm_ik.py` is pure maths with no ROS or hardware dependency, so it can be
tested properly.

1. **Round trip.** For a grid of reachable targets covering the pedestal workspace,
   `forward_kinematics(inverse_kinematics(p, pitch))` returns `p` within 1 mm and
   the pitch within 0.5°.
2. **Out of reach returns `None`.** Points beyond `r_max(z)`, and points inside the
   shoulder.
3. **Joint limits respected.** Every returned solution is inside the URDF limits.
4. **`reach_limit` agrees with FK.** At a sampled height, a target at
   `reach_limit(z)` solves and one at `reach_limit(z) + 5 mm` does not.
5. **Scene layout is graspable.** For every cube and the drop point in
   `config/pick_place.yaml`, some pitch in `approach_pitches_deg` solves *both* the
   grasp pose and its backed-off pre-grasp pose. This turns section 5.1's table into
   a test that fails if someone moves an object out of the workspace or shortens the
   pitch list.

Run with `colcon test --packages-select rospider_gazebo && colcon test-result --verbose`.

### Simulation verification

The remaining behaviour is only observable in Gazebo. Each step below is performed
during implementation and written into `SIMULATION.md` so it can be repeated:

1. Measure foot positions from TF; confirm the pedestal does not visibly clip a leg
   and does not intersect `sim_skid_link`, and that the robot stands still with it
   present.
2. Confirm `color_detect` finds all three cubes from the look pose, with
   `~/image_result` as evidence.
3. Confirm the latched 3D position of each cube is within 1 cm of its known spawn
   pose — this validates the whole depth → camera → `base_footprint` chain against
   ground truth.
4. Confirm attach and detach behave as section 5.2 expects, including at startup.
5. Run the full demo and confirm all three cubes reach the drop marker within 3 cm.

## 8. Risks

| risk | mitigation |
| --- | --- |
| Pedestal visibly clips a leg; the computed foot positions in 2.7 are wrong | Measured first (verification step 1); the layout lives in config, not in code. Cosmetic only: the legs have no collision geometry |
| Cube welded to the hand at startup by `DetachableJoint` | Detach-on-startup; fallback is spawn order (5.2) |
| Attach teleports the cube | Fallback in 5.2 |
| Grasp poses fail IK because a constrained approach reaches far less than the bare chain (2.1) | The approach pitch is a fallback list, the pre-grasp backs off along the tool axis, and the layout in 5.1 was chosen by solving both poses for every object; unit test 5 fails loudly if that stops being true |
| Look pose does not see all three cubes | Projected to pixels in 5.1 and confirmed in verification step 2; the look pose is a config value |
| Camera renders black on the first frames — the sensor only renders while something subscribes (`lazy: true` in `gz_bridge.yaml`, see `CLAUDE.md`) | `LOOK` requires `stable_frames` consecutive detections before latching |
| gz_ros2_control mimic-joint handling for the five gripper fingers | Already exercised by the existing simulation; if the fingers do not follow `r_joint`, the grasp is still correct because attachment does not depend on finger contact |
