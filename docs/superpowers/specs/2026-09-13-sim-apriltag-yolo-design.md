# AprilTag and YOLO detection (simulation)

Two additions to `src/simulations/rospider_gazebo`, delivered together:

1. **AprilTag detection** — identify a marker and publish its 6-DoF pose.
2. **YOLO detection** — a swappable alternative to `color_detect.py`, with
   tooling to capture data and train new classes.

No upstream Hiwonder package is modified.

## 0. Scope

**In:** the two detector nodes, the assets and tooling they need, a launch
switch to choose a detector, tests, and Thai documentation.

**Out, deliberately:**

- The timed mini-game, the mission manager, docking control, and the
  navigation-mode switch. Deferred to a later round; nothing here blocks it,
  and section 7 records the seams left for it.
- Editing `worlds/rospider_room.sdf` or rebuilding `maps/rospider_room.*`. The
  user designs the world and builds the map. Nothing here hardcodes a
  coordinate into a world file or into Python: tag stations are spawned at run
  time from a config list, and every launch file keeps its `world:=` argument.
- Generating the `DetachableJoint` plugins from a single object list. That is
  about *grasping* a new object, not detecting one, and it is not needed for
  either detector to work. Recorded as a follow-up in section 7.

## 1. Constraints found in the existing code

Load-bearing. Each one changed a decision below.

1. **`interfaces/ApriltagInfo.msg` has no pose field** — only
   `id, x, y, w, d` (`int32`). `interfaces` is a shared upstream package with a
   `mapping_rules.yaml` binding it to ROS 1 for ros1_bridge, so adding a field
   reaches far outside this simulation.
   → Not touched. Pose travels on TF instead.

2. **Upstream's own AprilTag node fills that message with unusable values.**
   `example/example/opencv_example/include/apriltag_recognition.py` builds its
   object points from unit corners (±1), not metres, so the `tvec` written into
   `d` is in units of "half a tag width"; `x, y` hold the pixel position of the
   *drawn text label*, not the tag centre.
   → Our node fills the same fields with honest values: `x, y` = tag centre in
   pixels, `w` = tag width in pixels, `d` = distance in **millimetres**. A
   deliberate deviation, commented in the source. Nothing in this workspace
   consumes upstream's version.

3. **`color_detect.py` already publishes the YOLO contract.** Its docstring
   says so outright: `interfaces/ObjectsInfo` on `/yolo/object_detect`, the
   same topic and message `competition/yolo_node.py` uses on the real robot,
   "so a YOLO node can replace this one without the pick node changing". And
   `pick_and_place.py::_box_centroid_and_area` already handles both the
   4-number axis-aligned box and the 8-number oriented box YOLO emits.
   → `yolo_detect.py` is a peer of `color_detect.py`, not a mode inside it.
   `pick_and_place.py` needs no change at all.

4. **The camera is mounted on the arm** (`depth_cam_link` on `link4`).
   `pick_place.launch.py` passes `arm_pose:=init`, which tilts the camera 52
   degrees down at the floor, and `pick_and_place.py` then drives it to
   `look_pose` for picking.
   → A tag standing on the floor is not in view at either pose. Section 3.2
   sizes and places the station so it is, and section 5 adds an `arm_pose`
   pass-through so a user can look straight ahead.

5. **Camera intrinsics** are fixed by the URDF: `horizontal_fov` 1.2 rad at
   640x480, so `fx = 320 / tan(0.6) = 467.7 px`.
   → Drives the tag size in section 3.2.

6. **Camera topics are bridged lazily** (`gz_bridge.yaml`, `lazy: true`); the
   sensor only renders while something subscribes, so a new subscriber's first
   frame is black.
   → Both detectors must tolerate a black first frame rather than logging it as
   a failure.

7. **`docs/` is git-ignored** (`ROSpider/.gitignore:14`), so this spec is a
   working document and is not committed, matching the previous spec. Anything
   a person cloning the repo must read has to be a tracked file — here,
   `SIMULATION.md`.

## 2. Fresh-clone guarantee

Someone who clones the repo must be able to run both detectors with no extra
downloads beyond one documented `pip install` for the YOLO path.

- The default `detector:=color` needs **zero `pip install`**: clone,
  `colcon build`, launch. Unchanged from today.
- `cv2.aruco` ships in the `opencv-python` that `cv_bridge` already pulls in.
  AprilTag adds **no new rosdep key and no new `package.xml` dependency**.
- Tag PNGs and the station model SDFs are **generated once and committed**.
  `make_tag_textures.py` is for adding ids, not a build step.
- `torch` and `ultralytics` are **not** declared in `package.xml`.
  `yolo_detect.py` imports them lazily and exits with an explanatory message
  naming `requirements-yolo.txt` if they are missing, so `colcon build` and the
  colour path never depend on them.
- A trained model is committed (section 4.3), so `detector:=yolo` works
  immediately for anyone who has installed torch.

## 3. AprilTag

### 3.1 Interfaces

| Topic | Type | Notes |
|---|---|---|
| `~/apriltag_info` | `interfaces/ApriltagsInfo` | Published every frame, `data` empty when nothing is seen, so a consumer can tell "looking, saw nothing" from "node is dead". |
| `~/image_result` | `sensor_msgs/Image` | Overlay: outline, id, axes. Same role as `color_detect.py`'s. |
| `/tf` | `tf2_msgs/TFMessage` | `depth_cam_frame` -> `tag_<id>`, one per tag seen this frame. |

Subscribes `/depth_cam/rgb/image_raw` and `/depth_cam/rgb/camera_info`.
Intrinsics come from `camera_info`, never hardcoded — upstream hardcodes a real
camera's matrix, which is wrong for the simulated one.

`tag_<id>` has z along the tag's outward normal, x right and y down across the
face, which is what `SOLVEPNP_IPPE_SQUARE` returns for the object points in
3.3. Transforms are published only for tags seen this frame, never latched: a
consumer must check the stamp. Holding a stale pose forever would let a future
consumer act on a tag that has left the frame.

### 3.2 Tag size and station geometry

**Tag size 0.15 m.** Apparent width is `467.7 * 0.15 / Z`:

| Distance | Apparent width |
|---|---|
| 0.3 m | 234 px |
| 1.0 m | 70 px |
| 2.0 m | 35 px |

A 36h11 tag is 10 modules across including its border, so 35 px is about
3.5 px per module: enough to read an id, marginal for pose. Height alignment is
forgiving: vertical FOV is `2*atan(240/467.7) = 54.4 deg`, so at 1 m the camera
sees +/- 0.51 m vertically.

Each `tag_station_<id>` model is two boxes in one static link, following the
`models/pick_pedestal` pattern (no mesh, `<static>true</static>`):

- **pedestal** `0.14 x 0.22 x 0.08`, identical to `pick_pedestal`, so its top
  face sits at z = 0.08 and `pick_place.yaml`'s existing `drop_slots` z of
  0.135 would land a cube on it correctly without re-deriving anything. The
  mini-game will need that; nothing in this round depends on it.
- **tag board** `0.16 x 0.01 x 0.30` standing behind the pedestal, textured
  with `tag_<id>.png`, tag centre at z = 0.25.

### 3.3 `scripts/apriltag_detect.py`

Structure mirrors `color_detect.py` so the two read alike. Per frame:

1. Skip if no `camera_info` yet.
2. Grayscale, then `cv2.aruco.ArucoDetector(dict, DetectorParameters())
   .detectMarkers()`.
3. For each detection, `cv2.solvePnPGeneric(objp, corners, K, D,
   flags=cv2.SOLVEPNP_IPPE_SQUARE)` with `objp` the four tag corners in
   **metres** in OpenCV's corner order.

   `solvePnPGeneric`, not `solvePnP`: a planar square has **two** poses that
   reproject almost identically when viewed near-obliquely. `solvePnP` picks
   one arbitrarily and the tag frame visibly flips between frames.
   `SOLVEPNP_IPPE_SQUARE` returns both with reprojection errors; take the
   lower, and drop the detection if it exceeds `max_reproj_error_px`.
4. Broadcast TF; append an `ApriltagInfo` with the constraint-2 semantics.
5. Publish `ApriltagsInfo` (possibly empty) and the overlay.

`config/apriltag.yaml`:

| Name | Default | Why |
|---|---|---|
| `tag_size` | 0.15 | Metres, black square excluding the quiet zone. Must match the texture or every distance is wrong by the ratio. |
| `family` | `tag36h11` | Only 36h11 is supported; the parameter exists so a wrong value fails loudly at startup instead of silently detecting nothing. |
| `max_reproj_error_px` | 3.0 | Above this the ambiguity resolution is untrustworthy. |
| `publish_tf` | true | |
| `image_topic` | `/depth_cam/rgb/image_raw` | |
| `draw` | true | The overlay costs a copy per frame. |

The same file carries the station spawn list the launch file reads, in the
nested-key form `get_parameters_by_prefix` understands — ROS 2 parameters
cannot hold a list of lists, the same reason `pick_place.yaml` stores
`drop_slots` that way:

```yaml
    stations:
      station0: [0.9, 0.0, 3.1416]     # x, y, yaw in the world frame
```

One station by default: enough to verify detection, and the user places more
when they design their world.

### 3.4 `tools/make_tag_textures.py`

Not a ROS node; run by hand, output committed. For each requested id it writes
**both**:

- `worlds/textures/tag_<id>.png` from
  `cv2.aruco.generateImageMarker(getPredefinedDictionary(DICT_APRILTAG_36h11),
  id, N)`, padded with a **white quiet zone** of at least one module on every
  side. Without that border the detector cannot find the tag at all; this is
  the most common way a rendered tag fails.
- `models/tag_station_<id>/model.sdf` from the geometry in 3.2.

One command adds a station. Verified available: OpenCV 5.0.0 on this machine
detects `DICT_APRILTAG_36h11` markers it generated itself.

### 3.5 `rospider_gazebo/tag_geometry.py`

Installed by the existing `ament_python_install_package`, next to `maps.py`.
Holds the facts the node, the model writer and the tests all need, so they
cannot drift: `TAG_SIZE`, the derived object-point array, the board and
pedestal dimensions, and `tag_to_pedestal_top()` returning the pedestal top
centre in the tag frame. The last one is unused this round and exists because
it is the natural home for the number; the mini-game will call it.

## 4. YOLO

### 4.1 `scripts/yolo_detect.py`

The same contract as `color_detect.py`, exactly: `interfaces/ObjectsInfo` on
`/yolo/object_detect`, plus `~/image_result`. Chosen instead of it by a launch
argument; the two never run together.

`config/yolo.yaml`: `model_path` (default: the committed model in 4.3), `conf`,
`task` (`detect` or `obb`), `image_topic`, `classes` (optional allow-list),
`draw`.

`ultralytics` is imported inside `__init__`, not at module scope, and
`ImportError` is caught and re-raised as a message naming
`requirements-yolo.txt`. A missing torch must not look like a broken node.

For `task: obb` the node publishes the 8-number corner box, which
`pick_and_place.py` already understands.

Inference runs on a worker thread reading a `queue.Queue(maxsize=2)`, the
pattern upstream's own `competition/yolo_node.py:35` uses, for the same reason: a slow
frame must drop, not queue up and lag the pick node's view of the world.

### 4.2 `tools/capture_dataset.py` — auto-labelling from ground truth

A ROS node, run by hand against a running simulation, not part of any launch
file.

**Corrected during implementation.** This section originally said the tool
places the objects itself and therefore knows ground truth without reading any
pose back. That is wrong, and testing caught it: the objects are dynamic
bodies. A cube commanded to (0.28, 0.11, 0.105) was measured settling at
(0.299, 0.110, 0.025) — it slid 2 cm and fell off the pedestal onto the floor.
Labels built from the commanded pose were visibly offset from every cube when
drawn back onto the images, which is exactly the silent-poisoning failure this
tool exists to prevent.

The tool therefore places the objects with `set_pose`, waits until two
consecutive reads agree that they have stopped moving, and reads back where
they actually ended up from `/world/<world>/dynamic_pose/info`. The read-back
also supplies a full orientation quaternion rather than a yaw, because a
dropped cube tips onto an edge and a yaw-only model would describe a box the
image does not contain.

The read-back goes through the `gz` CLI, not through `gz_bridge.yaml`, so that
decision survives unchanged: pose/info is a high-rate topic that would compete
with the camera for CPU, a cost `SIMULATION.md` already records here, and this
tool runs offline by hand.

Per sample:

1. Command each object to a random pose within a configured box, chosen to sit
   on the pedestal rather than off its edge.
2. Wait until the objects have settled, then read back their real positions
   and orientations.
3. Wait for a genuinely new frame by comparing `header.stamp`, not by sleeping.
4. Project the object's eight corners from the read-back pose: world ->
   `depth_cam_frame` via TF ->
   pixels via `K` from `camera_info`. The axis-aligned hull of those points is
   the box; clamp to the image and discard if the visible area falls below a
   threshold.
5. **Occlusion check**: compare the depth image at the box centroid with the
   computed distance. A mismatch beyond a tolerance means the object is behind
   something; drop that label. The depth stream is already bridged, so this
   costs nothing extra.
6. Write `images/<n>.jpg` and `labels/<n>.txt`.

Finally split train/val and write `data.yaml`.

### 4.3 `tools/train_yolo.py` and the shipped model

A thin wrapper over `ultralytics`: takes a dataset directory and writes
`models/yolo/<name>.pt`. It does not care whether the dataset came from
`capture_dataset.py` or from a hand-labelled export — Roboflow and labelImg
both emit the same YOLO layout, so there is one code path, not two.

A trained `yolo11n` covering the three cubes is **committed** (about 5-6 MB, in
line with the wall textures already in the repo) so that `detector:=yolo` works
immediately after a clone.

`requirements-yolo.txt` records `ultralytics` and the torch install. The RTX
5060 in use is Blackwell (sm_120) and needs a cu128 or newer wheel; the default
`pip install torch` build will not run on it. This goes in `SIMULATION.md`,
because it is the single most likely thing to stop a student.

## 5. Launch integration

No new top-level launch file. `launch/pick_place.launch.py` gains three
arguments and keeps every existing default, so the command in `SIMULATION.md`
today behaves exactly as it does today:

| Argument | Default | Effect |
|---|---|---|
| `detector` | `color` | `color` -> `color_detect.py`, `yolo` -> `yolo_detect.py`. Never both. |
| `tags` | `true` | Run `apriltag_detect.py` and spawn the stations from `apriltag.yaml`. |
| `arm_pose` | `init` | Passed through to `gazebo.launch.py`. `horizontal` points the camera straight ahead, which is what a wall-standing tag needs (constraint 4). |

`scripts/apriltag_detect.py` and `scripts/yolo_detect.py` must both be added to
`install(PROGRAMS ...)` in `CMakeLists.txt`; a script not listed there is not
installed and the launch file fails with "executable not found".

`tools/` is not installed. These are developer scripts run as
`python3 tools/<name>.py` from the source tree, like the repo's other one-off
utilities.

## 6. Testing

The only existing test in this workspace is `test/test_arm_ik.py`, pure-Python
pytest with no ROS runtime. This work follows that precedent: what can be
tested without a simulator is, and the rest is a written manual procedure that
doubles as the walkthrough in `SIMULATION.md`.

**Automated** — `test/test_apriltag.py`, registered in `CMakeLists.txt`
alongside `test_arm_ik` with the same `APPEND_ENV PYTHONPATH` trick:

1. `test_generated_tag_round_trips` — generate with the tool's own function,
   detect it, assert the id. Guards the quiet zone and the dictionary choice.
2. `test_pose_recovers_known_transform` — project a tag's corners at a known
   pose with a synthetic `K`, solve, assert translation within 1 mm and
   rotation within 0.5 deg.
3. `test_ambiguity_is_resolved` — the same at 60 degrees oblique, asserting the
   correct branch is chosen. Fails if `solvePnP` is ever substituted back.
4. `test_station_model_matches_geometry` — parse each generated
   `models/tag_station_<id>/model.sdf` and assert its boxes equal
   `tag_geometry`'s values, so a hand-edited SDF cannot drift from the
   constants the solver uses.

`yolo_detect.py` gets no unit test: its logic is a thin adaptation of
ultralytics output into a message, and asserting that would require torch,
which section 2 keeps out of the build. The manual procedure covers it.

**Manual, in the simulator**, written into `SIMULATION.md`:

5. `ros2 launch rospider_gazebo pick_place.launch.py arm_pose:=horizontal` —
   confirm in RViz that `tag_0` appears on the board surface, stays put as the
   robot moves, and does not flip orientation. Confirm
   `/apriltag_detect/apriltag_info` reports a plausible `d` in millimetres.
6. `detector:=yolo` — confirm `/yolo/object_detect` carries the same class
   names the colour detector produced, and that `pick_and_place` completes a
   cycle with no change to its configuration.
7. Capture a small dataset, train, point `model_path` at the result, and run
   again — the loop a student will repeat.

## 7. Seams left for the mini-game

Recorded so the next round does not have to rediscover them.

- `pick_and_place.py` already has the drive-between-pick-and-place split:
  `~/pick` parks the cube in a `CARRY` state deliberately exempt from
  `state_timeout`, and `~/place` takes `"x y z"` in `base_footprint` metres and
  answers with a reachability error if the arm cannot reach. A mission manager
  orchestrates through these with no change to that file.
- `sim_gait.py` accepts a fully holonomic twist (`linear.x`, `linear.y`,
  `angular.z`), so docking onto a tag is a 3-DoF proportional controller with
  no nonholonomic constraint.
- `navigation.launch.py` and `vslam.launch.py` each include `gazebo.launch.py`
  themselves, so a mission launch file must include exactly one of them and
  never launch Gazebo directly.
- `navigation.launch.py` leaves `arm_pose` at `init`, pointing the camera at
  the floor, so a mission launch file must force `horizontal` there.
- `maps/**/*.db` is git-ignored and can never be shipped, so a clonable
  `nav:=vslam` has to run RTAB-Map in mapping mode by default.
- `DetachableJoint` binds `child_model` at load time, one plugin per object
  (`urdf/rospider_gazebo.urdf.xacro:66`), so adding a graspable object today
  touches four files. Generating those plugins and the matching bridge entries
  from `pick_place.yaml`'s object list inside `gazebo.launch.py`'s existing
  URDF rewrite is the fix, deferred with the mini-game.

## 8. Risks

- **A committed `.pt` is a binary in git.** About 5-6 MB, comparable to the
  wall textures already committed, and it is what makes `detector:=yolo`
  clonable. Retraining replaces it rather than accumulating versions.
- **`capture_dataset.py` shells out to `gz service`.** It depends on the `gz`
  CLI being on `PATH` and on the world name, which it must read rather than
  assume. Failing to set a pose has to be an error, not a silently mislabelled
  sample.
- **Tag visibility depends on the arm pose**, which `pick_and_place.py` moves
  during a pick. Tags will drop out mid-cycle. Correct behaviour for this
  round — the node reports what it sees — but it is the first thing that will
  look like a bug.
