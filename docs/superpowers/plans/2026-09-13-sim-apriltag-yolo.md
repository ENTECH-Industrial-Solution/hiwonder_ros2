# AprilTag and YOLO Detection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an AprilTag detector that publishes 6-DoF tag poses on TF, and a YOLO detector that is a drop-in replacement for the existing colour detector, with tooling to capture a labelled dataset from the simulator and train new classes.

**Architecture:** Every piece of maths lives in a plain Python module inside the `rospider_gazebo` package and is tested with pytest and no simulator; the `scripts/*.py` files are thin ROS wrappers around it. This is the split the package already uses between `rospider_gazebo/arm_ik.py` and `scripts/pick_and_place.py`. The YOLO node publishes the exact contract `color_detect.py` already publishes, so `pick_and_place.py` is not modified at all.

**Tech Stack:** ROS 2 Jazzy, Python 3.12, OpenCV 5.0 (`cv2.aruco`, already present via `cv_bridge`), Gazebo Harmonic, `ultralytics` + `torch` (optional, YOLO path only).

**Spec:** `ROSpider/docs/superpowers/specs/2026-09-13-sim-apriltag-yolo-design.md`

## Global Constraints

- **Work from `ROSpider/`.** `colcon build` is run there, never from the repo root. All paths below are relative to `ROSpider/src/simulations/rospider_gazebo/` unless stated otherwise.
- **`need_compile=True` must be set** in the environment or launch files fall back to hardcoded `/home/ubuntu/ros2_ws/src` paths.
- **Do not modify any upstream package.** Everything lands in `src/simulations/rospider_gazebo`. `interfaces/` in particular is shared and ros1_bridge-mapped: it is read, never edited.
- **Do not modify `worlds/rospider_room.sdf` or anything under `ROSpider/maps/`.** The user owns the world and the map. Stations are spawned at run time from config.
- **Do not modify `scripts/pick_and_place.py`.** The spec's swappability claim is that it needs no change; a diff to it means something else went wrong.
- **No new `package.xml` dependency.** `cv2.aruco` ships with the `opencv-python` that `cv_bridge` already pulls in. `torch` and `ultralytics` are documented pip installs, imported lazily, never declared.
- **A new script must be added to `install(PROGRAMS ...)` in `CMakeLists.txt`** or the launch file fails with "executable not found".
- **A new test must be registered with `ament_add_pytest_test`** using the same `APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}` as `test_arm_ik`, or `from rospider_gazebo import ...` will not resolve.
- **`tools/` is not installed.** Those are developer scripts run as `python3 tools/<name>.py` from the source tree.
- **Tag size is 0.15 m**, the black square edge excluding the quiet zone.
- **Commit after every task.** Conventional Commits, scope `sim`.

### Build and test commands

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider
source /opt/ros/jazzy/setup.bash
export need_compile=True
colcon build --packages-select rospider_gazebo --symlink-install
source install/local_setup.bash
colcon test --packages-select rospider_gazebo && colcon test-result --verbose
```

Individual pytest during development, without colcon:

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider/src/simulations/rospider_gazebo
PYTHONPATH=. python3 -m pytest test/test_apriltag.py -v
```

---

### Task 1: Tag geometry, generation, detection and pose solving

A pure-Python module. No ROS, no simulator, no Gazebo. Everything in this task runs under plain pytest.

**Files:**
- Create: `rospider_gazebo/tags.py`
- Create: `test/test_apriltag.py`
- Modify: `CMakeLists.txt` (register the new pytest file)

**Interfaces:**
- Consumes: nothing.
- Produces, all imported as `from rospider_gazebo import tags`:
  - `tags.FAMILY: str` — `'tag36h11'`
  - `tags.TAG_SIZE: float` — `0.15`
  - `tags.QUIET_MODULES: int` — `2`
  - `tags.BOARD_FACE: float` — derived, `0.21`
  - `tags.PEDESTAL_SIZE: tuple[float, float, float]` — `(0.14, 0.22, 0.08)`
  - `tags.BOARD_THICKNESS: float` — `0.01`
  - `tags.BOARD_OFFSET_X: float` — `-0.12`
  - `tags.TAG_CENTRE_HEIGHT: float` — `0.25`
  - `tags.POST_SIZE: tuple[float, float, float]` — `(0.03, 0.03, 0.145)`
  - `tags.object_points(size: float = TAG_SIZE) -> np.ndarray` — shape `(4, 3)`, float32
  - `tags.generate_tag_image(tag_id: int, module_px: int = 40) -> np.ndarray` — grayscale uint8, square
  - `tags.detect_tags(gray: np.ndarray) -> list[tuple[int, np.ndarray]]` — `(tag_id, corners)`, corners shape `(4, 2)` float32
  - `tags.solve_tag_pose(corners, camera_matrix, dist_coeffs, size=TAG_SIZE) -> tuple[np.ndarray, np.ndarray, float] | None` — `(rvec, tvec, reproj_error_px)`
  - `tags.tag_to_pedestal_top() -> np.ndarray` — shape `(3,)`, the pedestal top centre expressed in the tag frame

**Note on naming.** The spec calls this module `tag_geometry.py`. It is `tags.py` here because it ends up holding the detection and pose solving too, not only geometry, and a module named for half its contents ages badly. Nothing else about it changes.

**Note on the tag frame.** The spec's section 3.1 says "x right and y down". That is wrong and is corrected here: with the object points below, the tag frame is **x right, y up, z out of the tag face toward the viewer**, which is OpenCV's own marker convention and what `SOLVEPNP_IPPE_SQUARE` requires. Everything downstream uses the corrected convention.

- [ ] **Step 1: Write the failing tests**

Create `test/test_apriltag.py`:

```python
import math

import cv2
import numpy as np

from rospider_gazebo import tags


def _camera_matrix():
    """The simulated depth camera: horizontal_fov 1.2 rad at 640x480, so
    fx = 320 / tan(0.6). See urdf/rospider_gazebo.urdf.xacro line 175."""
    fx = 320.0 / math.tan(0.6)
    return np.array([[fx, 0.0, 320.0],
                     [0.0, fx, 240.0],
                     [0.0, 0.0, 1.0]], dtype=np.float64)


def _project(rvec, tvec, size=tags.TAG_SIZE):
    points, _ = cv2.projectPoints(
        tags.object_points(size), rvec, tvec, _camera_matrix(), np.zeros(5))
    return points.reshape(4, 2).astype(np.float32)


def test_generated_tag_round_trips():
    # The guard against a missing quiet zone or the wrong dictionary: without
    # the white border the detector finds nothing at all, and that failure is
    # silent everywhere else.
    image = tags.generate_tag_image(7)
    found = tags.detect_tags(image)
    assert [tag_id for tag_id, _ in found] == [7]


def test_object_points_are_the_ippe_square_order():
    # SOLVEPNP_IPPE_SQUARE requires exactly this order and orientation:
    # top-left, top-right, bottom-right, bottom-left with y up.
    half = tags.TAG_SIZE / 2.0
    np.testing.assert_allclose(
        tags.object_points(),
        [[-half, half, 0.0], [half, half, 0.0],
         [half, -half, 0.0], [-half, -half, 0.0]],
        atol=1e-9)


def test_pose_recovers_known_transform():
    rvec = np.array([[0.0], [0.3], [0.0]])
    tvec = np.array([[0.02], [0.01], [1.0]])
    solved = tags.solve_tag_pose(
        _project(rvec, tvec), _camera_matrix(), np.zeros(5))
    assert solved is not None
    got_rvec, got_tvec, error = solved
    np.testing.assert_allclose(got_tvec.ravel(), tvec.ravel(), atol=1e-3)
    np.testing.assert_allclose(got_rvec.ravel(), rvec.ravel(), atol=math.radians(0.5))
    assert error < 0.1


def test_ambiguity_is_resolved():
    # A planar square has two poses that reproject almost identically. This
    # fails if solvePnP is ever substituted for solvePnPGeneric: solvePnP
    # returns one of the two arbitrarily and the tag frame flips between
    # frames.
    rvec = np.array([[0.0], [math.radians(60.0)], [0.0]])
    tvec = np.array([[0.02], [0.01], [1.0]])
    solved = tags.solve_tag_pose(
        _project(rvec, tvec), _camera_matrix(), np.zeros(5))
    assert solved is not None
    got_rvec, got_tvec, _ = solved
    np.testing.assert_allclose(got_tvec.ravel(), tvec.ravel(), atol=1e-3)
    np.testing.assert_allclose(got_rvec.ravel(), rvec.ravel(), atol=math.radians(0.5))


def test_board_face_matches_the_generated_texture():
    # The board is sized from the texture, not guessed: the black square must
    # come out at exactly TAG_SIZE once the texture is stretched across the
    # board face, or every distance the solver reports is wrong by the ratio.
    image = tags.generate_tag_image(0)
    module_px = image.shape[0] // (10 + 2 * tags.QUIET_MODULES)
    black_square_px = 10 * module_px
    assert math.isclose(
        tags.BOARD_FACE * black_square_px / image.shape[0],
        tags.TAG_SIZE, abs_tol=1e-9)


def test_pedestal_matches_pick_pedestal():
    # models/pick_pedestal/model.sdf is 0.14 x 0.22 x 0.08, which puts its top
    # face at z = 0.08. Matching it is what lets pick_place.yaml's drop_slots
    # z of 0.135 land a cube on a station without re-deriving anything.
    assert tags.PEDESTAL_SIZE == (0.14, 0.22, 0.08)


def test_tag_to_pedestal_top():
    # Tag frame: x = model +y, y = model +z, z = model +x (the tag faces +x).
    # Pedestal top centre is (0, 0, 0.08) in the model frame; the tag origin is
    # (BOARD_OFFSET_X, 0, TAG_CENTRE_HEIGHT).
    np.testing.assert_allclose(
        tags.tag_to_pedestal_top(),
        [0.0,
         tags.PEDESTAL_SIZE[2] - tags.TAG_CENTRE_HEIGHT,
         -tags.BOARD_OFFSET_X],
        atol=1e-9)
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider/src/simulations/rospider_gazebo
PYTHONPATH=. python3 -m pytest test/test_apriltag.py -v
```

Expected: collection error, `ModuleNotFoundError: No module named 'rospider_gazebo.tags'`.

- [ ] **Step 3: Write the module**

Create `rospider_gazebo/tags.py`:

```python
"""AprilTag geometry, rendering, detection and pose solving, without ROS.

scripts/apriltag_detect.py is a thin ROS wrapper around this module. Keeping
the maths here is what lets test/test_apriltag.py run under plain pytest with
no simulator -- the same split rospider_gazebo/arm_ik.py and
scripts/pick_and_place.py already use.

Tag frame convention: x right, y up, z out of the tag face toward the viewer.
That is OpenCV's own marker convention and what SOLVEPNP_IPPE_SQUARE requires
of the object points below; changing it silently breaks pose solving.
"""

import cv2
import numpy as np

FAMILY = 'tag36h11'
_DICT_ID = cv2.aruco.DICT_APRILTAG_36h11

# A 36h11 marker is 10 modules across including its own black border.
_TAG_MODULES = 10

# Edge of the black square in metres, excluding the quiet zone. Must match the
# physical board through BOARD_FACE below, or every distance solve_tag_pose
# reports is wrong by the ratio between them.
TAG_SIZE = 0.15

# White margin around the marker, in modules. The detector needs light around
# the tag and finds nothing at all without it -- the most common way a rendered
# tag fails, and it fails silently.
QUIET_MODULES = 2

# The board carries the whole texture, marker plus quiet zone, so its face is
# larger than the tag by exactly that ratio. Derived rather than chosen: a
# hand-picked board size would make the black square some other size and put a
# constant scale error into every pose.
BOARD_FACE = TAG_SIZE * (_TAG_MODULES + 2 * QUIET_MODULES) / _TAG_MODULES

# Station model, all in the model frame, which faces +x: a robot approaching
# from +x sees the tag.
#   - pedestal, identical to models/pick_pedestal so its top face is at
#     z = 0.08 and pick_place.yaml's drop_slots z of 0.135 lands a cube on it
#   - a post holding the board up
#   - the board, BOARD_FACE square, behind the pedestal
PEDESTAL_SIZE = (0.14, 0.22, 0.08)
BOARD_THICKNESS = 0.01
BOARD_OFFSET_X = -0.12
TAG_CENTRE_HEIGHT = 0.25
POST_SIZE = (0.03, 0.03, TAG_CENTRE_HEIGHT - BOARD_FACE / 2.0)


def object_points(size=TAG_SIZE):
    """The tag's four corners in the tag frame, metres, float32.

    Order and orientation are fixed by SOLVEPNP_IPPE_SQUARE: top-left,
    top-right, bottom-right, bottom-left, with y up. This is also the order
    cv2.aruco.detectMarkers returns image corners in, so the two line up
    element by element.
    """
    half = size / 2.0
    return np.array([[-half, half, 0.0],
                     [half, half, 0.0],
                     [half, -half, 0.0],
                     [-half, -half, 0.0]], dtype=np.float32)


def generate_tag_image(tag_id, module_px=40):
    """A square tag36h11 image with its white quiet zone, grayscale uint8."""
    tag_px = _TAG_MODULES * module_px
    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(_DICT_ID), tag_id, tag_px)
    pad = QUIET_MODULES * module_px
    image = np.full((tag_px + 2 * pad, tag_px + 2 * pad), 255, np.uint8)
    image[pad:pad + tag_px, pad:pad + tag_px] = marker
    return image


def detect_tags(gray):
    """[(tag_id, corners)] for every tag36h11 in a grayscale image.

    corners is (4, 2) float32 in the same order as object_points().
    """
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(_DICT_ID),
        cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return []
    return [(int(tag_id), corner.reshape(4, 2).astype(np.float32))
            for tag_id, corner in zip(ids.ravel(), corners)]


def solve_tag_pose(corners, camera_matrix, dist_coeffs, size=TAG_SIZE):
    """(rvec, tvec, reprojection error in px) for one tag, or None.

    solvePnPGeneric with IPPE_SQUARE, not solvePnP: a planar square has two
    poses that reproject almost identically when viewed near-obliquely, and
    solvePnP returns one of them arbitrarily -- the published tag frame then
    visibly flips back and forth between frames. IPPE_SQUARE returns both with
    their reprojection errors, so the ambiguity is resolved rather than
    guessed. Measured on a synthetic 60-degree view the two errors are
    1.1e-06 px and 3.27 px, so the choice is not marginal.
    """
    count, rvecs, tvecs, errors = cv2.solvePnPGeneric(
        object_points(size), corners, camera_matrix, dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not count:
        return None
    # Each entry of `errors` is a 1x1 array, not a scalar; float() on it is
    # deprecated in NumPy and will raise in a future release.
    scalar_errors = [float(np.asarray(e).ravel()[0]) for e in errors]
    best = int(np.argmin(scalar_errors))
    return rvecs[best], tvecs[best], scalar_errors[best]


def tag_to_pedestal_top():
    """The pedestal top centre, expressed in the tag frame, metres.

    Unused by the detector; this is the natural home for the number and a
    consumer that wants to place an object on the station's pedestal needs it.

    The model faces +x, so tag x = model +y, tag y = model +z, tag z = model
    +x. The pedestal top centre is (0, 0, PEDESTAL_SIZE[2]) in the model frame
    and the tag origin is (BOARD_OFFSET_X, 0, TAG_CENTRE_HEIGHT).
    """
    return np.array([0.0,
                     PEDESTAL_SIZE[2] - TAG_CENTRE_HEIGHT,
                     -BOARD_OFFSET_X])
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider/src/simulations/rospider_gazebo
PYTHONPATH=. python3 -m pytest test/test_apriltag.py -v
```

Expected: 7 passed.

- [ ] **Step 5: Register the test with colcon**

In `CMakeLists.txt`, directly after the existing `ament_add_pytest_test(test_arm_ik ...)` block, add:

```cmake
  ament_add_pytest_test(test_apriltag test/test_apriltag.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
```

- [ ] **Step 6: Verify through colcon**

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider
source /opt/ros/jazzy/setup.bash && export need_compile=True
colcon build --packages-select rospider_gazebo --symlink-install
colcon test --packages-select rospider_gazebo && colcon test-result --verbose
```

Expected: `test_apriltag` present and passing, `test_arm_ik` still passing.

- [ ] **Step 7: Commit**

```bash
cd ~/entech_hiwonder_ros2_ws
git add ROSpider/src/simulations/rospider_gazebo/rospider_gazebo/tags.py \
        ROSpider/src/simulations/rospider_gazebo/test/test_apriltag.py \
        ROSpider/src/simulations/rospider_gazebo/CMakeLists.txt
git commit -m "feat(sim): add AprilTag geometry, detection and pose solving

Pure-Python module plus pytest, no simulator needed, matching the
arm_ik.py / pick_and_place.py split. solvePnPGeneric with IPPE_SQUARE
resolves the planar-square pose ambiguity that makes solvePnP flip the
tag frame between frames.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: Tag textures and station models

One command that writes both the texture and the model SDF for a station id, so adding a station is one step rather than two files kept in sync by hand.

**Files:**
- Create: `tools/make_tag_textures.py`
- Create (generated, committed): `worlds/textures/tag_0.png`
- Create (generated, committed): `models/tag_station_0/model.sdf`
- Modify: `test/test_apriltag.py` (add the SDF consistency test)

**Interfaces:**
- Consumes: `rospider_gazebo.tags` — `TAG_SIZE`, `BOARD_FACE`, `BOARD_THICKNESS`, `BOARD_OFFSET_X`, `TAG_CENTRE_HEIGHT`, `PEDESTAL_SIZE`, `POST_SIZE`, `generate_tag_image`.
- Produces:
  - `worlds/textures/tag_<id>.png`
  - `models/tag_station_<id>/model.sdf`, a static model named `tag_station_<id>` facing +x
  - `station_sdf(tag_id: int) -> str` importable from the tool for the test

- [ ] **Step 1: Write the failing test**

Append to `test/test_apriltag.py`:

```python
import importlib.util
import pathlib
import xml.etree.ElementTree as ET

_PKG_ROOT = pathlib.Path(__file__).resolve().parent.parent


def _load_tool():
    """tools/ is not an installed package; load the script by path."""
    spec = importlib.util.spec_from_file_location(
        'make_tag_textures', _PKG_ROOT / 'tools' / 'make_tag_textures.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _boxes(sdf_text):
    """{visual or collision name: (sx, sy, sz)} for every box in the model."""
    root = ET.fromstring(sdf_text)
    out = {}
    for element in root.iter():
        if element.tag not in ('visual', 'collision'):
            continue
        size = element.find('.//box/size')
        if size is not None:
            out[element.get('name')] = tuple(
                round(float(v), 6) for v in size.text.split())
    return out


def test_committed_station_sdf_matches_geometry():
    # The SDF on disk is generated, but nothing stops someone editing it by
    # hand. If it drifts from tags.py the solver's distances stay right while
    # the rendered tag is the wrong size -- a failure with no visible symptom
    # except systematically wrong poses.
    committed = (_PKG_ROOT / 'models' / 'tag_station_0' / 'model.sdf').read_text()
    assert committed == _load_tool().station_sdf(0)


def test_station_boxes_match_tags_module():
    boxes = _boxes(_load_tool().station_sdf(0))
    assert boxes['pedestal_v'] == tuple(round(v, 6) for v in tags.PEDESTAL_SIZE)
    assert boxes['board_v'] == (round(tags.BOARD_THICKNESS, 6),
                                round(tags.BOARD_FACE, 6),
                                round(tags.BOARD_FACE, 6))
    assert boxes['post_v'] == tuple(round(v, 6) for v in tags.POST_SIZE)


def test_committed_texture_is_detectable():
    image = cv2.imread(
        str(_PKG_ROOT / 'worlds' / 'textures' / 'tag_0.png'),
        cv2.IMREAD_GRAYSCALE)
    assert image is not None, 'worlds/textures/tag_0.png is missing'
    assert [tag_id for tag_id, _ in tags.detect_tags(image)] == [0]
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider/src/simulations/rospider_gazebo
PYTHONPATH=. python3 -m pytest test/test_apriltag.py -k station -v
```

Expected: FAIL, `FileNotFoundError` on `tools/make_tag_textures.py`.

- [ ] **Step 3: Write the tool**

Create `tools/make_tag_textures.py`:

```python
#!/usr/bin/env python3
"""Write the texture and the station model for one or more AprilTag ids.

Not a ROS node and not installed: run it from the package source tree, and
commit what it produces.

    python3 tools/make_tag_textures.py 0 1 2

Both outputs come from rospider_gazebo/tags.py, so the rendered tag and the
board it is stretched across can never disagree about scale.
"""

import argparse
import os
import sys

import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rospider_gazebo import tags  # noqa: E402  (needs the sys.path above)

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def station_sdf(tag_id):
    """The SDF text for tag_station_<id>.

    Static, box geometry only, no meshes -- the same shape as
    models/pick_pedestal, and for the same reason: gazebo.launch.py strips
    mesh collisions because the robot's own STLs are 1.5M triangles, and
    scenery that needs the same treatment would be a trap.

    The model faces +x: a robot approaching from +x sees the tag.
    """
    px, py, pz = tags.PEDESTAL_SIZE
    ox, oy, oz = tags.POST_SIZE
    face = tags.BOARD_FACE
    board = f'{tags.BOARD_THICKNESS:.6f} {face:.6f} {face:.6f}'
    return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <!-- Generated by tools/make_tag_textures.py. Edit that, not this. -->
  <model name="tag_station_{tag_id}">
    <static>true</static>
    <link name="link">
      <collision name="pedestal_c">
        <pose>0 0 {pz / 2:.6f} 0 0 0</pose>
        <geometry><box><size>{px:.6f} {py:.6f} {pz:.6f}</size></box></geometry>
      </collision>
      <visual name="pedestal_v">
        <pose>0 0 {pz / 2:.6f} 0 0 0</pose>
        <geometry><box><size>{px:.6f} {py:.6f} {pz:.6f}</size></box></geometry>
        <material>
          <ambient>0.6 0.6 0.62 1</ambient>
          <diffuse>0.6 0.6 0.62 1</diffuse>
        </material>
      </visual>
      <collision name="post_c">
        <pose>{tags.BOARD_OFFSET_X:.6f} 0 {oz / 2:.6f} 0 0 0</pose>
        <geometry><box><size>{ox:.6f} {oy:.6f} {oz:.6f}</size></box></geometry>
      </collision>
      <visual name="post_v">
        <pose>{tags.BOARD_OFFSET_X:.6f} 0 {oz / 2:.6f} 0 0 0</pose>
        <geometry><box><size>{ox:.6f} {oy:.6f} {oz:.6f}</size></box></geometry>
        <material>
          <ambient>0.3 0.3 0.32 1</ambient>
          <diffuse>0.3 0.3 0.32 1</diffuse>
        </material>
      </visual>
      <collision name="board_c">
        <pose>{tags.BOARD_OFFSET_X:.6f} 0 {tags.TAG_CENTRE_HEIGHT:.6f} 0 0 0</pose>
        <geometry><box><size>{board}</size></box></geometry>
      </collision>
      <visual name="board_v">
        <pose>{tags.BOARD_OFFSET_X:.6f} 0 {tags.TAG_CENTRE_HEIGHT:.6f} 0 0 0</pose>
        <geometry><box><size>{board}</size></box></geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <pbr><metal>
            <albedo_map>../../worlds/textures/tag_{tag_id}.png</albedo_map>
            <roughness>0.9</roughness>
            <metalness>0</metalness>
          </metal></pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
'''


def write_station(tag_id, module_px):
    texture_dir = os.path.join(PKG_ROOT, 'worlds', 'textures')
    model_dir = os.path.join(PKG_ROOT, 'models', f'tag_station_{tag_id}')
    os.makedirs(model_dir, exist_ok=True)

    texture = os.path.join(texture_dir, f'tag_{tag_id}.png')
    cv2.imwrite(texture, tags.generate_tag_image(tag_id, module_px))

    model = os.path.join(model_dir, 'model.sdf')
    with open(model, 'w') as handle:
        handle.write(station_sdf(tag_id))

    return texture, model


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ids', type=int, nargs='+', help='tag36h11 ids')
    parser.add_argument('--module-px', type=int, default=40,
                        help='pixels per tag module (default 40)')
    args = parser.parse_args()
    for tag_id in args.ids:
        for path in write_station(tag_id, args.module_px):
            print(f'wrote {os.path.relpath(path, PKG_ROOT)}')


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Generate the committed assets**

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider/src/simulations/rospider_gazebo
python3 tools/make_tag_textures.py 0
```

Expected output: `wrote worlds/textures/tag_0.png` and `wrote models/tag_station_0/model.sdf`.

- [ ] **Step 5: Run the tests to verify they pass**

```bash
PYTHONPATH=. python3 -m pytest test/test_apriltag.py -v
```

Expected: 10 passed.

- [ ] **Step 6: Commit**

```bash
cd ~/entech_hiwonder_ros2_ws
git add ROSpider/src/simulations/rospider_gazebo/tools/make_tag_textures.py \
        ROSpider/src/simulations/rospider_gazebo/worlds/textures/tag_0.png \
        ROSpider/src/simulations/rospider_gazebo/models/tag_station_0/ \
        ROSpider/src/simulations/rospider_gazebo/test/test_apriltag.py
git commit -m "feat(sim): generate AprilTag textures and station models

One command writes both the texture and the SDF for a tag id, from the
same constants the solver uses, so the rendered tag and the board it is
stretched across cannot disagree about scale. Assets for tag 0 are
committed so a fresh clone needs no generation step.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: The AprilTag ROS node

**Files:**
- Create: `rospider_gazebo/ros_image.py`
- Create: `scripts/apriltag_detect.py`
- Create: `config/apriltag.yaml`
- Modify: `rospider_gazebo/tags.py` (add `quaternion_from_rvec`)
- Modify: `scripts/color_detect.py:322-341` (move `_to_image_msg` into the new module)
- Modify: `test/test_apriltag.py` (test the quaternion conversion)
- Modify: `CMakeLists.txt`
- Modify: `launch/pick_place.launch.py`
- Modify: `../../../SIMULATION.md` (new section 8)

**Interfaces:**
- Consumes: `rospider_gazebo.tags` — `FAMILY`, `TAG_SIZE`, `detect_tags`, `solve_tag_pose`.
- Produces:
  - `rospider_gazebo.ros_image.to_image_msg(frame: np.ndarray, header) -> sensor_msgs.msg.Image`
  - `rospider_gazebo.tags.quaternion_from_rvec(rvec) -> tuple[float, float, float, float]` as `(x, y, z, w)`
  - node `apriltag_detect`, topics `/apriltag_detect/apriltag_info` and `/apriltag_detect/image_result`, TF frames `tag_<id>` parented to `depth_cam_frame`
  - launch arguments `tags` and `arm_pose` on `pick_place.launch.py`

**Why `ros_image.py` exists.** `color_detect.py:322-341` does not use `cv_bridge.cv2_to_imgmsg`, and its comment explains why: the pip-installed `opencv-python` 5.0 shadows the apt OpenCV that `cv_bridge`'s C++ extension was built against, so `cv2_to_imgmsg` raises `KeyError: 16`. Two more nodes need to publish overlays. Copying that workaround three times would leave three places to get it wrong, so it moves to one module that all three import. `imgmsg_to_cv2` is unaffected and keeps being used directly.

- [ ] **Step 1: Write the failing test for the quaternion conversion**

Append to `test/test_apriltag.py`:

```python
def test_quaternion_from_rvec_matches_the_rotation_matrix():
    # Checked against cv2.Rodrigues rather than a hand-written expected
    # quaternion: the property that matters is that rotating a vector by the
    # quaternion and by the matrix give the same answer.
    for axis_angle in ([0.0, 0.0, 0.0],
                       [0.3, -0.2, 1.1],
                       [math.pi - 1e-6, 0.0, 0.0],
                       [0.0, math.pi - 1e-6, 0.0],
                       [0.0, 0.0, math.pi - 1e-6]):
        rvec = np.array(axis_angle, dtype=np.float64).reshape(3, 1)
        matrix, _ = cv2.Rodrigues(rvec)
        x, y, z, w = tags.quaternion_from_rvec(rvec)
        assert math.isclose(x * x + y * y + z * z + w * w, 1.0, abs_tol=1e-9)
        from_quat = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        np.testing.assert_allclose(from_quat, matrix, atol=1e-9)
```

- [ ] **Step 2: Run it to verify it fails**

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider/src/simulations/rospider_gazebo
PYTHONPATH=. python3 -m pytest test/test_apriltag.py -k quaternion -v
```

Expected: FAIL, `AttributeError: module 'rospider_gazebo.tags' has no attribute 'quaternion_from_rvec'`.

- [ ] **Step 3: Add the conversion to `tags.py`**

Append to `rospider_gazebo/tags.py`:

```python
def quaternion_from_rvec(rvec):
    """Rodrigues rotation vector -> (x, y, z, w), the order ROS uses.

    Written out rather than pulled from tf_transformations, which is not a
    declared dependency of this package and is not needed for four lines of
    algebra. The branch on the trace is not an optimisation: the direct
    formula divides by sqrt(trace + 1), which goes to zero for rotations near
    180 degrees and loses all precision there.
    """
    matrix, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 0.5 / np.sqrt(trace + 1.0)
        return (float((matrix[2, 1] - matrix[1, 2]) * scale),
                float((matrix[0, 2] - matrix[2, 0]) * scale),
                float((matrix[1, 0] - matrix[0, 1]) * scale),
                float(0.25 / scale))
    i = int(np.argmax(np.diag(matrix)))
    j, k = (i + 1) % 3, (i + 2) % 3
    scale = float(np.sqrt(matrix[i, i] - matrix[j, j] - matrix[k, k] + 1.0) * 2.0)
    quaternion = [0.0, 0.0, 0.0, 0.0]
    quaternion[i] = 0.25 * scale
    quaternion[j] = float((matrix[j, i] + matrix[i, j]) / scale)
    quaternion[k] = float((matrix[k, i] + matrix[i, k]) / scale)
    quaternion[3] = float((matrix[k, j] - matrix[j, k]) / scale)
    return tuple(quaternion)
```

- [ ] **Step 4: Run it to verify it passes**

```bash
PYTHONPATH=. python3 -m pytest test/test_apriltag.py -v
```

Expected: 11 passed.

- [ ] **Step 5: Extract the image-message helper**

Create `rospider_gazebo/ros_image.py`:

```python
"""Build a sensor_msgs/Image without cv_bridge.cv2_to_imgmsg.

On this dev machine a pip-installed opencv-python (5.0.0) shadows the apt
OpenCV that cv_bridge's C++ extension was compiled against, so cv2_to_imgmsg
raises "KeyError: 16" while looking up the encoding's numpy dtype from the
wrong module's constants. imgmsg_to_cv2, used for incoming images, is
unaffected.

A contiguous bgr8 or mono8 frame needs no cv_bridge machinery to serialize, so
it is built here instead of depending on the shadowed cv2 import resolving
right. Every node in this package that publishes an overlay uses this.
"""

import numpy as np
from sensor_msgs.msg import Image

_CHANNELS = {'bgr8': 3, 'rgb8': 3, 'mono8': 1}


def to_image_msg(frame, header, encoding='bgr8'):
    """A sensor_msgs/Image carrying `frame`, stamped with `header`."""
    if encoding not in _CHANNELS:
        raise ValueError(f'unsupported encoding {encoding!r}; '
                         f'expected one of {sorted(_CHANNELS)}')
    msg = Image()
    msg.header = header
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = frame.shape[1] * _CHANNELS[encoding]
    msg.data = np.ascontiguousarray(frame).tobytes()
    return msg
```

Then in `scripts/color_detect.py`, delete the whole `_to_image_msg` method (lines 322-341, comment included), add `from rospider_gazebo.ros_image import to_image_msg` to the imports, and replace every `self._to_image_msg(` call with `to_image_msg(`.

- [ ] **Step 6: Verify the extraction changed no behaviour**

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider
colcon build --packages-select rospider_gazebo --symlink-install
source install/local_setup.bash
ros2 launch rospider_gazebo pick_place.launch.py auto_start:=false
```

In a second shell:

```bash
source ~/entech_hiwonder_ros2_ws/ROSpider/install/local_setup.bash
ros2 topic hz /color_detect/image_result
```

Expected: a steady rate, no exception in the launch shell. Then Ctrl-C both.

- [ ] **Step 7: Write the node**

Create `scripts/apriltag_detect.py`:

```python
#!/usr/bin/env python3
"""AprilTag detector for the simulation.

Publishes interfaces/ApriltagsInfo on ~/apriltag_info -- the topic and message
upstream's example/opencv_example/apriltag_recognition.py uses -- and
broadcasts each tag's pose as the TF frame tag_<id>. The TF pose is the useful
output; the message exists so upstream code has something familiar to read.

Two deliberate differences from upstream's node:

  * Intrinsics come from /depth_cam/rgb/camera_info. Upstream hardcodes a real
    camera's matrix, which is simply wrong for the simulated one.
  * The ApriltagInfo fields carry honest values. Upstream builds its object
    points from unit corners rather than metres, so the tvec it writes into
    `d` is in units of half a tag width, and `x, y` hold the pixel position of
    the text label it draws, not the tag centre. Here `x, y` is the tag centre
    in pixels, `w` its pixel width, and `d` the distance in millimetres.
    Nothing in this workspace reads upstream's version, so nothing breaks.

The maths lives in rospider_gazebo/tags.py so it can be tested without a
simulator; this file is only the ROS plumbing.
"""

import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from interfaces.msg import ApriltagInfo, ApriltagsInfo
from rclpy.node import Node
from rospider_gazebo import tags
from rospider_gazebo.ros_image import to_image_msg
from sensor_msgs.msg import CameraInfo, Image


class AprilTagNode(Node):

    def __init__(self):
        super().__init__('apriltag_detect',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.bridge = CvBridge()

        family = str(self._param('family', tags.FAMILY))
        if family != tags.FAMILY:
            # Loudly, at startup. A wrong family otherwise runs forever and
            # detects nothing, which looks identical to a camera problem.
            raise ValueError(
                f'family {family!r} is not supported; this node only handles '
                f'{tags.FAMILY}')

        self.tag_size = float(self._param('tag_size', tags.TAG_SIZE))
        self.max_reproj_error = float(self._param('max_reproj_error_px', 3.0))
        self.publish_tf = bool(self._param('publish_tf', True))
        self.draw = bool(self._param('draw', True))
        image_topic = str(self._param('image_topic',
                                      '/depth_cam/rgb/image_raw'))

        self.camera_matrix = None
        self.dist_coeffs = None

        self.tags_pub = self.create_publisher(ApriltagsInfo,
                                              '~/apriltag_info', 1)
        self.image_pub = self.create_publisher(Image, '~/image_result', 1)
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Image, image_topic, self.image_callback, 1)
        self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info',
                                 self.info_callback, 1)
        self.get_logger().info(
            f'watching for {tags.FAMILY} tags of {self.tag_size} m on '
            f'{image_topic}')

    def _param(self, name, default):
        """Read a parameter, falling back when the YAML does not carry it.

        Parameters are declared from overrides, so anything absent from
        config/apriltag.yaml comes back as None rather than raising. Same
        helper, same reason, as color_detect.py.
        """
        value = self.get_parameter(name).value
        return default if value is None else value

    def info_callback(self, msg):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        if self.dist_coeffs.size == 0:
            self.dist_coeffs = np.zeros(5)

    def image_callback(self, msg):
        try:
            self._process_image(msg)
        except Exception:
            self.get_logger().error(
                'image_callback failed on this frame; skipping it',
                exc_info=True)

    def _process_image(self, msg):
        if self.camera_matrix is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        info = ApriltagsInfo()
        transforms = []
        for tag_id, corners in tags.detect_tags(gray):
            solved = tags.solve_tag_pose(
                corners, self.camera_matrix, self.dist_coeffs, self.tag_size)
            if solved is None:
                continue
            rvec, tvec, error = solved
            if error > self.max_reproj_error:
                # The two IPPE_SQUARE solutions are both poor; whichever won
                # the argmin is not trustworthy. Drop it rather than publish a
                # frame that may be the mirror pose.
                continue

            info.data.append(self._tag_info(tag_id, corners, tvec))
            transforms.append(self._transform(tag_id, rvec, tvec, msg.header))
            if self.draw:
                self._draw(frame, corners, tag_id, rvec, tvec)

        if self.publish_tf and transforms:
            self.broadcaster.sendTransform(transforms)
        # Published even when empty, so a consumer can tell "looking, saw
        # nothing" from "the node is dead".
        self.tags_pub.publish(info)
        if self.draw:
            self.image_pub.publish(to_image_msg(frame, msg.header))

    @staticmethod
    def _tag_info(tag_id, corners, tvec):
        centre = corners.mean(axis=0)
        width = float(np.linalg.norm(corners[1] - corners[0]))
        info = ApriltagInfo()
        info.id = int(tag_id)
        info.x = int(round(float(centre[0])))
        info.y = int(round(float(centre[1])))
        info.w = int(round(width))
        info.d = int(round(float(tvec[2][0]) * 1000.0))   # millimetres
        return info

    def _transform(self, tag_id, rvec, tvec, header):
        transform = TransformStamped()
        transform.header = header          # same stamp and frame as the image
        transform.child_frame_id = f'tag_{tag_id}'
        transform.transform.translation.x = float(tvec[0][0])
        transform.transform.translation.y = float(tvec[1][0])
        transform.transform.translation.z = float(tvec[2][0])
        x, y, z, w = tags.quaternion_from_rvec(rvec)
        transform.transform.rotation.x = x
        transform.transform.rotation.y = y
        transform.transform.rotation.z = z
        transform.transform.rotation.w = w
        return transform

    def _draw(self, frame, corners, tag_id, rvec, tvec):
        cv2.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 2)
        centre = corners.mean(axis=0).astype(int)
        cv2.putText(frame, f'id {tag_id}', (int(centre[0]) - 20, int(centre[1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs,
                          rvec, tvec, self.tag_size * 0.5)


def main():
    rclpy.init()
    node = AprilTagNode()
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

Make it executable: `chmod +x scripts/apriltag_detect.py`.

- [ ] **Step 8: Write the config**

Create `config/apriltag.yaml`:

```yaml
# AprilTag detection for the simulation. The geometry constants live in
# rospider_gazebo/tags.py, which also generates the textures and the station
# models; tag_size here must agree with them or every reported distance is
# wrong by the ratio between the two.
apriltag_detect:
  ros__parameters:
    # Edge of the black square in metres, excluding the white quiet zone.
    tag_size: 0.15

    # Only tag36h11 is supported. The parameter exists so a wrong value stops
    # the node at startup instead of leaving it running and detecting nothing,
    # which looks exactly like a broken camera.
    family: tag36h11

    # A planar square has two poses that reproject almost identically; the
    # node keeps the lower-error one. Above this threshold neither is
    # trustworthy and the detection is dropped. On a synthetic 60-degree view
    # the two errors measure 1.1e-06 px and 3.27 px, so 3.0 sits between a
    # clean solve and a genuinely ambiguous one.
    max_reproj_error_px: 3.0

    publish_tf: true
    image_topic: /depth_cam/rgb/image_raw

    # The overlay costs a frame copy; turn it off for a headless run.
    draw: true

    # Where pick_place.launch.py spawns tag_station_<id> models, in the world
    # frame: [x, y, yaw]. The key names the tag id. The node never reads this;
    # the launch file loads the YAML directly, exactly as it already does for
    # pick_place.yaml's `scene`, so the layout has one source of truth.
    #
    # One station by default: enough to see detection working. Add more when
    # you design your own world, and generate their assets with
    #   python3 tools/make_tag_textures.py 1 2
    stations:
      station0: [0.9, 0.0, 3.14159]
```

- [ ] **Step 9: Install the script**

In `CMakeLists.txt`, extend the existing `install(PROGRAMS ...)` line:

```cmake
install(PROGRAMS scripts/sim_gait.py scripts/color_detect.py scripts/pick_and_place.py
                 scripts/apriltag_detect.py
  DESTINATION lib/${PROJECT_NAME})
```

- [ ] **Step 10: Wire it into the launch file**

In `launch/pick_place.launch.py`:

Read the station list next to where the scene is already read, just after the `SCENE = tuple(scene_params.items())` line:

```python
    # Tag stations come from config/apriltag.yaml, never from the world file:
    # the world and the map belong to the user, so nothing here writes a
    # coordinate into either. [x, y, yaw] in the world frame.
    apriltag_config = os.path.join(pkg, 'config', 'apriltag.yaml')
    with open(apriltag_config) as f:
        station_params = yaml.safe_load(f)['apriltag_detect']['ros__parameters']
    STATIONS = tuple(sorted(station_params.get('stations', {}).items()))
```

Add the station spawns next to the existing `spawns` list:

```python
    station_spawns = [
        Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_tag_{name}',
            output='screen',
            arguments=[
                '-file', os.path.join(
                    pkg, 'models', f'tag_station_{index}', 'model.sdf'),
                '-name', f'tag_station_{index}',
                '-x', str(pose[0]), '-y', str(pose[1]), '-Y', str(pose[2]),
            ],
            condition=IfCondition(LaunchConfiguration('tags')),
        )
        # The id comes from the key, not from the position in the list: a
        # config naming only station2 must still spawn tag_station_2 and its
        # tag_2 texture, not the first model on disk.
        for name, pose in STATIONS
        for index in [int(name.removeprefix('station'))]
    ]
```

Add the node:

```python
    tag_detector = Node(
        package='rospider_gazebo',
        executable='apriltag_detect.py',
        name='apriltag_detect',
        output='screen',
        parameters=[apriltag_config, {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('tags')),
    )
```

Pass `arm_pose` through to the Gazebo include, replacing the hardcoded `'arm_pose': 'init'`:

```python
            'arm_pose': LaunchConfiguration('arm_pose'),
```

Declare the two new arguments alongside the existing ones:

```python
        DeclareLaunchArgument(
            'tags', default_value='true',
            description='run apriltag_detect and spawn the tag stations'),
        DeclareLaunchArgument(
            'arm_pose', default_value='init', choices=['init', 'horizontal'],
            description='init points the camera 52 deg down at the floor, '
                        'which is what picking needs; horizontal points it '
                        'straight ahead, which is what seeing a tag needs'),
```

Add `station_spawns + [tag_detector]` to the `TimerAction` actions list, and add the imports `from launch.conditions import IfCondition` and `from launch.substitutions import LaunchConfiguration` (the latter is already imported).

- [ ] **Step 11: Build and verify in the simulator**

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider
colcon build --packages-select rospider_gazebo --symlink-install
source install/local_setup.bash
ros2 launch rospider_gazebo pick_place.launch.py arm_pose:=horizontal auto_start:=false
```

In a second shell:

```bash
source ~/entech_hiwonder_ros2_ws/ROSpider/install/local_setup.bash
ros2 topic echo /apriltag_detect/apriltag_info --once
ros2 run tf2_ros tf2_echo depth_cam_frame tag_0
```

Expected: `data` carries one entry with `id: 0` and `d` near 900 (millimetres, the station is at x = 0.9), and `tf2_echo` reports a translation whose z is about 0.9 m. In RViz, `tag_0` sits on the board face, stays put as the robot moves, and does not flip.

- [ ] **Step 12: Check the default is unchanged**

```bash
ros2 launch rospider_gazebo pick_place.launch.py
```

Expected: the pick-and-place demo runs exactly as it did before this task, with the tag station visible but out of the camera's view at `arm_pose:=init`.

- [ ] **Step 13: Document it**

Add a section 8 to `ROSpider/SIMULATION.md`, in Thai, after the existing section 7, covering: what the station is and where it is spawned from; the launch command with `arm_pose:=horizontal`; how to read `/apriltag_detect/apriltag_info` and `tag_<id>`; how to add a station with `make_tag_textures.py` plus a `stations:` entry; and the caveat that **the camera rides on the arm, so tags disappear whenever `pick_and_place` moves the arm to its look pose** — the first thing that will look like a bug and is not one.

- [ ] **Step 14: Run the full test suite and commit**

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider
colcon test --packages-select rospider_gazebo && colcon test-result --verbose
cd ~/entech_hiwonder_ros2_ws
git add ROSpider/src/simulations/rospider_gazebo ROSpider/SIMULATION.md
git commit -m "feat(sim): add the AprilTag detection node

Publishes interfaces/ApriltagsInfo on the topic upstream uses and the
real output, each tag's pose, as TF frame tag_<id>. Intrinsics come from
camera_info rather than upstream's hardcoded real-camera matrix, and the
message fields carry honest values rather than upstream's unit-corner
tvec.

pick_place.launch.py gains tags:= and arm_pose:=; every existing default
is unchanged. The sensor_msgs/Image builder moves out of color_detect.py
into rospider_gazebo/ros_image.py so all three overlay publishers share
one copy of the cv_bridge workaround.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: The YOLO ROS node

**Files:**
- Create: `scripts/yolo_detect.py`
- Create: `config/yolo.yaml`
- Create: `requirements-yolo.txt`
- Modify: `CMakeLists.txt`
- Modify: `launch/pick_place.launch.py`
- Modify: `../../../SIMULATION.md` (section 9, usage half)

**Interfaces:**
- Consumes: `rospider_gazebo.ros_image.to_image_msg` from Task 3.
- Produces: node `yolo_detect` publishing `interfaces/ObjectsInfo` on `/yolo/object_detect` and `sensor_msgs/Image` on `/yolo_detect/image_result`; launch argument `detector` on `pick_place.launch.py`.

**The contract is already fixed.** `color_detect.py`'s docstring states it: `interfaces/ObjectsInfo` on `/yolo/object_detect`, the same topic and message `competition/yolo_node.py` uses on the real robot. `pick_and_place.py::_box_centroid_and_area` already handles both the 4-number axis-aligned box and the 8-number oriented box. So this node publishes the same thing and **`pick_and_place.py` is not modified**. A diff to that file means something went wrong.

- [ ] **Step 1: Write the dependency file**

Create `requirements-yolo.txt`:

```
# Optional: only the detector:=yolo path needs these. They are deliberately
# NOT in package.xml -- torch is about 3 GB, and the default detector:=color
# path, colcon build, and every test must all work without it.
#
#   pip install -r requirements-yolo.txt
#
# The GPU on the development machine is an RTX 5060, which is Blackwell
# (sm_120). The default PyPI torch wheel is not built for it and will fail at
# the first CUDA call. Install torch from the cu128 index first:
#
#   pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
#
# CPU-only inference works without any of that, just slower; yolo_detect.py's
# `device` parameter accepts 'cpu'.
ultralytics>=8.3
```

- [ ] **Step 2: Write the node**

Create `scripts/yolo_detect.py`:

```python
#!/usr/bin/env python3
"""YOLO object detector for the simulation.

A drop-in replacement for color_detect.py: same message, same topic, same
image_result. pick_and_place.py needs no change to consume it -- its
_box_centroid_and_area already reads both the 4-number axis-aligned box and
the 8-number oriented box an OBB model produces.

The two detectors are alternatives, never both at once:

    ros2 launch rospider_gazebo pick_place.launch.py detector:=yolo

ultralytics and torch are imported inside __init__, not at module scope, and
are not declared in package.xml. The colour path, colcon build and every test
must all work on a machine that has never installed them, so a missing torch
has to produce one clear sentence rather than an import traceback at launch.
"""

import queue
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from interfaces.msg import ObjectInfo, ObjectsInfo
from rclpy.node import Node
from rospider_gazebo.ros_image import to_image_msg
from sensor_msgs.msg import Image


def _load_yolo(model_path, task):
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError(
            'detector:=yolo needs ultralytics and torch, which this package '
            'deliberately does not declare as dependencies. Install them '
            'with:  pip install -r requirements-yolo.txt  (see that file for '
            'the torch index URL this GPU needs).') from exc
    import logging
    logging.getLogger('ultralytics').setLevel(logging.WARNING)
    return YOLO(model_path, task=task)


class YoloDetectNode(Node):

    def __init__(self):
        super().__init__('yolo_detect',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.bridge = CvBridge()

        model_path = str(self._param('model_path', ''))
        if not model_path:
            raise ValueError('yolo_detect needs a model_path; see '
                             'config/yolo.yaml')
        self.task = str(self._param('task', 'detect'))
        self.conf = float(self._param('conf', 0.5))
        self.device = str(self._param('device', ''))
        self.draw = bool(self._param('draw', True))
        allow = self._param('classes', [])
        self.allow = set(allow) if allow else None
        image_topic = str(self._param('image_topic',
                                      '/depth_cam/rgb/image_raw'))

        self.model = _load_yolo(model_path, self.task)

        # maxsize 2, and a full queue drops the frame rather than blocking.
        # Inference is slower than the 15 Hz camera, and a growing backlog
        # would feed pick_and_place a steadily staler view of the world. Same
        # shape as upstream's competition/yolo_node.py:35.
        self.frames = queue.Queue(maxsize=2)
        self.running = True

        self.objects_pub = self.create_publisher(
            ObjectsInfo, '/yolo/object_detect', 1)
        self.image_pub = self.create_publisher(Image, '~/image_result', 1)
        self.create_subscription(Image, image_topic, self.image_callback, 1)

        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()
        self.get_logger().info(
            f'{model_path} ({self.task}) on {image_topic}, '
            f'classes {sorted(self.model.names.values())}')

    def _param(self, name, default):
        """Read a parameter, falling back when the YAML does not carry it.

        Parameters are declared from overrides, so anything absent from
        config/yolo.yaml comes back as None. Same helper, same reason, as
        color_detect.py.
        """
        value = self.get_parameter(name).value
        return default if value is None else value

    def image_callback(self, msg):
        try:
            self.frames.put_nowait(msg)
        except queue.Full:
            pass

    def _run(self):
        while self.running:
            try:
                msg = self.frames.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._process_image(msg)
            except Exception:
                self.get_logger().error(
                    'inference failed on this frame; skipping it',
                    exc_info=True)

    def _process_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        kwargs = {'conf': self.conf, 'verbose': False}
        if self.device:
            kwargs['device'] = self.device
        result = self.model(frame, **kwargs)[0]

        objects = ObjectsInfo()
        for name, box, score in self._detections(result):
            if self.allow is not None and name not in self.allow:
                continue
            info = ObjectInfo()
            info.class_name = name
            info.box = [int(round(v)) for v in box]
            info.score = float(score)
            info.width = int(frame.shape[1])
            info.height = int(frame.shape[0])
            objects.objects.append(info)
            if self.draw:
                self._draw(frame, name, info.box, score)

        # Published even when empty, so a consumer can tell "looking, saw
        # nothing" from "the node is dead". Matches color_detect.py.
        self.objects_pub.publish(objects)
        if self.draw:
            self.image_pub.publish(to_image_msg(frame, msg.header))

    def _detections(self, result):
        """(class_name, box, score) per detection.

        An OBB model yields the four corners as eight numbers, which is what
        upstream's yolo_node.py publishes and what pick_and_place.py's
        _box_centroid_and_area averages. A plain detect model yields the
        axis-aligned [x1, y1, x2, y2]. Both are legal ObjectInfo.box values.
        """
        obb = getattr(result, 'obb', None)
        if self.task == 'obb' and obb is not None and len(obb):
            for corners, cls, conf in zip(obb.xyxyxyxy.cpu().numpy(),
                                          obb.cls.cpu().numpy(),
                                          obb.conf.cpu().numpy()):
                yield (self.model.names[int(cls)],
                       np.asarray(corners).reshape(-1).tolist(),
                       float(conf))
            return
        if result.boxes is None:
            return
        for xyxy, cls, conf in zip(result.boxes.xyxy.cpu().numpy(),
                                   result.boxes.cls.cpu().numpy(),
                                   result.boxes.conf.cpu().numpy()):
            yield self.model.names[int(cls)], xyxy.tolist(), float(conf)

    @staticmethod
    def _draw(frame, name, box, score):
        points = np.array(box, dtype=np.int32).reshape(-1, 2)
        if len(points) == 2:
            points = np.array([[box[0], box[1]], [box[2], box[1]],
                               [box[2], box[3]], [box[0], box[3]]],
                              dtype=np.int32)
        cv2.polylines(frame, [points], True, (0, 255, 0), 2)
        cv2.putText(frame, f'{name} {score:.2f}',
                    (int(points[:, 0].min()), int(points[:, 1].min()) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    def destroy_node(self):
        self.running = False
        self.worker.join(timeout=1.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = YoloDetectNode()
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

Make it executable: `chmod +x scripts/yolo_detect.py`.

- [ ] **Step 3: Write the config**

Create `config/yolo.yaml`:

```yaml
# YOLO detection for the simulation. Swapped in for color_detect with
#   ros2 launch rospider_gazebo pick_place.launch.py detector:=yolo
# Both publish interfaces/ObjectsInfo on /yolo/object_detect; they are
# alternatives and never run together.
yolo_detect:
  ros__parameters:
    # Trained by tools/train_yolo.py and committed, so a fresh clone can run
    # detector:=yolo without training anything. Replace the file, or point
    # this somewhere else, to use your own model. An absolute path or a name
    # ultralytics can fetch (yolo11n.pt) both work.
    model_path: models/yolo/cubes.pt

    # 'detect' for axis-aligned boxes, 'obb' for oriented ones. Both are legal
    # ObjectInfo.box shapes and pick_and_place.py reads either.
    task: detect

    conf: 0.5

    # '' lets ultralytics choose. 'cpu' forces CPU, which works everywhere and
    # is the fallback when the GPU wheel is wrong for the card.
    device: ''

    # Empty means publish every class the model knows. Narrow it to keep an
    # over-eager model from feeding pick_and_place things it cannot grasp.
    # Names here must match pick_place.yaml's `colors` for a class to be
    # pickable.
    classes: []

    image_topic: /depth_cam/rgb/image_raw
    draw: true
```

- [ ] **Step 4: Install the script and add the launch switch**

In `CMakeLists.txt`, add `scripts/yolo_detect.py` to `install(PROGRAMS ...)`.

In `launch/pick_place.launch.py`, guard the existing `detector` node with a condition and add its YOLO peer. Replace the existing `detector = Node(...)` assignment's `parameters` list unchanged, and add `condition=IfCondition(PythonExpression(["'", LaunchConfiguration('detector'), "' == 'color'"]))`. Then add:

```python
    yolo_detector = Node(
        package='rospider_gazebo',
        executable='yolo_detect.py',
        name='yolo_detect',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'yolo.yaml'),
                    {'use_sim_time': True}],
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('detector'), "' == 'yolo'"])),
    )
```

Declare the argument:

```python
        DeclareLaunchArgument(
            'detector', default_value='color', choices=['color', 'yolo'],
            description='which node publishes /yolo/object_detect'),
```

Add `yolo_detector` to the `TimerAction` actions list, and import `PythonExpression` from `launch.substitutions`.

- [ ] **Step 5: Verify the plumbing with a stock model**

The trained model does not exist until Task 6, so verify the contract against a stock COCO model. Install the dependencies first:

```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
pip install -r ~/entech_hiwonder_ros2_ws/ROSpider/src/simulations/rospider_gazebo/requirements-yolo.txt
```

Then:

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider
colcon build --packages-select rospider_gazebo --symlink-install
source install/local_setup.bash
ros2 launch rospider_gazebo pick_place.launch.py \
    detector:=yolo auto_start:=false tags:=false
```

That fails to find `models/yolo/cubes.pt`. Re-run with an override proving the path is configurable:

```bash
ros2 launch rospider_gazebo pick_place.launch.py \
    detector:=yolo auto_start:=false tags:=false \
    --ros-args -p yolo_detect.model_path:=yolo11n.pt
```

In a second shell:

```bash
ros2 topic echo /yolo/object_detect --once
ros2 node list | grep detect
```

Expected: exactly one detector node (`/yolo_detect`, no `/color_detect`), and an `ObjectsInfo` message, empty or carrying COCO class names. The point is the message shape and the exclusivity of the switch, not the classes.

- [ ] **Step 6: Verify the missing-dependency message**

In a shell with no ultralytics on the path (or temporarily rename the install), launch `detector:=yolo` and confirm the log shows the one-sentence message naming `requirements-yolo.txt` rather than an `ImportError` traceback.

- [ ] **Step 7: Confirm the default path is untouched**

```bash
ros2 launch rospider_gazebo pick_place.launch.py
```

Expected: identical behaviour to before this task — `color_detect` runs, the pick cycle completes.

- [ ] **Step 8: Document the usage half of section 9**

In `ROSpider/SIMULATION.md`, add section 9 in Thai covering: `detector:=color|yolo`, what the two have in common (`/yolo/object_detect`), the pip install including the cu128 index URL and why, `device: cpu` as the fallback, and that class names must appear in `pick_place.yaml`'s `colors` for `pick_and_place` to pick them.

- [ ] **Step 9: Run the tests and commit**

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider
colcon test --packages-select rospider_gazebo && colcon test-result --verbose
cd ~/entech_hiwonder_ros2_ws
git add ROSpider/src/simulations/rospider_gazebo ROSpider/SIMULATION.md
git commit -m "feat(sim): add a YOLO detector alongside the colour one

Publishes the contract color_detect.py already documents, so
pick_and_place.py is unchanged. ultralytics and torch are imported
lazily and are not declared in package.xml: the default colour path,
colcon build and the tests all work without them, and a missing torch
produces one sentence instead of a traceback.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: Dataset capture with automatic labelling

**Files:**
- Create: `rospider_gazebo/labelling.py`
- Create: `test/test_labelling.py`
- Create: `tools/capture_dataset.py`
- Modify: `CMakeLists.txt` (register the new pytest file)

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces, imported as `from rospider_gazebo import labelling`:
  - `labelling.box_corners(size: tuple[float, float, float], pose: tuple[float, float, float], yaw: float = 0.0) -> np.ndarray` — shape `(8, 3)` in the world frame
  - `labelling.transform_points(points: np.ndarray, matrix: np.ndarray) -> np.ndarray` — `matrix` is 4x4
  - `labelling.project_points(points_cam: np.ndarray, camera_matrix: np.ndarray) -> np.ndarray` — shape `(n, 2)`; points with z <= 0 are dropped
  - `labelling.bounding_box(pixels: np.ndarray, width: int, height: int, min_area_px: float) -> tuple[float, float, float, float] | None`
  - `labelling.yolo_line(class_index: int, box, width: int, height: int) -> str`

**Why the maths is a separate module.** The same reason as `tags.py`: this is the part that can be wrong in a way nothing else notices — a mislabelled dataset trains a model that is confidently wrong — and it is the part that can be tested with no simulator running.

- [ ] **Step 1: Write the failing tests**

Create `test/test_labelling.py`:

```python
import math

import numpy as np

from rospider_gazebo import labelling


def _camera_matrix():
    fx = 320.0 / math.tan(0.6)
    return np.array([[fx, 0.0, 320.0],
                     [0.0, fx, 240.0],
                     [0.0, 0.0, 1.0]], dtype=np.float64)


def test_box_corners_are_the_eight_vertices():
    corners = labelling.box_corners((0.05, 0.05, 0.05), (1.0, 2.0, 3.0))
    assert corners.shape == (8, 3)
    np.testing.assert_allclose(corners.min(axis=0), [0.975, 1.975, 2.975])
    np.testing.assert_allclose(corners.max(axis=0), [1.025, 2.025, 3.025])


def test_box_corners_rotate_about_z():
    # A 45-degree yaw widens the footprint of a square by sqrt(2).
    corners = labelling.box_corners(
        (0.10, 0.10, 0.10), (0.0, 0.0, 0.0), yaw=math.radians(45.0))
    assert math.isclose(corners[:, 0].max(), 0.05 * math.sqrt(2), abs_tol=1e-9)
    # Height is unaffected by a yaw.
    assert math.isclose(corners[:, 2].max(), 0.05, abs_tol=1e-9)


def test_project_points_drops_what_is_behind_the_camera():
    # An optical frame has +z forward. A point at z <= 0 has no projection,
    # and projecting it anyway puts a phantom box in the label file.
    points = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, -1.0], [0.0, 0.0, 0.0]])
    pixels = labelling.project_points(points, _camera_matrix())
    assert pixels.shape == (1, 2)
    np.testing.assert_allclose(pixels[0], [320.0, 240.0], atol=1e-9)


def test_bounding_box_clamps_to_the_image():
    pixels = np.array([[-50.0, -20.0], [700.0, 500.0]])
    box = labelling.bounding_box(pixels, 640, 480, min_area_px=1.0)
    assert box == (0.0, 0.0, 640.0, 480.0)


def test_bounding_box_rejects_a_sliver():
    # An object almost entirely out of frame leaves a few pixels at the edge.
    # Labelling that teaches the model that a 2-pixel strip is a cube.
    pixels = np.array([[638.0, 200.0], [645.0, 203.0]])
    assert labelling.bounding_box(pixels, 640, 480, min_area_px=300.0) is None


def test_bounding_box_rejects_an_empty_projection():
    assert labelling.bounding_box(
        np.empty((0, 2)), 640, 480, min_area_px=1.0) is None


def test_yolo_line_is_normalised_centre_and_size():
    line = labelling.yolo_line(2, (160.0, 120.0, 480.0, 360.0), 640, 480)
    assert line == '2 0.500000 0.500000 0.500000 0.500000'


def test_transform_points_applies_rotation_and_translation():
    matrix = np.eye(4)
    matrix[:3, 3] = [1.0, 2.0, 3.0]
    out = labelling.transform_points(np.array([[0.0, 0.0, 0.0]]), matrix)
    np.testing.assert_allclose(out, [[1.0, 2.0, 3.0]])
```

- [ ] **Step 2: Run them to verify they fail**

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider/src/simulations/rospider_gazebo
PYTHONPATH=. python3 -m pytest test/test_labelling.py -v
```

Expected: collection error, `No module named 'rospider_gazebo.labelling'`.

- [ ] **Step 3: Write the module**

Create `rospider_gazebo/labelling.py`:

```python
"""Turn a known object pose into a YOLO label, without ROS.

tools/capture_dataset.py is the ROS wrapper. The maths is here because a
mislabelled dataset is the worst kind of bug in this pipeline: nothing errors,
training succeeds, and the model is confidently wrong. Tested under plain
pytest with no simulator.
"""

import numpy as np

_UNIT_CORNERS = np.array([[sx, sy, sz]
                          for sx in (-0.5, 0.5)
                          for sy in (-0.5, 0.5)
                          for sz in (-0.5, 0.5)], dtype=np.float64)


def box_corners(size, pose, yaw=0.0):
    """The eight vertices of an axis-aligned-then-yawed box, in the world frame.

    `size` is (sx, sy, sz) in metres, `pose` is the box centre.
    """
    corners = _UNIT_CORNERS * np.asarray(size, dtype=np.float64)
    cos, sin = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[cos, -sin, 0.0],
                         [sin, cos, 0.0],
                         [0.0, 0.0, 1.0]])
    return corners @ rotation.T + np.asarray(pose, dtype=np.float64)


def transform_points(points, matrix):
    """Apply a 4x4 homogeneous transform to (n, 3) points."""
    points = np.asarray(points, dtype=np.float64)
    return points @ np.asarray(matrix)[:3, :3].T + np.asarray(matrix)[:3, 3]


def project_points(points_cam, camera_matrix):
    """(n, 2) pixels for the points in front of an optical-frame camera.

    Points at z <= 0 are behind or in the plane of the camera and have no
    projection. Projecting them anyway flips their sign and plants a phantom
    box somewhere in the image, which is exactly the kind of label that
    silently poisons a dataset.
    """
    points_cam = np.asarray(points_cam, dtype=np.float64).reshape(-1, 3)
    points_cam = points_cam[points_cam[:, 2] > 0.0]
    if points_cam.size == 0:
        return np.empty((0, 2))
    projected = points_cam @ np.asarray(camera_matrix).T
    return projected[:, :2] / projected[:, 2:3]


def bounding_box(pixels, width, height, min_area_px):
    """(x1, y1, x2, y2) clamped to the image, or None if too little is visible.

    The threshold matters: an object mostly out of frame leaves a few pixels
    at the edge, and labelling that teaches the model that a thin strip is the
    object.
    """
    pixels = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
    if pixels.size == 0:
        return None
    x1 = float(np.clip(pixels[:, 0].min(), 0.0, width))
    y1 = float(np.clip(pixels[:, 1].min(), 0.0, height))
    x2 = float(np.clip(pixels[:, 0].max(), 0.0, width))
    y2 = float(np.clip(pixels[:, 1].max(), 0.0, height))
    if (x2 - x1) * (y2 - y1) < min_area_px:
        return None
    return x1, y1, x2, y2


def yolo_line(class_index, box, width, height):
    """One line of a YOLO label file: class, centre and size, all normalised."""
    x1, y1, x2, y2 = box
    return (f'{class_index} '
            f'{(x1 + x2) / 2.0 / width:.6f} '
            f'{(y1 + y2) / 2.0 / height:.6f} '
            f'{(x2 - x1) / width:.6f} '
            f'{(y2 - y1) / height:.6f}')
```

- [ ] **Step 4: Run them to verify they pass**

```bash
PYTHONPATH=. python3 -m pytest test/test_labelling.py -v
```

Expected: 8 passed.

- [ ] **Step 5: Register the test**

In `CMakeLists.txt`, next to the other `ament_add_pytest_test` calls:

```cmake
  ament_add_pytest_test(test_labelling test/test_labelling.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
```

- [ ] **Step 6: Check the `gz service` precondition by hand**

The capture tool moves objects by calling Gazebo's own service. Confirm the call works before writing code around it. With a simulation running:

```bash
source ~/entech_hiwonder_ros2_ws/ROSpider/install/local_setup.bash
gz service -s /world/rospider_room/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 \
  --req 'name: "pick_cube_red", position: {x: 0.235, y: 0.07, z: 0.2}'
```

Expected: `data: true`, and the red cube visibly jumps. If `gz` reports it cannot find the command, the environment is not sourced — `gz` lives in the ROS gz vendor prefix and needs `GZ_CONFIG_PATH`, which `install/local_setup.bash` sets.

- [ ] **Step 7: Write the capture tool**

Create `tools/capture_dataset.py`:

```python
#!/usr/bin/env python3
"""Capture a labelled YOLO dataset from a running simulation.

Not installed and not part of any launch file. Start the simulation first,
then run this from the package source tree:

    ros2 launch rospider_gazebo pick_place.launch.py auto_start:=false tags:=false
    python3 tools/capture_dataset.py --samples 400 --out ~/datasets/cubes

The tool PLACES the objects itself, through Gazebo's own set_pose service, so
it already knows ground truth and never reads a pose back. That is why no
/world/.../pose/info entry is added to config/gz_bridge.yaml: it is a
high-rate topic and SIMULATION.md already records what the camera bridge alone
costs in CPU here.

Labels are geometric, not hand-drawn: the object's eight corners are projected
through the live camera_info and the live TF, and the depth image is used to
throw away anything that turns out to be occluded.
"""

import argparse
import json
import os
import random
import subprocess
import sys

import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rospider_gazebo import labelling  # noqa: E402  (needs the sys.path above)

# model name -> (class name, size in metres). The cubes pick_place.launch.py
# spawns; change this to capture something else.
OBJECTS = {
    'pick_cube_red': ('red', (0.05, 0.05, 0.05)),
    'pick_cube_green': ('green', (0.05, 0.05, 0.05)),
    'pick_cube_blue': ('blue', (0.05, 0.05, 0.05)),
}

CAMERA_FRAME = 'depth_cam_frame'
WORLD_FRAME = 'odom'


def set_pose(world, name, position, yaw):
    """Move a model. Raises on failure -- a pose that did not take would
    produce a correctly formatted label for the wrong place, which is worse
    than a crash."""
    request = (f'name: "{name}", position: {{x: {position[0]}, '
               f'y: {position[1]}, z: {position[2]}}}, '
               f'orientation: {{z: {np.sin(yaw / 2):.6f}, '
               f'w: {np.cos(yaw / 2):.6f}}}')
    result = subprocess.run(
        ['gz', 'service', '-s', f'/world/{world}/set_pose',
         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '2000', '--req', request],
        capture_output=True, text=True)
    if result.returncode != 0 or 'true' not in result.stdout:
        raise RuntimeError(
            f'set_pose failed for {name}: {result.stdout}{result.stderr}\n'
            'Is the simulation running, and is install/local_setup.bash '
            'sourced so gz can find its config?')


class CaptureNode(Node):

    def __init__(self, args):
        super().__init__('capture_dataset')
        self.args = args
        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        self.camera_matrix = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Image, '/depth_cam/rgb/image_raw',
                                 self._on_rgb, 1)
        self.create_subscription(Image, '/depth_cam/depth/image_raw',
                                 self._on_depth, 1)
        self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info',
                                 self._on_info, 1)

    def _on_rgb(self, msg):
        self.rgb = (msg.header.stamp, self.bridge.imgmsg_to_cv2(msg, 'bgr8'))

    def _on_depth(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg)

    def _on_info(self, msg):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def wait_for_fresh_frame(self, timeout=5.0):
        """Block until an RGB frame newer than the last one arrives.

        Not a sleep: the camera renders lazily (config/gz_bridge.yaml sets
        lazy: true) and its first frame after a new subscriber attaches is
        black. Sleeping a fixed amount would silently pair a stale image with
        the new poses.
        """
        previous = self.rgb[0] if self.rgb else None
        deadline = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.rgb is not None and self.rgb[0] != previous
                    and self.depth is not None
                    and self.camera_matrix is not None):
                return self.rgb[1]
            if self.get_clock().now().nanoseconds > deadline:
                raise TimeoutError(
                    'no new camera frame; is the simulation running and is '
                    'something subscribed to the camera?')

    def camera_from_world(self):
        tf = self.tf_buffer.lookup_transform(
            CAMERA_FRAME, WORLD_FRAME, rclpy.time.Time())
        t = tf.transform.translation
        r = tf.transform.rotation
        matrix = np.eye(4)
        matrix[:3, :3] = _rotation_matrix(r.x, r.y, r.z, r.w)
        matrix[:3, 3] = [t.x, t.y, t.z]
        return matrix

    def label_for(self, size, pose, yaw, matrix, frame):
        """One YOLO line for this object, or None if it is not usefully visible."""
        corners_cam = labelling.transform_points(
            labelling.box_corners(size, pose, yaw), matrix)
        pixels = labelling.project_points(corners_cam, self.camera_matrix)
        box = labelling.bounding_box(
            pixels, frame.shape[1], frame.shape[0], self.args.min_area_px)
        if box is None:
            return None
        if self.occluded(box, corners_cam):
            return None
        return box

    def occluded(self, box, corners_cam):
        """True when the depth image says something nearer is in the way.

        The projection has no idea what is in front of what. Without this a
        cube behind the pedestal is labelled as if it were visible, and the
        model learns to hallucinate it.
        """
        u = int((box[0] + box[2]) / 2.0)
        v = int((box[1] + box[3]) / 2.0)
        patch = self.depth[max(0, v - 2):v + 3, max(0, u - 2):u + 3]
        patch = patch[np.isfinite(patch) & (patch > 0.0)]
        if patch.size == 0:
            return True
        expected = float(np.min(corners_cam[:, 2]))
        return float(np.median(patch)) < expected - self.args.occlusion_tol


def _rotation_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=int, default=200)
    parser.add_argument('--out', required=True)
    parser.add_argument('--world', default='rospider_room')
    parser.add_argument('--val-split', type=float, default=0.2)
    parser.add_argument('--min-area-px', type=float, default=300.0)
    parser.add_argument('--occlusion-tol', type=float, default=0.03)
    parser.add_argument('--x-range', type=float, nargs=2, default=[0.18, 0.30])
    parser.add_argument('--y-range', type=float, nargs=2, default=[-0.12, 0.12])
    parser.add_argument('--z', type=float, default=0.105)
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args()

    random.seed(args.seed)
    classes = [name for name, _ in OBJECTS.values()]
    root = os.path.expanduser(args.out)
    for split in ('train', 'val'):
        os.makedirs(os.path.join(root, 'images', split), exist_ok=True)
        os.makedirs(os.path.join(root, 'labels', split), exist_ok=True)

    rclpy.init()
    node = CaptureNode(args)
    written = 0
    try:
        for index in range(args.samples):
            placed = {}
            for model, (class_name, size) in OBJECTS.items():
                pose = (random.uniform(*args.x_range),
                        random.uniform(*args.y_range),
                        args.z)
                yaw = random.uniform(-np.pi, np.pi)
                set_pose(args.world, model, pose, yaw)
                placed[model] = (class_name, size, pose, yaw)

            frame = node.wait_for_fresh_frame()
            matrix = node.camera_from_world()

            lines = []
            for class_name, size, pose, yaw in placed.values():
                box = node.label_for(size, pose, yaw, matrix, frame)
                if box is not None:
                    lines.append(labelling.yolo_line(
                        classes.index(class_name), box,
                        frame.shape[1], frame.shape[0]))
            if not lines:
                continue

            split = 'val' if random.random() < args.val_split else 'train'
            stem = f'{index:05d}'
            cv2.imwrite(os.path.join(root, 'images', split, stem + '.jpg'), frame)
            with open(os.path.join(root, 'labels', split, stem + '.txt'), 'w') as f:
                f.write('\n'.join(lines) + '\n')
            written += 1
            if written % 20 == 0:
                print(f'{written} samples')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    with open(os.path.join(root, 'data.yaml'), 'w') as f:
        f.write(f'path: {root}\ntrain: images/train\nval: images/val\n'
                f'names:\n')
        for i, name in enumerate(classes):
            f.write(f'  {i}: {name}\n')
    print(f'wrote {written} samples and data.yaml to {root}')
    print(json.dumps({'classes': classes, 'samples': written}))


if __name__ == '__main__':
    main()
```

- [ ] **Step 8: Capture a small dataset and eyeball the labels**

```bash
# shell 1
ros2 launch rospider_gazebo pick_place.launch.py auto_start:=false tags:=false
# shell 2
cd ~/entech_hiwonder_ros2_ws/ROSpider/src/simulations/rospider_gazebo
python3 tools/capture_dataset.py --samples 20 --out /tmp/cubes_smoke
```

Then draw the labels back onto the images and look at them. A dataset that is subtly wrong looks fine in a text file:

```bash
python3 - <<'EOF'
import glob, cv2
for path in sorted(glob.glob('/tmp/cubes_smoke/images/*/*.jpg'))[:5]:
    img = cv2.imread(path)
    h, w = img.shape[:2]
    for line in open(path.replace('/images/', '/labels/').replace('.jpg', '.txt')):
        c, x, y, bw, bh = line.split()
        x, y, bw, bh = float(x)*w, float(y)*h, float(bw)*w, float(bh)*h
        cv2.rectangle(img, (int(x-bw/2), int(y-bh/2)),
                      (int(x+bw/2), int(y+bh/2)), (0,255,0), 2)
    cv2.imwrite('/tmp/check_' + path.split('/')[-1], img)
    print('/tmp/check_' + path.split('/')[-1])
EOF
```

Expected: every drawn box sits tightly on its cube. A consistent offset means the TF or the projection is wrong; boxes on cubes that are hidden means the occlusion check is not firing.

- [ ] **Step 9: Run the tests and commit**

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider
colcon build --packages-select rospider_gazebo --symlink-install
colcon test --packages-select rospider_gazebo && colcon test-result --verbose
cd ~/entech_hiwonder_ros2_ws
git add ROSpider/src/simulations/rospider_gazebo
git commit -m "feat(sim): capture an auto-labelled YOLO dataset from the sim

The tool places the objects itself through Gazebo's set_pose service, so
it knows ground truth without reading poses back and without adding a
high-rate pose topic to the bridge. Labels come from projecting the
object's corners through the live camera_info and TF, with a depth-based
occlusion check. The projection maths is a separate tested module: a
mislabelled dataset trains a confidently wrong model and nothing errors.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: Training, the shipped model, and the training documentation

**Files:**
- Create: `tools/train_yolo.py`
- Create (binary, committed): `models/yolo/cubes.pt`
- Modify: `../../../SIMULATION.md` (section 9, training half)

**Interfaces:**
- Consumes: a dataset directory from Task 5, or any hand-labelled export in the same YOLO layout.
- Produces: `models/yolo/cubes.pt`, the default `model_path` in `config/yolo.yaml` from Task 4.

- [ ] **Step 1: Write the trainer**

Create `tools/train_yolo.py`:

```python
#!/usr/bin/env python3
"""Train a YOLO model for the simulation's detector.

    python3 tools/train_yolo.py --data ~/datasets/cubes --name cubes

The dataset directory can come from tools/capture_dataset.py or from a
hand-labelled export -- Roboflow and labelImg both emit the same YOLO layout,
so there is one code path rather than two. It needs a data.yaml naming the
classes and the train/val image directories.

The result is written to models/yolo/<name>.pt, which is what
config/yolo.yaml points at by default.
"""

import argparse
import os
import shutil
import sys

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', required=True,
                        help='dataset directory containing data.yaml')
    parser.add_argument('--name', default='cubes')
    parser.add_argument('--base', default='yolo11n.pt',
                        help='starting weights; n is enough for a handful of '
                             'classes and keeps the committed file small')
    parser.add_argument('--epochs', type=int, default=60)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--batch', type=int, default=16)
    parser.add_argument('--device', default='0',
                        help="'0' for the first GPU, 'cpu' to force CPU")
    args = parser.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        sys.exit('needs ultralytics and torch: pip install -r '
                 'requirements-yolo.txt (see that file for the torch index '
                 'URL a Blackwell GPU needs)')

    data = os.path.join(os.path.expanduser(args.data), 'data.yaml')
    if not os.path.exists(data):
        sys.exit(f'{data} not found; capture_dataset.py writes one, and a '
                 'hand-labelled export needs one written by hand')

    result = YOLO(args.base).train(
        data=data, epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
        device=args.device, name=args.name)

    best = os.path.join(result.save_dir, 'weights', 'best.pt')
    out_dir = os.path.join(PKG_ROOT, 'models', 'yolo')
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, f'{args.name}.pt')
    shutil.copy(best, out)
    print(f'wrote {os.path.relpath(out, PKG_ROOT)}')
    print('point config/yolo.yaml model_path at it, then launch with '
          'detector:=yolo')


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Capture a real dataset**

```bash
# shell 1
ros2 launch rospider_gazebo pick_place.launch.py auto_start:=false tags:=false
# shell 2
cd ~/entech_hiwonder_ros2_ws/ROSpider/src/simulations/rospider_gazebo
python3 tools/capture_dataset.py --samples 500 --out ~/datasets/cubes
```

Expected: about 500 samples and a `data.yaml` listing `red`, `green`, `blue`.

- [ ] **Step 3: Train**

```bash
python3 tools/train_yolo.py --data ~/datasets/cubes --name cubes
```

Expected: training completes and `models/yolo/cubes.pt` appears. Check the reported mAP50 for the three classes is above 0.9 — the scene is synthetic and uncluttered, so anything much lower means the labels are wrong, not that the model needs more epochs. If it is low, go back to Task 5 step 8 and look at the drawn boxes again.

- [ ] **Step 4: Verify the trained model end to end**

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider
colcon build --packages-select rospider_gazebo --symlink-install
source install/local_setup.bash
ros2 launch rospider_gazebo pick_place.launch.py detector:=yolo
```

Expected: `/yolo/object_detect` carries `red`, `green` and `blue` with the same class names `color_detect` produced, and the pick-and-place cycle completes with **no change to `pick_place.yaml`** — that is the swappability claim, demonstrated.

- [ ] **Step 5: Check the committed model's size**

```bash
ls -lh ~/entech_hiwonder_ros2_ws/ROSpider/src/simulations/rospider_gazebo/models/yolo/cubes.pt
```

Expected: about 5-6 MB. If it is much larger, a bigger base model was used than `yolo11n`; retrain with `--base yolo11n.pt`. GitHub rejects files over 100 MB and this must stay comfortably small enough to ship.

- [ ] **Step 6: Document the training half of section 9**

In `ROSpider/SIMULATION.md` section 9, in Thai, add: the capture command and what it does (it moves the cubes itself, labels come from geometry, no hand-labelling); how to check the labels by drawing them back; the training command; how to point `config/yolo.yaml` at a new model; how to add a class (edit `OBJECTS` in `capture_dataset.py`, recapture, retrain); and that a hand-labelled Roboflow or labelImg export works with the same `--data` argument.

- [ ] **Step 7: Confirm `pick_and_place.py` was never touched**

```bash
cd ~/entech_hiwonder_ros2_ws
git diff --stat 8fe22a0 -- ROSpider/src/simulations/rospider_gazebo/scripts/pick_and_place.py
```

`8fe22a0` is the commit this work starts from, not `main`: `pick_and_place.py`
does not exist on `main` at all -- it is 709 new lines on this branch, so a
diff against `main` can never be empty and would be a check that always fails.

Expected: an empty diff. A non-empty one means the swappability claim was not
actually met and something was patched around instead.

- [ ] **Step 8: Run everything and commit**

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider
colcon test --packages-select rospider_gazebo && colcon test-result --verbose
cd ~/entech_hiwonder_ros2_ws
git add ROSpider/src/simulations/rospider_gazebo ROSpider/SIMULATION.md
git commit -m "feat(sim): add YOLO training and ship a trained cube model

train_yolo.py takes any YOLO-layout dataset, whether captured from the
simulator or hand-labelled, so there is one code path. The trained
yolo11n is committed (about 5 MB) so detector:=yolo works on a fresh
clone without training anything.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

## Done when

- `colcon test --packages-select rospider_gazebo` passes with `test_arm_ik`, `test_apriltag` and `test_labelling`.
- `ros2 launch rospider_gazebo pick_place.launch.py` behaves exactly as it did before this work.
- `arm_pose:=horizontal` shows `tag_0` in RViz, stable and not flipping.
- `detector:=yolo` completes a pick cycle with no change to `pick_place.yaml` or `pick_and_place.py`.
- `git diff main -- .../scripts/pick_and_place.py` is empty.
- `SIMULATION.md` has sections 8 and 9 in Thai, and someone who has only cloned the repo can follow them.
