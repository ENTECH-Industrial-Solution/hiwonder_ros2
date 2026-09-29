# SLAM Mapping Challenge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A 10-15 minute workshop exercise: participants fix three wrong slam_toolbox values in a YAML file, map a new two-room world, and an automatic checker scores the saved map against a reference in three levels.

**Architecture:** Two pure-Python modules hold all logic and are unit tested: `challenge_params.py` (copy, validate and merge the participant's config) and `slam_check.py` (read the map_saver PGM/YAML, align maps in metres, measure the three levels, format a Thai report). A launch file and a CLI script are thin wrappers around them. `slam.launch.py` gains a `params_file` argument so the challenge reuses it unchanged otherwise.

**Tech Stack:** ROS 2 Jazzy, Gazebo Harmonic, slam_toolbox, Python 3.12, numpy, PyYAML, pytest (ament_cmake_pytest).

**Spec:** `docs/superpowers/specs/2026-09-28-slam-challenge-design.md`

## Global Constraints

- Package root: `ROSpider/src/simulations/rospider_gazebo/` (all paths below are relative to it unless they start with `ROSpider/` or `docs/`).
- Build and run from `ROSpider/` only: `source /opt/ros/jazzy/setup.bash`, `colcon build --packages-select rospider_gazebo --symlink-install`, `source install/local_setup.bash`, `export need_compile=True`.
- Room: inside walls x -1.0..4.0, y -2.0..2.0; partition at x = 1.5 with doorway y -0.25..0.25; box 0.4 m square at (0.5, 1.2); pillar radius 0.1 at (2.8, -1.0); robot spawns at (0, 0) yaw 0.
- Starting config: `scan_topic: scan_raw`, `max_laser_range: 1.0`, `resolution: 0.25`, `map_update_interval: 2.0`, `minimum_travel_distance: 0.03`, `minimum_travel_heading: 0.3`. Solved: `scan`, `12.0`, `0.05`, the other three unchanged.
- Participant file: `~/.ros/slam_challenge.yaml`, copied from the template only when missing.
- Levels: (1) >= 1 m² known; (2) >= 80% of reference free cells known; (3) every doorway sample (x = 1.5, y -0.15..0.15) free AND occupied-area ratio <= 2.0.
- Learner-facing text (YAML comments, checker output, launch errors) is Thai. Hints describe symptoms and never name the parameter.
- No cv2 in `slam_check.py` or `challenge_params.py` (launch files import them; cv2 inside `ros2 launch` breaks the Gazebo GUI).
- The user wants ONE commit per finished feature: no per-task commits. Task 6 ends by asking the user to commit.
- Unit tests run from the package root: `python3 -m pytest -q test/<file>.py` (the cwd puts `rospider_gazebo` on the path, as for the existing tests).

## Review Focus

- A misspelled key in the participant's YAML (`max_laser_rang: 12`) must stop the launch naming the key, not be silently ignored by slam_toolbox. Test in Task 2.
- A whole number for a float parameter (`max_laser_range: 12`, `resolution: 1`) must be converted, not crash slam_toolbox with a type mismatch; a string there (`"12"`) must stop the launch. Test in Task 2.
- map_saver_cli writes a `# CREATOR:` comment line in the PGM header; the reader must skip comments. Test in Task 1.
- A map name that does not exist, or a YAML whose `image` file is missing, must give level 1 "fail" and a readable Thai line, never a traceback. Test in Task 1 (`load_map` raises `MapLoadError`) and Task 3 (CLI exit code 1 with message).
- A participant map at a resolution that is not a multiple of the reference's (e.g. 0.07) must still align and be measured in metres. Test in Task 1.

---

### Task 1: `slam_check.py` — map loading, alignment, the three levels, report

**Files:**
- Create: `rospider_gazebo/slam_check.py`
- Create: `test/test_slam_check.py`
- Modify: `CMakeLists.txt` (register the test)

**Interfaces:**
- Produces:
  - constants `FREE = 0`, `OCCUPIED = 100`, `UNKNOWN = -1`, `DOOR_X = 1.5`, `DOOR_Y = (-0.15, 0.15)`, `DOOR_STEP = 0.01`, `MIN_KNOWN_M2 = 1.0`, `MIN_COVERAGE = 0.80`, `MAX_THICKNESS = 2.0`
  - `class MapLoadError(Exception)`
  - `@dataclass GridMap(states: np.ndarray, resolution: float, origin: tuple)` — `states` int8, row 0 = lowest y; methods `states_at(xs, ys) -> np.ndarray`, `cell_centres() -> (X, Y)`, `area(state) -> float`
  - `read_pgm(path) -> np.ndarray` (uint8, row 0 = top of image), `write_pgm(path, pixels) -> None`
  - `from_pixels(pixels, resolution, origin, negate=0, occupied_thresh=0.65, free_thresh=0.196) -> GridMap`, `to_pixels(grid) -> np.ndarray`
  - `load_map(yaml_path) -> GridMap` (raises `MapLoadError`), `save_map(grid, yaml_path) -> None`
  - `coverage(learner, reference) -> float`, `blocked_door_samples(learner) -> int`, `thickness_ratio(learner, reference) -> float`
  - `@dataclass Level(number: int, status: str, numbers: dict)` with status `'pass' | 'fail' | 'skip'`
  - `evaluate(learner: GridMap | None, reference: GridMap) -> list[Level]`, `format_report(levels) -> str`

- [ ] **Step 1: Write the failing tests**

Create `test/test_slam_check.py`:

```python
import numpy as np
import pytest
from rospider_gazebo import slam_check
from rospider_gazebo.slam_check import FREE, OCCUPIED, UNKNOWN, GridMap

RES = 0.05
ORIGIN = (-1.2, -2.2)
W, H = 108, 88          # 5.4 x 4.4 m: the 5 x 4 m room plus a margin


def _room(resolution=RES, origin=ORIGIN, width=W, height=H):
    """The exercise room rasterised: 0.1 m wall bands on the wall lines,
    free inside, unknown outside. Same geometry whatever the grid."""
    xs = origin[0] + (np.arange(width) + 0.5) * resolution
    ys = origin[1] + (np.arange(height) + 0.5) * resolution
    x, y = np.meshgrid(xs, ys)

    def near(d):
        return np.abs(d) < 0.05

    inside = (x > -1.05) & (x < 4.05) & (y > -2.05) & (y < 2.05)
    wall = inside & (near(x + 1.0) | near(x - 4.0) | near(y + 2.0) | near(y - 2.0)
                     | (near(x - 1.5) & (np.abs(y) >= 0.25)))
    states = np.full(x.shape, UNKNOWN, dtype=np.int8)
    states[inside] = FREE
    states[wall] = OCCUPIED
    return GridMap(states, resolution, origin)


def _downsample(grid, factor, start):
    """Coarser cells the way a SLAM grid fills them: a block is occupied if
    any fine cell is, else free if any is free. `start` offsets the blocks."""
    s = grid.states[start[1]:, start[0]:]
    h, w = (s.shape[0] // factor) * factor, (s.shape[1] // factor) * factor
    blocks = s[:h, :w].reshape(h // factor, factor, w // factor, factor).swapaxes(1, 2)
    out = np.full(blocks.shape[:2], UNKNOWN, dtype=np.int8)
    out[(blocks == FREE).any(axis=(2, 3))] = FREE
    out[(blocks == OCCUPIED).any(axis=(2, 3))] = OCCUPIED
    origin = (grid.origin[0] + start[0] * grid.resolution,
              grid.origin[1] + start[1] * grid.resolution)
    return GridMap(out, grid.resolution * factor, origin)


def _statuses(levels):
    return [level.status for level in levels]


def test_exact_copy_passes_every_level():
    reference = _room()
    assert _statuses(slam_check.evaluate(_room(), reference)) == ['pass'] * 3


def test_half_the_room_fails_coverage_only():
    reference = _room()
    learner = _room()
    x, _ = learner.cell_centres()
    learner.states[x > 1.5] = UNKNOWN
    levels = slam_check.evaluate(learner, reference)
    assert levels[0].status == 'pass'
    assert levels[1].status == 'fail'
    assert levels[1].numbers['coverage'] == pytest.approx(0.5, abs=0.05)


def test_coarse_cells_close_the_door_and_thicken_the_walls():
    reference = _room()
    # Blocks start one fine cell in, so block edges sit at y = 0.1 and the
    # block [0.1, 0.35) holds the jamb: the doorway sample at y = 0.15 is
    # inside an occupied block, as it is in a real 0.25 m SLAM map.
    coarse = _downsample(reference, 5, (1, 1))
    levels = slam_check.evaluate(coarse, reference)
    assert levels[1].status == 'pass'            # still explored
    assert levels[2].status == 'fail'
    assert levels[2].numbers['blocked'] > 0
    assert levels[2].numbers['ratio'] > slam_check.MAX_THICKNESS


def test_shifted_origin_same_room_still_passes():
    reference = _room()
    shifted = _room(origin=(-1.213, -2.187))
    assert _statuses(slam_check.evaluate(shifted, reference)) == ['pass'] * 3


def test_other_resolution_is_measured_in_metres():
    reference = _room()
    other = _room(resolution=0.07, origin=(-1.2, -2.2), width=78, height=63)
    levels = slam_check.evaluate(other, reference)
    assert levels[1].numbers['coverage'] > 0.95
    assert levels[2].numbers['blocked'] == 0


def test_no_map_fails_level_one_and_skips_the_rest():
    levels = slam_check.evaluate(None, _room())
    assert _statuses(levels) == ['fail', 'skip', 'skip']


def test_empty_map_fails_level_one():
    empty = _room()
    empty.states[:] = UNKNOWN
    assert _statuses(slam_check.evaluate(empty, _room())) == ['fail', 'skip', 'skip']


def test_map_round_trip(tmp_path):
    grid = _room()
    path = tmp_path / 'room.yaml'
    slam_check.save_map(grid, str(path))
    loaded = slam_check.load_map(str(path))
    assert loaded.resolution == pytest.approx(RES)
    assert loaded.origin == pytest.approx(ORIGIN)
    assert np.array_equal(loaded.states, grid.states)


def test_pgm_header_comment_is_skipped(tmp_path):
    # map_saver_cli writes a CREATOR comment line.
    path = tmp_path / 'm.pgm'
    path.write_bytes(b'P5\n# CREATOR: map_saver.cpp 0.050 m/pix\n3 2\n255\n'
                     + bytes([0, 205, 254, 254, 205, 0]))
    pixels = slam_check.read_pgm(str(path))
    assert pixels.shape == (2, 3)
    assert pixels[0].tolist() == [0, 205, 254]


def test_missing_image_raises_map_load_error(tmp_path):
    path = tmp_path / 'm.yaml'
    path.write_text('image: nowhere.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    with pytest.raises(slam_check.MapLoadError):
        slam_check.load_map(str(path))


def test_missing_yaml_raises_map_load_error(tmp_path):
    with pytest.raises(slam_check.MapLoadError):
        slam_check.load_map(str(tmp_path / 'nope.yaml'))


def test_report_hints_name_symptoms_not_parameters():
    reference = _room()
    text = slam_check.format_report(slam_check.evaluate(None, reference))
    assert 'ros2 topic list' in text
    coarse = _downsample(reference, 5, (1, 1))
    text = slam_check.format_report(slam_check.evaluate(coarse, reference))
    for parameter in ('resolution', 'max_laser_range', 'scan_topic'):
        assert parameter not in text
    assert 'ผ่านครบทุกด่าน' in slam_check.format_report(
        slam_check.evaluate(_room(), reference))
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd ROSpider/src/simulations/rospider_gazebo && python3 -m pytest -q test/test_slam_check.py`
Expected: collection error, `ImportError: cannot import name 'slam_check'`.

- [ ] **Step 3: Write the implementation**

Create `rospider_gazebo/slam_check.py`:

```python
"""Score a saved SLAM map against the SLAM exercise's reference map.

Pure Python + numpy (no cv2, no ROS), so it is unit tested and importable
anywhere. The two maps may differ in resolution and origin, so every
comparison is made in metres: a point is looked up in a map through that
map's own origin and resolution.

Levels (docs/superpowers/specs/2026-09-28-slam-challenge-design.md):
  1. a map exists: at least MIN_KNOWN_M2 of known cells;
  2. coverage: MIN_COVERAGE of the reference's free cells are known;
  3. detail: the doorway is free along its centre line and the occupied
     area is at most MAX_THICKNESS times the reference's.
"""

import os
from dataclasses import dataclass, field

import numpy as np
import yaml

FREE, OCCUPIED, UNKNOWN = 0, 100, -1

# worlds/slam_challenge.sdf: the partition is centred on x = 1.5 and the
# doorway is the gap y -0.25..0.25. Samples stay 0.10 m clear of the jambs.
DOOR_X = 1.5
DOOR_Y = (-0.15, 0.15)
DOOR_STEP = 0.01

MIN_KNOWN_M2 = 1.0
MIN_COVERAGE = 0.80
MAX_THICKNESS = 2.0

# map_saver trinary values
_PIXEL = {FREE: 254, OCCUPIED: 0, UNKNOWN: 205}


class MapLoadError(Exception):
    """The map's YAML or image could not be read."""


@dataclass
class GridMap:
    states: np.ndarray      # int8 FREE/OCCUPIED/UNKNOWN, row 0 = lowest y
    resolution: float       # metres per cell
    origin: tuple           # (x, y) of the lower-left corner of cell (0, 0)

    def states_at(self, xs, ys):
        """States at world points; UNKNOWN outside the grid."""
        cols = np.floor((np.asarray(xs, float) - self.origin[0]) / self.resolution).astype(int)
        rows = np.floor((np.asarray(ys, float) - self.origin[1]) / self.resolution).astype(int)
        height, width = self.states.shape
        inside = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
        out = np.full(cols.shape, UNKNOWN, dtype=np.int8)
        out[inside] = self.states[rows[inside], cols[inside]]
        return out

    def cell_centres(self):
        """(X, Y) arrays, shaped like `states`, of every cell's centre."""
        height, width = self.states.shape
        xs = self.origin[0] + (np.arange(width) + 0.5) * self.resolution
        ys = self.origin[1] + (np.arange(height) + 0.5) * self.resolution
        return np.meshgrid(xs, ys)

    def area(self, state):
        return float(np.count_nonzero(self.states == state)) * self.resolution ** 2


def read_pgm(path):
    """A binary (P5) 8-bit PGM as a uint8 array, row 0 = top of the image."""
    try:
        with open(path, 'rb') as handle:
            data = handle.read()
    except OSError as err:
        raise MapLoadError(f'{path}: {err}') from err
    tokens, pos = [], 0
    while len(tokens) < 4:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            end = data.find(b'\n', pos)
            pos = len(data) if end < 0 else end + 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        if start == pos:
            raise MapLoadError(f'{path}: truncated PGM header')
        tokens.append(data[start:pos])
    if tokens[0] != b'P5' or int(tokens[3]) > 255:
        raise MapLoadError(f'{path}: not an 8-bit binary PGM (P5)')
    width, height = int(tokens[1]), int(tokens[2])
    pos += 1                            # the single whitespace after maxval
    if len(data) - pos < width * height:
        raise MapLoadError(f'{path}: image data is shorter than {width}x{height}')
    return np.frombuffer(data, dtype=np.uint8, count=width * height,
                         offset=pos).reshape(height, width)


def write_pgm(path, pixels):
    height, width = pixels.shape
    with open(path, 'wb') as handle:
        handle.write(f'P5\n{width} {height}\n255\n'.encode())
        handle.write(np.ascontiguousarray(pixels, dtype=np.uint8).tobytes())


def from_pixels(pixels, resolution, origin, negate=0,
                occupied_thresh=0.65, free_thresh=0.196):
    """map_server's trinary rule: p = (255 - v) / 255 (v / 255 if negate)."""
    value = pixels.astype(float)
    p = value / 255.0 if negate else (255.0 - value) / 255.0
    states = np.full(pixels.shape, UNKNOWN, dtype=np.int8)
    states[p > occupied_thresh] = OCCUPIED
    states[p < free_thresh] = FREE
    return GridMap(states[::-1].copy(), float(resolution),
                   (float(origin[0]), float(origin[1])))


def to_pixels(grid):
    pixels = np.full(grid.states.shape, _PIXEL[UNKNOWN], dtype=np.uint8)
    pixels[grid.states == FREE] = _PIXEL[FREE]
    pixels[grid.states == OCCUPIED] = _PIXEL[OCCUPIED]
    return pixels[::-1].copy()


def load_map(yaml_path):
    """A map_saver .yaml + .pgm pair as a GridMap. Raises MapLoadError."""
    try:
        with open(yaml_path) as handle:
            meta = yaml.safe_load(handle)
        image = meta['image']
        if not os.path.isabs(image):
            image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
        pixels = read_pgm(image)
        return from_pixels(pixels, meta['resolution'], meta['origin'][:2],
                           meta.get('negate', 0), meta.get('occupied_thresh', 0.65),
                           meta.get('free_thresh', 0.196))
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as err:
        raise MapLoadError(f'{yaml_path}: {err}') from err


def save_map(grid, yaml_path):
    image = os.path.splitext(yaml_path)[0] + '.pgm'
    write_pgm(image, to_pixels(grid))
    with open(yaml_path, 'w') as handle:
        yaml.safe_dump({'image': os.path.basename(image), 'mode': 'trinary',
                        'resolution': grid.resolution,
                        'origin': [grid.origin[0], grid.origin[1], 0.0],
                        'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196},
                       handle)


def coverage(learner, reference):
    """Share of the reference's free cells that the learner's map knows."""
    x, y = reference.cell_centres()
    free = reference.states == FREE
    if not free.any():
        return 0.0
    known = learner.states_at(x[free], y[free]) != UNKNOWN
    return float(np.count_nonzero(known)) / float(np.count_nonzero(free))


def blocked_door_samples(learner):
    """Doorway centre-line samples that are not free in the learner's map."""
    ys = np.arange(DOOR_Y[0], DOOR_Y[1] + DOOR_STEP / 2, DOOR_STEP)
    xs = np.full(ys.shape, DOOR_X)
    return int(np.count_nonzero(learner.states_at(xs, ys) != FREE))


def thickness_ratio(learner, reference):
    """Learner's occupied area / reference's, inside the reference's bounds."""
    reference_area = reference.area(OCCUPIED)
    if reference_area == 0.0:
        return float('inf')
    x, y = learner.cell_centres()
    height, width = reference.states.shape
    x0, y0 = reference.origin
    inside = ((x >= x0) & (x < x0 + width * reference.resolution)
              & (y >= y0) & (y < y0 + height * reference.resolution))
    occupied = np.count_nonzero((learner.states == OCCUPIED) & inside)
    return occupied * learner.resolution ** 2 / reference_area


@dataclass
class Level:
    number: int
    status: str                         # 'pass', 'fail' or 'skip'
    numbers: dict = field(default_factory=dict)


def evaluate(learner, reference):
    """The three levels for a learner's map (None when it could not load)."""
    known = 0.0 if learner is None else learner.area(FREE) + learner.area(OCCUPIED)
    if known < MIN_KNOWN_M2:
        return [Level(1, 'fail', {'known': known}), Level(2, 'skip'), Level(3, 'skip')]
    share = coverage(learner, reference)
    blocked = blocked_door_samples(learner)
    ratio = thickness_ratio(learner, reference)
    detail = blocked == 0 and ratio <= MAX_THICKNESS
    return [
        Level(1, 'pass', {'known': known}),
        Level(2, 'pass' if share >= MIN_COVERAGE else 'fail', {'coverage': share}),
        Level(3, 'pass' if detail else 'fail', {'blocked': blocked, 'ratio': ratio}),
    ]


_TITLES = {1: 'มีแผนที่', 2: 'สำรวจครอบคลุม', 3: 'แผนที่ละเอียด'}


def format_report(levels):
    """Thai text for the terminal. Hints describe symptoms, never parameters."""
    lines = []
    for level in levels:
        head = f'ด่าน {level.number} {_TITLES[level.number]}:'
        n = level.numbers
        if level.status == 'skip':
            lines.append(f'{head} ยังไม่ตรวจ (ผ่านด่าน 1 ก่อน)')
        elif level.number == 1 and level.status == 'pass':
            lines.append(f'{head} ผ่าน (รู้จักพื้นที่ {n["known"]:.1f} ตร.ม.)')
        elif level.number == 1:
            lines.append(f'{head} ไม่ผ่าน - ไม่มีแผนที่ให้ตรวจ')
            lines.append('  คำใบ้: ใน RViz เห็นแผนที่ไหม? slam_toolbox อ่านข้อมูล LiDAR '
                         'จาก topic ไหนอยู่ ลอง `ros2 topic list` ดูว่ามี topic อะไรบ้าง')
        elif level.number == 2:
            verdict = 'ผ่าน' if level.status == 'pass' else 'ไม่ผ่าน'
            lines.append(f'{head} {verdict} (สำรวจได้ {n["coverage"]:.0%}, '
                         f'ต้องได้อย่างน้อย {MIN_COVERAGE:.0%})')
            if level.status == 'fail':
                lines.append('  คำใบ้: ขับเข้าไปทั้งสองห้องหรือยัง? ถ้าขับทั่วแล้ว '
                             'แผนที่เห็นแค่ใกล้ ๆ ตัวหุ่นหรือเปล่า - LiDAR ถูกใช้ไกลแค่ไหน')
        else:
            verdict = 'ผ่าน' if level.status == 'pass' else 'ไม่ผ่าน'
            door = 'เปิด' if n['blocked'] == 0 else f'ตัน ({n["blocked"]} จุด)'
            lines.append(f'{head} {verdict} (ช่องประตู{door}, ผนังหนา '
                         f'{n["ratio"]:.1f} เท่าของจริง ต้องไม่เกิน {MAX_THICKNESS:.0f} เท่า)')
            if level.status == 'fail':
                lines.append('  คำใบ้: แผนที่เป็นบล็อกหยาบ ๆ ไหม? '
                             'ช่องหนึ่งช่องในแผนที่กว้างกี่เซนติเมตร')
    if all(level.status == 'pass' for level in levels):
        lines.append('ผ่านครบทุกด่าน!')
    return '\n'.join(lines)
```

Add to `CMakeLists.txt`, inside `if(BUILD_TESTING)`, after the `test_line_follower` entry:

```cmake
  ament_add_pytest_test(test_slam_check test/test_slam_check.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd ROSpider/src/simulations/rospider_gazebo && python3 -m pytest -q test/test_slam_check.py`
Expected: 12 passed. If `test_coarse_cells_close_the_door_and_thicken_the_walls` fails on `blocked`, print `coarse.states_at([1.5]*31, np.arange(-0.15, 0.151, 0.01))` and check the block edges against the comment in the test before changing any constant.

---

### Task 2: `challenge_params.py` — copy, validate and merge the participant's config

**Files:**
- Create: `rospider_gazebo/challenge_params.py`
- Create: `test/test_challenge_params.py`
- Modify: `CMakeLists.txt` (register the test)

**Interfaces:**
- Produces:
  - `class ChallengeConfigError(Exception)` — its message is Thai and names the file (and line or key)
  - `ensure_user_file(template: str, path: str) -> bool` — True if it created the file
  - `load_params(path: str) -> dict` — the `/**` → `ros__parameters` mapping
  - `merge(base: dict, override: dict, source: str) -> dict`
  - `merged_params_file(base_path: str, user_path: str) -> str` — path of a temp YAML in slam_toolbox's `/**: ros__parameters:` shape

- [ ] **Step 1: Write the failing tests**

Create `test/test_challenge_params.py`:

```python
import pytest
import yaml
from rospider_gazebo import challenge_params
from rospider_gazebo.challenge_params import ChallengeConfigError

BASE = {'scan_topic': 'scan', 'max_laser_range': 12.0, 'resolution': 0.05,
        'map_update_interval': 2.0, 'mode': 'mapping'}


def _write(path, params):
    path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    return str(path)


def test_ensure_user_file_copies_once(tmp_path):
    template = tmp_path / 'template.yaml'
    template.write_text('a: 1\n')
    user = tmp_path / 'home' / '.ros' / 'slam_challenge.yaml'
    assert challenge_params.ensure_user_file(str(template), str(user)) is True
    user.write_text('edited\n')
    assert challenge_params.ensure_user_file(str(template), str(user)) is False
    assert user.read_text() == 'edited\n'


def test_participant_values_win_and_others_come_from_base():
    merged = challenge_params.merge(BASE, {'resolution': 0.25}, 'f.yaml')
    assert merged['resolution'] == 0.25
    assert merged['mode'] == 'mapping'


def test_whole_number_for_a_float_is_converted():
    merged = challenge_params.merge(BASE, {'max_laser_range': 12}, 'f.yaml')
    assert merged['max_laser_range'] == 12.0
    assert isinstance(merged['max_laser_range'], float)


def test_text_for_a_number_is_refused():
    with pytest.raises(ChallengeConfigError, match='max_laser_range'):
        challenge_params.merge(BASE, {'max_laser_range': '12'}, 'f.yaml')


def test_misspelled_key_is_refused_by_name():
    with pytest.raises(ChallengeConfigError, match='max_laser_rang'):
        challenge_params.merge(BASE, {'max_laser_rang': 12.0}, 'f.yaml')


def test_yaml_syntax_error_names_file_and_line(tmp_path):
    bad = tmp_path / 'bad.yaml'
    bad.write_text('/**:\n  ros__parameters:\n    resolution: [0.05\n')
    with pytest.raises(ChallengeConfigError) as err:
        challenge_params.load_params(str(bad))
    assert 'bad.yaml' in str(err.value)
    assert 'บรรทัด' in str(err.value)


def test_wrong_shape_is_refused(tmp_path):
    flat = tmp_path / 'flat.yaml'
    flat.write_text('resolution: 0.05\n')
    with pytest.raises(ChallengeConfigError, match='ros__parameters'):
        challenge_params.load_params(str(flat))


def test_merged_file_is_slam_toolbox_shaped(tmp_path):
    base = _write(tmp_path / 'base.yaml', BASE)
    user = _write(tmp_path / 'user.yaml', {'scan_topic': 'scan_raw'})
    path = challenge_params.merged_params_file(base, user)
    params = yaml.safe_load(open(path))['/**']['ros__parameters']
    assert params['scan_topic'] == 'scan_raw'
    assert params['max_laser_range'] == 12.0


def test_shipped_template_and_answer_key_merge_cleanly():
    """The two files in config/ must only use keys Hiwonder's slam.yaml has."""
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    package = os.path.dirname(here)
    base = os.path.join(package, '..', '..', 'slam', 'config', 'slam.yaml')
    for name in ('slam_challenge.yaml', 'slam_challenge_solved.yaml'):
        user = os.path.join(package, 'config', name)
        challenge_params.merge(challenge_params.load_params(base),
                               challenge_params.load_params(user), user)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd ROSpider/src/simulations/rospider_gazebo && python3 -m pytest -q test/test_challenge_params.py`
Expected: collection error, `ImportError: cannot import name 'challenge_params'`.

- [ ] **Step 3: Write the implementation and the two config files**

Create `rospider_gazebo/challenge_params.py`:

```python
"""The SLAM exercise's config: the participant's file over Hiwonder's slam.yaml.

Pure Python (no cv2: slam_challenge.launch.py imports this). The launch
copies config/slam_challenge.yaml to ~/.ros/slam_challenge.yaml the first
time, and on every run merges that file over slam/config/slam.yaml into a
temp file for slam_toolbox. Two mistakes a participant can make would be
silent or baffling inside slam_toolbox, so they are caught here:
  - a misspelled key: slam_toolbox ignores unknown parameters;
  - 12 where the default is 12.0: rclcpp aborts on the type mismatch.
"""

import os
import shutil
import tempfile

import yaml


class ChallengeConfigError(Exception):
    """The participant's file cannot be used; the message says why, in Thai."""


def ensure_user_file(template, path):
    """Copy the starting config to `path` unless it is already there."""
    if os.path.exists(path):
        return False
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    shutil.copyfile(template, path)
    return True


def load_params(path):
    """The `/**: ros__parameters:` mapping of a slam_toolbox params file."""
    try:
        with open(path) as handle:
            data = yaml.safe_load(handle)
    except yaml.YAMLError as err:
        mark = getattr(err, 'problem_mark', None)
        where = f' บรรทัด {mark.line + 1}' if mark is not None else ''
        problem = getattr(err, 'problem', None) or err
        raise ChallengeConfigError(f'{path}{where}: รูปแบบ YAML ผิด ({problem})') from err
    except OSError as err:
        raise ChallengeConfigError(f'{path}: เปิดไฟล์ไม่ได้ ({err})') from err
    try:
        params = data['/**']['ros__parameters']
    except (TypeError, KeyError):
        params = None
    if not isinstance(params, dict):
        raise ChallengeConfigError(
            f"{path}: ต้องขึ้นต้นด้วย '/**:' ตามด้วย 'ros__parameters:' เหมือนไฟล์ตั้งต้น")
    return params


def merge(base, override, source):
    """`base` with `override`'s values; `source` names the file in errors."""
    merged = dict(base)
    for key, value in override.items():
        if key not in base:
            raise ChallengeConfigError(
                f"{source}: ไม่รู้จัก parameter '{key}' (พิมพ์ชื่อผิดหรือเปล่า?)")
        default = base[key]
        is_number = isinstance(value, (int, float)) and not isinstance(value, bool)
        if isinstance(default, float) and not isinstance(default, bool):
            if not is_number:
                raise ChallengeConfigError(
                    f"{source}: '{key}' ต้องเป็นตัวเลข แต่ได้ {value!r}")
            value = float(value)
        merged[key] = value
    return merged


def merged_params_file(base_path, user_path):
    """Write the merged params to a temp file and return its path."""
    params = merge(load_params(base_path), load_params(user_path), user_path)
    with tempfile.NamedTemporaryFile('w', prefix='slam_challenge_', suffix='.yaml',
                                     delete=False) as handle:
        yaml.safe_dump({'/**': {'ros__parameters': params}}, handle)
        return handle.name
```

Create `config/slam_challenge.yaml`:

```yaml
# โจทย์ SLAM Mapping
# แก้ค่าในไฟล์นี้ แล้วปิด launch (Ctrl+C) และเปิดใหม่ด้วยคำสั่งเดิม
# ค่าที่ไม่ได้อยู่ในไฟล์นี้ใช้ของ Hiwonder (slam/config/slam.yaml)
# อยากเริ่มโจทย์ใหม่: ลบไฟล์นี้ทิ้ง แล้ว launch ใหม่
/**:
  ros__parameters:
    # topic ของข้อมูล LiDAR ที่ slam_toolbox อ่าน
    scan_topic: scan_raw
    # ระยะไกลสุดของ LiDAR ที่เอามาวาดแผนที่ (เมตร)
    max_laser_range: 1.0
    # ขนาดของช่องหนึ่งช่องในแผนที่ (เมตรต่อช่อง)
    resolution: 0.25
    # ส่งแผนที่ใหม่ออกมาทุกกี่วินาที
    map_update_interval: 2.0
    # ต้องเดินไกลกี่เมตร ถึงจะเอาสแกนใหม่มาใช้
    minimum_travel_distance: 0.03
    # หรือต้องหันกี่เรเดียน ถึงจะเอาสแกนใหม่มาใช้
    minimum_travel_heading: 0.3
```

Create `config/slam_challenge_solved.yaml` (answer key, for instructors):

```yaml
# เฉลยโจทย์ SLAM Mapping (สำหรับวิทยากร) - ใช้สร้างแผนที่อ้างอิง maps/slam_challenge.*
# ros2 launch rospider_gazebo slam_challenge.launch.py params:=<path ของไฟล์นี้>
/**:
  ros__parameters:
    scan_topic: scan
    max_laser_range: 12.0
    resolution: 0.05
    map_update_interval: 2.0
    minimum_travel_distance: 0.03
    minimum_travel_heading: 0.3
```

Add to `CMakeLists.txt`, after the `test_slam_check` entry:

```cmake
  ament_add_pytest_test(test_challenge_params test/test_challenge_params.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd ROSpider/src/simulations/rospider_gazebo && python3 -m pytest -q test/test_challenge_params.py`
Expected: 9 passed.

---

### Task 3: `check_slam.py` CLI

**Files:**
- Create: `scripts/check_slam.py`
- Modify: `CMakeLists.txt` (install it)

**Interfaces:**
- Consumes: `slam_check.load_map`, `slam_check.MapLoadError`, `slam_check.evaluate`, `slam_check.format_report` (Task 1); `rospider_gazebo.maps.resolve_map(value, extension)` (existing).
- Produces: `ros2 run rospider_gazebo check_slam.py <map> [--reference <yaml>]`, exit code 0 only when all three levels pass. Task 4 uses `--reference` before the reference map exists.

- [ ] **Step 1: Write the script**

Create `scripts/check_slam.py` (mode 755):

```python
#!/usr/bin/env python3
"""ตรวจแผนที่ของโจทย์ SLAM Mapping ว่าผ่านกี่ด่าน

    ros2 run rospider_gazebo check_slam.py room1              # ROSpider/maps/room1.yaml
    ros2 run rospider_gazebo check_slam.py ~/somewhere/m.yaml

Compares the map with maps/slam_challenge.yaml (the reference saved with
config/slam_challenge_solved.yaml); see rospider_gazebo/slam_check.py.
"""

import argparse
import os
import sys

from ament_index_python.packages import get_package_share_directory
from rospider_gazebo import slam_check
from rospider_gazebo.maps import resolve_map


def main():
    parser = argparse.ArgumentParser(description='ตรวจแผนที่ของโจทย์ SLAM Mapping')
    parser.add_argument('map', help='ชื่อแผนที่ใน ROSpider/maps (เช่น room1) '
                                    'หรือ path ของไฟล์ .yaml')
    parser.add_argument('--reference', default=os.path.join(
        get_package_share_directory('rospider_gazebo'), 'maps', 'slam_challenge.yaml'),
        help='แผนที่อ้างอิง (สำหรับวิทยากร)')
    args = parser.parse_args()

    reference = slam_check.load_map(args.reference)
    path = resolve_map(args.map, '.yaml')
    try:
        learner = slam_check.load_map(path)
    except slam_check.MapLoadError as err:
        print(f'อ่านแผนที่ไม่ได้: {err}')
        learner = None
    levels = slam_check.evaluate(learner, reference)
    print(slam_check.format_report(levels))
    return 0 if all(level.status == 'pass' for level in levels) else 1


if __name__ == '__main__':
    sys.exit(main())
```

In `CMakeLists.txt`, change the first `install(PROGRAMS ...)` block to add the script:

```cmake
install(PROGRAMS scripts/sim_gait.py scripts/color_detect.py scripts/pick_and_place.py
                 scripts/apriltag_detect.py scripts/yolo_detect.py scripts/check_slam.py
  DESTINATION lib/${PROJECT_NAME})
```

- [ ] **Step 2: Build and check the error paths**

Run (from `ROSpider/`):

```bash
chmod 755 src/simulations/rospider_gazebo/scripts/check_slam.py
colcon build --packages-select rospider_gazebo --symlink-install
source install/local_setup.bash
python3 - <<'EOF'
import sys
sys.path.insert(0, 'src/simulations/rospider_gazebo')
sys.path.insert(0, 'src/simulations/rospider_gazebo/test')
from test_slam_check import _room
from rospider_gazebo import slam_check
slam_check.save_map(_room(), '/tmp/claude_ref.yaml')
EOF
ros2 run rospider_gazebo check_slam.py /tmp/claude_ref.yaml --reference /tmp/claude_ref.yaml; echo "exit $?"
ros2 run rospider_gazebo check_slam.py no_such_map --reference /tmp/claude_ref.yaml; echo "exit $?"
```

Expected: the first prints three "ผ่าน" lines and "ผ่านครบทุกด่าน!", `exit 0`. The second prints "อ่านแผนที่ไม่ได้: ...", level 1 "ไม่ผ่าน" with the `ros2 topic list` hint, two "ยังไม่ตรวจ" lines, `exit 1`, and no traceback. (Use the session scratchpad instead of `/tmp` when running as an agent.)

---

### Task 4: the room, the launch file, and `slam.launch.py`'s `params_file`

**Files:**
- Create: `worlds/slam_challenge.sdf`
- Create: `launch/slam_challenge.launch.py`
- Modify: `launch/slam.launch.py:27` and its `generate_launch_description` argument list

**Interfaces:**
- Consumes: `challenge_params.ensure_user_file`, `challenge_params.merged_params_file`, `challenge_params.ChallengeConfigError` (Task 2).
- Produces: `ros2 launch rospider_gazebo slam_challenge.launch.py [params:=<path>] [gui:=] [rviz:=]`; `slam.launch.py params_file:=<path>`.

- [ ] **Step 1: Add `params_file` to `slam.launch.py`**

In `launch/slam.launch.py`, replace

```python
            'slam_params_file': os.path.join(slam_pkg, 'config', 'slam.yaml'),
```

with

```python
            'slam_params_file': LaunchConfiguration('params_file'),
```

and add to the returned `LaunchDescription` list, after the `rviz` argument:

```python
        DeclareLaunchArgument('params_file', default_value=os.path.join(slam_pkg, 'config', 'slam.yaml'),
                              description="slam_toolbox parameters (default: Hiwonder's slam/config/slam.yaml)"),
```

- [ ] **Step 2: Write the world**

Create `worlds/slam_challenge.sdf`. Wall centres sit 0.025 m outside the inner faces, so the inner faces are exactly x = -1.0/4.0 and y = -2.0/2.0; the partition is centred on x = 1.5 with the doorway y -0.25..0.25:

```xml
<?xml version="1.0"?>
<!-- The SLAM Mapping exercise room (docs/superpowers/specs/2026-09-28-slam-challenge-design.md).
     Inside the walls: x -1.0..4.0, y -2.0..2.0. A partition centred on x = 1.5 splits it into room A
     (the robot spawns at the origin facing +x) and room B; the doorway is the gap y -0.25..0.25.
     rospider_gazebo/slam_check.py's DOOR_X / DOOR_Y depend on these numbers. -->
<sdf version="1.9">
  <world name="slam_challenge">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.8 0.8 0.8 1</background>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.75 0.75 0.72 1</ambient><diffuse>0.75 0.75 0.72 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="walls">
      <static>true</static>
      <link name="link">
        <collision name="north_c"><pose>1.5 2.025 0.5 0 0 0</pose><geometry><box><size>5.1 0.05 1.0</size></box></geometry></collision>
        <visual name="north_v"><pose>1.5 2.025 0.5 0 0 0</pose><geometry><box><size>5.1 0.05 1.0</size></box></geometry><material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>textures/wall_north.png</albedo_map><roughness>0.9</roughness><metalness>0</metalness></metal></pbr></material></visual>
        <collision name="south_c"><pose>1.5 -2.025 0.5 0 0 0</pose><geometry><box><size>5.1 0.05 1.0</size></box></geometry></collision>
        <visual name="south_v"><pose>1.5 -2.025 0.5 0 0 0</pose><geometry><box><size>5.1 0.05 1.0</size></box></geometry><material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>textures/wall_south.png</albedo_map><roughness>0.9</roughness><metalness>0</metalness></metal></pbr></material></visual>
        <collision name="east_c"><pose>4.025 0 0.5 0 0 0</pose><geometry><box><size>0.05 4.1 1.0</size></box></geometry></collision>
        <visual name="east_v"><pose>4.025 0 0.5 0 0 0</pose><geometry><box><size>0.05 4.1 1.0</size></box></geometry><material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>textures/wall_east.png</albedo_map><roughness>0.9</roughness><metalness>0</metalness></metal></pbr></material></visual>
        <collision name="west_c"><pose>-1.025 0 0.5 0 0 0</pose><geometry><box><size>0.05 4.1 1.0</size></box></geometry></collision>
        <visual name="west_v"><pose>-1.025 0 0.5 0 0 0</pose><geometry><box><size>0.05 4.1 1.0</size></box></geometry><material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>textures/wall_west.png</albedo_map><roughness>0.9</roughness><metalness>0</metalness></metal></pbr></material></visual>
        <collision name="partition_n_c"><pose>1.5 1.125 0.5 0 0 0</pose><geometry><box><size>0.05 1.75 1.0</size></box></geometry></collision>
        <visual name="partition_n_v"><pose>1.5 1.125 0.5 0 0 0</pose><geometry><box><size>0.05 1.75 1.0</size></box></geometry><material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>textures/partition.png</albedo_map><roughness>0.9</roughness><metalness>0</metalness></metal></pbr></material></visual>
        <collision name="partition_s_c"><pose>1.5 -1.125 0.5 0 0 0</pose><geometry><box><size>0.05 1.75 1.0</size></box></geometry></collision>
        <visual name="partition_s_v"><pose>1.5 -1.125 0.5 0 0 0</pose><geometry><box><size>0.05 1.75 1.0</size></box></geometry><material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>textures/partition.png</albedo_map><roughness>0.9</roughness><metalness>0</metalness></metal></pbr></material></visual>
      </link>
    </model>

    <model name="box_obstacle">
      <static>true</static>
      <pose>0.5 1.2 0.15 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.4 0.4 0.3</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.4 0.4 0.3</size></box></geometry><material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>textures/box.png</albedo_map><roughness>0.9</roughness><metalness>0</metalness></metal></pbr></material></visual>
      </link>
    </model>

    <model name="pillar">
      <static>true</static>
      <pose>2.8 -1.0 0.3 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><cylinder><radius>0.1</radius><length>0.6</length></cylinder></geometry></collision>
        <visual name="v"><geometry><cylinder><radius>0.1</radius><length>0.6</length></cylinder></geometry><material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>textures/cylinder.png</albedo_map><roughness>0.9</roughness><metalness>0</metalness></metal></pbr></material></visual>
      </link>
    </model>
  </world>
</sdf>
```

- [ ] **Step 3: Write the launch file**

Create `launch/slam_challenge.launch.py`:

```python
"""`ros2 launch rospider_gazebo slam_challenge.launch.py` -- the SLAM Mapping exercise.

The two-room world (worlds/slam_challenge.sdf) with slam.launch.py, run on
the participant's own params file, ~/.ros/slam_challenge.yaml. The first
run copies config/slam_challenge.yaml (three values deliberately wrong)
there; the participant edits it and relaunches, and deleting it starts the
exercise over. Every run merges it over Hiwonder's slam/config/slam.yaml
(rospider_gazebo/challenge_params.py). Score the saved map with
`ros2 run rospider_gazebo check_slam.py <name>`.

  params  the participant's file (default ~/.ros/slam_challenge.yaml);
          instructors pass config/slam_challenge_solved.yaml's path
  gui, rviz  as slam.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from rospider_gazebo import challenge_params


def launch_setup(context):
    pkg = get_package_share_directory('rospider_gazebo')
    user = os.path.expanduser(LaunchConfiguration('params').perform(context))
    created = challenge_params.ensure_user_file(
        os.path.join(pkg, 'config', 'slam_challenge.yaml'), user)
    base = os.path.join(get_package_share_directory('slam'), 'config', 'slam.yaml')
    try:
        merged = challenge_params.merged_params_file(base, user)
    except challenge_params.ChallengeConfigError as err:
        raise RuntimeError(f'ไฟล์โจทย์ใช้ไม่ได้: {err}') from None

    note = ('สร้างไฟล์โจทย์ใหม่ที่' if created else 'ใช้ไฟล์โจทย์') + f' {user}'
    return [
        LogInfo(msg=f'[slam_challenge] {note} -- แก้ไฟล์นี้แล้ว launch ใหม่'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'slam.launch.py')),
            launch_arguments={
                'world': os.path.join(pkg, 'worlds', 'slam_challenge.sdf'),
                'params_file': merged,
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
            }.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value='~/.ros/slam_challenge.yaml',
                              description="the participant's slam_toolbox values"),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
```

- [ ] **Step 4: Build and check the launch files load**

Run (from `ROSpider/`):

```bash
colcon build --packages-select rospider_gazebo --symlink-install && source install/local_setup.bash
ros2 launch rospider_gazebo slam.launch.py --show-args | grep -A2 params_file
ros2 launch rospider_gazebo slam_challenge.launch.py --show-args
```

Expected: `params_file` listed with Hiwonder's slam.yaml as default; the challenge lists `params`, `gui`, `rviz`. No traceback.

---

### Task 5: verify in the sim, record the reference map, tune thresholds

**Files:**
- Create: `maps/slam_challenge.yaml`, `maps/slam_challenge.pgm` (saved by map_saver_cli)
- Possibly modify: `rospider_gazebo/slam_check.py` constants `MIN_COVERAGE`, `MAX_THICKNESS` (only if measurements demand it, with the measured numbers in a comment)
- Scratch only (not committed): `<scratchpad>/drive_route.py`

**Interfaces:**
- Consumes: everything above.

- [ ] **Step 1: Write the scripted route (scratchpad, not the repo)**

`<scratchpad>/drive_route.py` — holonomic P-control on `/odom` (the hexapod strafes; `sim_gait` reads `linear.y`), clear of the box (0.3..0.7, 1.0..1.4) and the pillar (2.8, -1.0):

```python
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

ROUTE = [(1.0, 0.0), (2.2, 0.0), (2.2, 1.3), (3.5, 1.3), (3.5, -1.6), (2.2, -1.6),
         (2.2, 0.0), (0.0, 0.0), (-0.5, 1.3), (-0.5, -1.4), (0.8, -1.4), (0.0, 0.0)]
SPEED, TOLERANCE, LEG_TIMEOUT = 0.15, 0.05, 60.0


def main():
    rclpy.init()
    node = rclpy.create_node('drive_route')
    pose = {}

    def on_odom(msg):
        q = msg.pose.pose.orientation
        pose['x'], pose['y'] = msg.pose.pose.position.x, msg.pose.pose.position.y
        pose['yaw'] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    node.create_subscription(Odometry, '/odom', on_odom, 10)
    pub = node.create_publisher(Twist, '/controller/cmd_vel', 1)
    while 'x' not in pose:
        rclpy.spin_once(node, timeout_sec=0.1)
    start = time.time()
    for gx, gy in ROUTE:
        deadline = time.time() + LEG_TIMEOUT
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            dx, dy = gx - pose['x'], gy - pose['y']
            dist = math.hypot(dx, dy)
            if dist < TOLERANCE:
                break
            c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])
            scale = min(SPEED, dist) / dist
            twist = Twist()
            twist.linear.x = (c * dx + s * dy) * scale
            twist.linear.y = (-s * dx + c * dy) * scale
            pub.publish(twist)
        else:
            print(f'leg to ({gx}, {gy}) timed out at ({pose["x"]:.2f}, {pose["y"]:.2f})')
    pub.publish(Twist())
    print(f'route done in {time.time() - start:.0f} s')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Record the reference map with the answer key**

```bash
# terminal 1 (from ROSpider/)
ros2 launch rospider_gazebo slam_challenge.launch.py rviz:=false \
  params:=$PWD/src/simulations/rospider_gazebo/config/slam_challenge_solved.yaml
# terminal 2, once /map is published (ros2 topic echo --once /map --no-arr)
python3 <scratchpad>/drive_route.py
ros2 run nav2_map_server map_saver_cli -f src/simulations/rospider_gazebo/maps/slam_challenge \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0
```

Expected: route completes without a timed-out leg; the saved PGM shows both rooms, the box, the pillar and an open doorway (view the PGM image). Then check it against itself: `ros2 run rospider_gazebo check_slam.py src/simulations/rospider_gazebo/maps/slam_challenge.yaml --reference src/simulations/rospider_gazebo/maps/slam_challenge.yaml` → all pass. Rebuild so the map is installed.

- [ ] **Step 3: Run the four participant states and record the numbers**

For each state, write the values into a scratch copy of `config/slam_challenge.yaml`, pass it with `params:=`, run the route, save the map to `maps/check_<state>` and run `ros2 run rospider_gazebo check_slam.py check_<state>`:

| State | scan_topic | max_laser_range | resolution | Expected |
|---|---|---|---|---|
| start | scan_raw | 1.0 | 0.25 | map_saver fails (no /map) → level 1 fail |
| topic fixed | scan | 1.0 | 0.25 | level 1 pass, level 2 fail, level 3 fail |
| + range fixed | scan | 12.0 | 0.25 | levels 1-2 pass, level 3 fail |
| all fixed | scan | 12.0 | 0.05 | all pass |

Also record: coverage when the robot never leaves room A (drive the first leg only, then save) — must fail level 2, so a participant who skips room B cannot pass.

If a state does not match the expectation, adjust `MIN_COVERAGE` / `MAX_THICKNESS` only when the measured values leave a clear margin on both sides (e.g. solved coverage 0.95 and range-1.0 coverage 0.40 → 0.80 stays). Write the measured numbers in a comment above the constants. If no threshold separates the states, stop and report to the user rather than moving the goalposts. Delete the `maps/check_*` files afterwards.

- [ ] **Step 4: Time the exercise**

Time a manual run with teleop from the starting file to "ผ่านครบทุกด่าน!" (edit, relaunch three times, drive, save, check). Expected: 10-15 minutes. Report the number.

- [ ] **Step 5: Run the whole test suite**

Run: `cd ROSpider/src/simulations/rospider_gazebo && python3 -m pytest -q test`
Expected: all pass (179 existing + 21 new).

---

### Task 6: docs, then hand back for the commit

**Files:**
- Modify: `ROSpider/SIMULATION.md` (under `### SLAM Mapping`)
- Modify: `CLAUDE.md` (the simulation SLAM/Nav2 bullet list)

- [ ] **Step 1: Add the exercise to `ROSpider/SIMULATION.md`**

Insert at the end of the `### SLAM Mapping` section (before `### RTAB-VSLAM 3D Mapping`):

````markdown
**โจทย์: ซ่อมค่าให้ผ่านด่าน** (10–15 นาที)

ห้องโจทย์มีสองห้องเชื่อมกันด้วยประตู ค่า SLAM ที่ให้มาตั้งผิดไว้ 3 จาก 6 ค่า หาให้เจอแล้วแก้จนผ่านครบ 3 ด่าน

```bash
ros2 launch rospider_gazebo slam_challenge.launch.py
```

1. launch จะบอก path ของไฟล์โจทย์ (`~/.ros/slam_challenge.yaml`) แก้ค่าในไฟล์นั้น แล้วปิด-เปิด launch ใหม่
2. ขับให้ทั่วทั้งสองห้อง แล้วเซฟแผนที่: `ros2 run nav2_map_server map_saver_cli -f maps/<ชื่อ> --ros-args -p use_sim_time:=true`
3. ตรวจ: `ros2 run rospider_gazebo check_slam.py <ชื่อ>` — บอกผลทีละด่านพร้อมคำใบ้

ด่าน: 1 มีแผนที่ · 2 สำรวจครอบคลุม ≥ 80% · 3 แผนที่ละเอียด (ช่องประตูเปิด ผนังไม่หนาเกินจริง) — อยากเริ่มใหม่ให้ลบไฟล์โจทย์ทิ้ง
````

- [ ] **Step 2: Add a bullet to `CLAUDE.md`**

In the simulation section, after the `- **SLAM/Nav2 plumbing.**` bullet group, add:

```markdown
- **SLAM exercise (`slam_challenge.launch.py`).** `worlds/slam_challenge.sdf` (two rooms, 0.5 m doorway at x = 1.5) with `slam.launch.py params_file:=` a temp merge of Hiwonder's `slam.yaml` under the participant's `~/.ros/slam_challenge.yaml` (copied from `config/slam_challenge.yaml`, three values wrong: `scan_topic`, `max_laser_range`, `resolution`; `rospider_gazebo/challenge_params.py`, tested: unknown keys and non-numbers are refused, ints become floats because rclcpp aborts on the type mismatch). `check_slam.py` scores the saved map against `maps/slam_challenge.*` (recorded with `config/slam_challenge_solved.yaml`) in metres (`rospider_gazebo/slam_check.py`, tested): known area, coverage of the reference's free cells (known, not free — coarse cells turn wall-side cells occupied), doorway samples free and occupied-area ratio. The doorway constants in `slam_check.py` depend on the world file.
```

- [ ] **Step 3: Final checks**

Run (from `ROSpider/`): `colcon build --packages-select rospider_gazebo --symlink-install` and `cd src/simulations/rospider_gazebo && python3 -m pytest -q test`
Expected: build OK, all tests pass.

- [ ] **Step 4: Hand back**

Report to the user: files added, measured coverage/ratio per state, the exercise time, any threshold change. Ask whether to commit (their rule: one commit per finished feature; the earlier workshop trim is also still uncommitted, so ask whether to commit them together or separately).
