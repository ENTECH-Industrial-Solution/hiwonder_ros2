"""Score a saved SLAM map against the SLAM exercise's reference map.

Pure Python + numpy (no cv2, no ROS), so it is unit tested and importable
anywhere. The two maps may differ in resolution and origin, so every
comparison is made in metres: a point is looked up in a map through that
map's own origin and resolution.

Levels (docs/superpowers/specs/2026-09-28-slam-challenge-design.md):
  1. a map exists: at least MIN_KNOWN_M2 of known cells;
  2. coverage: MIN_COVERAGE of the reference's free cells are known;
  3. detail: the doorway is free along its centre line and the occupied
     area is at most MAX_THICKNESS times the reference's. Checked only
     once level 2 passes: on a partial map an unseen doorway reads as
     blocked and unseen walls make the ratio small.
"""

import os
from dataclasses import dataclass, field

import numpy as np
import yaml

FREE, OCCUPIED, UNKNOWN = 0, 100, -1

# worlds/slam_challenge.sdf: the partition is centred on x = 1.0 and the
# doorway is the gap y 0.3..0.8. Samples stay 0.15 m clear of the jambs: in
# the recorded reference the lower jamb's cells already reach y 0.353, so a
# correct map with one more cell of smear must not read as a closed door.
DOOR_X = 1.0
DOOR_Y = (0.45, 0.65)
DOOR_STEP = 0.01

# Measured in the sim on a scripted route through both rooms (coverage /
# wall ratio): answer key 100% / 0.9-1.0; answer key but never entering
# room B 85-86%; max_laser_range 0.5 (the starting value) ~20%; range fixed
# but resolution 0.25: 100% / 2.9 with 18 of 31 doorway samples blocked.
# 90% makes staying in room A fail too.
MIN_KNOWN_M2 = 1.0
MIN_COVERAGE = 0.90
MAX_THICKNESS = 1.2

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


def evaluate(learner, reference, missing=False):
    """The three levels for a learner's map (None when it could not load;
    `missing` when that was because the file does not exist)."""
    known = 0.0 if learner is None else learner.area(FREE) + learner.area(OCCUPIED)
    if known < MIN_KNOWN_M2:
        return [Level(1, 'fail', {'known': known, 'missing': missing}),
                Level(2, 'skip', {'after': 1}), Level(3, 'skip', {'after': 1})]
    share = coverage(learner, reference)
    if share < MIN_COVERAGE:
        return [Level(1, 'pass', {'known': known}),
                Level(2, 'fail', {'coverage': share}), Level(3, 'skip', {'after': 2})]
    blocked = blocked_door_samples(learner)
    ratio = thickness_ratio(learner, reference)
    detail = blocked == 0 and ratio <= MAX_THICKNESS
    return [
        Level(1, 'pass', {'known': known}),
        Level(2, 'pass', {'coverage': share}),
        Level(3, 'pass' if detail else 'fail',
              {'blocked': blocked, 'ratio': ratio,
               'coarse': learner.resolution > 1.5 * reference.resolution}),
    ]


_TITLES = {1: 'มีแผนที่', 2: 'สำรวจครอบคลุม', 3: 'แผนที่ละเอียด'}


def format_report(levels):
    """Thai text for the terminal. Hints describe symptoms, never parameters."""
    lines = []
    for level in levels:
        head = f'ด่าน {level.number} {_TITLES[level.number]}:'
        n = level.numbers
        if level.status == 'skip':
            lines.append(f'{head} ยังไม่ตรวจ (ผ่านด่าน {n["after"]} ก่อน)')
        elif level.number == 1 and level.status == 'pass':
            lines.append(f'{head} ผ่าน (รู้จักพื้นที่ {n["known"]:.1f} ตร.ม.)')
        elif level.number == 1 and n.get('missing'):
            lines.append(f'{head} ไม่ผ่าน - ไม่พบไฟล์แผนที่')
            lines.append('  คำใบ้: map_saver_cli เซฟสำเร็จไหม? ถ้าขึ้น Failed แปลว่ายังไม่มีแผนที่เลย - '
                         'ใน RViz เห็นแผนที่ไหม? slam_toolbox อ่านข้อมูล LiDAR จาก topic ไหนอยู่ '
                         'ลอง `ros2 topic list` ดู / ถ้าเซฟสำเร็จ ตรวจว่าพิมพ์ชื่อตรงกับที่เซฟ')
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
                         f'{n["ratio"]:.1f} เท่าของจริง ต้องไม่เกิน {MAX_THICKNESS:g} เท่า)')
            if level.status == 'fail' and n.get('coarse'):
                lines.append('  คำใบ้: แผนที่เป็นบล็อกหยาบ ๆ ไหม? '
                             'ช่องหนึ่งช่องในแผนที่กว้างกี่เซนติเมตร')
            elif level.status == 'fail':
                lines.append('  คำใบ้: ผนังในแผนที่ซ้อนเป็นหลายชั้นหรือเบี้ยวไหม? แปลว่าสแกนแต่ละรอบ'
                             'วางไม่ตรงกัน - SLAM ต้องเห็นผนังรอบตัวมากพอถึงจะจับคู่สแกนได้แม่น')
    if all(level.status == 'pass' for level in levels):
        lines.append('ผ่านครบทุกด่าน!')
    return '\n'.join(lines)
