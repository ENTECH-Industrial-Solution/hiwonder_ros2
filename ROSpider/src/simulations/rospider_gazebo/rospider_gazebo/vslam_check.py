"""Score the V-SLAM mapping exercise (docs/superpowers/specs/2026-09-28-vslam-challenge-design.md).

scripts/check_vslam.py exports the 2D grid from the participant's RTAB-Map database with
rtabmap-export and reads its graph here. Pure Python (sqlite3, numpy; no rclpy), unit tested.

Loop closures are judged against odometry: in the sim odometry is exact, and RTAB-Map stores
each node's odometry pose in Node.pose, so a closure link (Link.type 1 global, 2 local) whose
transform disagrees with the odometry between its nodes is a place the robot "recognised"
wrongly. On a real robot, with drifting odometry, this test would not hold.

Measured in the textured maze room with tools/drive_route.py (reference = the answer key,
Hiwonder's values; levels as scored here):
  answer key (3 runs): 17.1-17.3 m2, walls 94-100%, coverage 98-100%, 25-31 closures, largest
    error 0.12 m (depth registration noise -- hence MAX_CLOSURE_ERROR_M 0.3), 1.0-1.1x: pass;
  starting file (camera at the floor, Grid/RangeMax 0.5, Vis/MinInliers 5): 7.7 m2, walls 22%,
    14 wrong closures (the floor looks the same everywhere): level 1;
  camera fixed, range still 0.5: 0.5 m2: level 1; camera at the floor, range fixed: walls 27%:
    level 1 -- walls count only next to mapped floor (WALL_SEEN_CELLS), so each order of fixes
    stays at level 1 until both are right;
  only Vis/MinInliers 5 left: level 3 in 4 of 5 runs (1 wrong closure, or walls 1.3x), passed
    once -- accepted, like the SLAM exercise's partial fix; "3" passed 2 of 2, and turning
    RGBD/OptimizeMaxError off bent the map past level 1, so neither replaces it;
  answer key, room A only: walls 98%, coverage 52%: level 2.
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
MAX_CLOSURE_ERROR_M = 0.3
MAX_CLOSURE_ERROR_DEG = 10.0
WALL_SEEN_CELLS = 3
#: A closure is a return to the start when its older node lies this close to the first node and
#: its newer node came this many nodes (~seconds) later: mid-route closures in room B and those
#: made standing still at the spawn point do not show that the route came back.
RETURN_RADIUS_M = 0.5
RETURN_GAP_NODES = 30
CLOSURE_TYPES = (1, 2)


def _matrix(blob):
    m = np.eye(4)
    m[:3, :4] = np.array(struct.unpack('12f', blob[:48])).reshape(3, 4)
    return m


def read_graph(db_path):
    """Odometry poses by node id, and each closure link once (RTAB-Map stores both directions)."""
    types = ', '.join(str(t) for t in CLOSURE_TYPES)
    with closing(sqlite3.connect(f'file:{db_path}?mode=ro', uri=True)) as db:
        poses = {i: _matrix(p) for i, p in db.execute('SELECT id, pose FROM Node')}
        links = [(f, t, k, _matrix(tr)) for f, t, k, tr in db.execute(
            f'SELECT from_id, to_id, type, transform FROM Link WHERE type IN ({types}) '
            'AND from_id > to_id ORDER BY from_id, to_id')]
    return poses, links


def closure_errors(poses, links):
    """(from, to, metres, degrees, back at the start) for each closure link: its disagreement
    with the odometry between its nodes, and whether it is a return to the first node's place."""
    if not poses:
        return []
    first = poses[min(poses)]
    errors = []
    for f, t, _, transform in links:
        if f not in poses or t not in poses:
            continue
        odometry = np.linalg.inv(poses[f]) @ poses[t]
        d = np.linalg.inv(odometry) @ transform
        home = math.hypot(poses[t][0, 3] - first[0, 3], poses[t][1, 3] - first[1, 3])
        errors.append((f, t, math.hypot(d[0, 3], d[1, 3]),
                       abs(math.degrees(math.atan2(d[1, 0], d[0, 0]))),
                       home <= RETURN_RADIUS_M and f - t >= RETURN_GAP_NODES))
    return errors


def wrong_count(errors):
    """How many closures put two nodes further apart than odometry says they are."""
    return sum(1 for _, _, metres, degrees, _ in errors
               if metres > MAX_CLOSURE_ERROR_M or degrees > MAX_CLOSURE_ERROR_DEG)


def _grow(mask, cells):
    """`mask` dilated by a square of `cells` cells each way."""
    padded = np.pad(mask, cells)
    h, w = mask.shape
    grown = np.zeros_like(mask)
    for dy in range(2 * cells + 1):
        for dx in range(2 * cells + 1):
            grown |= padded[dy:dy + h, dx:dx + w]
    return grown


def wall_recall(learner, reference, slack=WALL_SLACK_CELLS, seen=WALL_SEEN_CELLS):
    """Of the reference's wall cells next to floor the learner mapped (within `seen` cells),
    the share with a learner wall within `slack` cells. Walls of rooms never entered do not
    count (that is level 2's coverage), so a camera that saw the floor but not the walls
    scores low whatever route was driven."""
    def near(mask, cells):
        return GridMap(np.where(_grow(mask, cells), OCCUPIED, FREE).astype(np.int8),
                       learner.resolution, learner.origin)
    x, y = reference.cell_centres()
    walls = reference.states == OCCUPIED
    xs, ys = x[walls], y[walls]
    visited = near(learner.states == FREE, seen).states_at(xs, ys) == OCCUPIED
    if not visited.any():
        return 0.0
    hit = near(learner.states == OCCUPIED, slack).states_at(xs, ys) == OCCUPIED
    return float(np.count_nonzero(hit & visited)) / float(np.count_nonzero(visited))


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
    walls = 0.0 if learner is None else wall_recall(learner, reference)
    level1 = {'known': known, 'walls': walls, 'missing': missing,
              'wrong': wrong_count(closures or [])}
    if known < MIN_KNOWN_M2 or walls < MIN_WALLS:
        return [Level(1, 'fail', level1), Level(2, 'skip', {'after': 1}),
                Level(3, 'skip', {'after': 1})]
    share = coverage(learner, reference)
    if share < MIN_COVERAGE:
        return [Level(1, 'pass', level1), Level(2, 'fail', {'coverage': share}),
                Level(3, 'skip', {'after': 2})]
    closures = closures or []
    wrong, ratio = wrong_count(closures), thickness_ratio(learner, reference)
    returns = sum(1 for closure in closures if closure[4])
    straight = returns > 0 and wrong == 0 and ratio <= MAX_THICKNESS
    return [Level(1, 'pass', level1), Level(2, 'pass', {'coverage': share}),
            Level(3, 'pass' if straight else 'fail',
                  {'closures': len(closures), 'returns': returns, 'wrong': wrong,
                   'ratio': ratio})]


_TITLES = {1: 'มีแผนที่และเห็นผนัง', 2: 'สำรวจครอบคลุม', 3: 'แผนที่ไม่เบี้ยว'}


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
            lines.append(f'{head} {verdict} (รู้จักพื้นที่ {n["known"]:.1f} ตร.ม. ต้องได้อย่างน้อย '
                         f'{MIN_KNOWN_M2:g}, เจอผนัง {n["walls"]:.0%} ต้องได้อย่างน้อย '
                         f'{MIN_WALLS:.0%})')
            if level.status == 'fail' and n['known'] < MIN_KNOWN_M2:
                lines.append('  คำใบ้: แผนที่เห็นแค่รอบ ๆ ตัวหุ่นไหม? '
                             'ภาพความลึกจากกล้องถูกเอามาวาดแผนที่ไกลแค่ไหน')
            elif level.status == 'fail':
                lines.append('  คำใบ้: ใน RViz เห็นพื้นแต่แทบไม่เห็นผนังไหม? กล้องหันไปทางไหนอยู่')
            if level.status == 'fail' and n.get('wrong'):
                lines.append(f'  และหุ่นจำผิดที่ {n["wrong"]} ครั้ง (คิดว่ากลับมาที่เดิมทั้งที่ไม่ใช่) '
                             'ภาพที่กล้องเห็นแยกแต่ละที่ออกจากกันได้ไหม')
        elif level.number == 2:
            lines.append(f'{head} {verdict} (สำรวจได้ {n["coverage"]:.0%} ต้องได้อย่างน้อย '
                         f'{MIN_COVERAGE:.0%})')
            if level.status == 'fail':
                lines.append('  คำใบ้: ขับเข้าไปทั้งสองห้องและทางเดินหรือยัง?')
        else:
            lines.append(f'{head} {verdict} (จำที่เดิมได้ {n["closures"]} ครั้ง '
                         f'ในนั้นกลับมาถึงจุดเริ่ม {n["returns"]} ครั้ง, จำผิดที่ '
                         f'{n["wrong"]} ครั้ง, ผนังหนา {n["ratio"]:.1f} เท่าของเฉลย '
                         f'ต้องไม่เกิน {MAX_THICKNESS:g} เท่า)')
            if level.status == 'fail' and n['wrong'] == 0 and n['returns'] == 0:
                lines.append('  คำใบ้: ขับกลับมาที่จุดเริ่มแล้วหันไปทางเดียวกับตอนเริ่มหรือยัง? '
                             'หุ่นต้องกลับมาเห็นภาพเดิมถึงจะรู้ว่าเคยมาแล้ว')
            elif level.status == 'fail' and n['wrong'] > 0:
                lines.append('  คำใบ้: หุ่นคิดว่ากลับมาที่เดิมทั้งที่ไม่ใช่ ผนังจึงซ้อนหรือเบี้ยว - '
                             'เกณฑ์ที่ใช้ตัดสินว่าภาพสองภาพเป็นที่เดียวกันหลวมไปไหม')
            elif level.status == 'fail':
                lines.append('  คำใบ้: ผนังในแผนที่ซ้อนเป็นหลายชั้นหรือหนาเกินไหม?')
    if all(level.status == 'pass' for level in levels):
        lines.append('ผ่านครบทุกด่าน!')
    return '\n'.join(lines)
