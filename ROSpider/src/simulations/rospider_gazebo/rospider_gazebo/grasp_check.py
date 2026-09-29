"""Score the 3D Object Grasping exercise (docs/superpowers/specs/2026-09-29-grasp-challenge-design.md).

scripts/check_grasp.py orders the blue block from track_and_grab (`~/pick blue`), follows the
robot's odometry (the sim's odometry is the world pose) and pick_and_place's `~/state`, and at the
end reads where the block came to rest (`gz model -p`). The scene is track_and_grab.launch.py's
`spread` layout: the blue block on a pedestal 0.8 m to the robot's right, and the target pad that
grasp_challenge.launch.py lays on the floor where the answer key puts the block down. Pure Python,
unit tested.
"""

import math
import re

from rospider_gazebo.challenge_params import ChallengeConfigError
from rospider_gazebo.slam_check import Level

#: The blue block's spawn (track_and_grab.launch.py SPREAD), x y z in the world.
BLOCK = (0.0, -0.8, 0.105)
BLOCK_MODEL = 'pick_cube_blue'
COLOR = 'blue'
#: Where the answer key puts the block down (measured -0.234 -0.584 and -0.226 -0.575), and how
#: far from it still counts as on the pad (the pad is 0.12 m square, the block 0.05 m).
PAD = (-0.23, -0.58)
PAD_SIZE = 0.12
PAD_TOL = 0.05
#: A block resting on the floor has its centre at 0.025 m; on its pedestal, 0.105 m.
FLOOR_Z = 0.05
#: Level 1: the robot walked up this close to the block (base to block centre) and faced it.
#: Measured: the answer key stops 0.29 m off, target_x 0.40 (out of the arm's reach) 0.46 m.
NEAR_M = 0.6
FACING_DEG = 20.0
#: Walked: further than this from the spawn.
MOVED_M = 0.05


def parse_point(text, source):
    """track_and_grab's place_point, "x y z" in metres, as three floats; refused otherwise."""
    try:
        point = tuple(float(v) for v in text.split())
    except ValueError:
        point = ()
    if len(point) != 3:
        raise ChallengeConfigError(
            f"{source}: 'place_point' ต้องเป็นตัวเลข 3 ตัวคั่นด้วยช่องว่าง เช่น '0.1 0.0 0.035' "
            f"(ได้ {text!r})")
    return point


def parse_gz_pose(text):
    """(x, y, z) from `gz model -m <name> -p` output, or None."""
    match = re.search(r'Pose \[ XYZ \(m\) \].*?\[\s*([-\d.e]+)\s+([-\d.e]+)\s+([-\d.e]+)\s*\]',
                      text, re.S)
    return None if match is None else tuple(float(v) for v in match.groups())


def _off_bearing(robot, point):
    x, y, yaw = robot
    bearing = math.atan2(point[1] - y, point[0] - x)
    return abs(math.degrees(math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))))


def summarise(robot, states, block):
    """The run's facts: `robot` (x, y, yaw) at the end, the pick_and_place states seen in order,
    and the block's resting (x, y, z) (None when it could not be read)."""
    return {'moved': math.hypot(robot[0], robot[1]) > MOVED_M,
            'distance': math.dist(robot[:2], BLOCK[:2]),
            'facing': _off_bearing(robot, BLOCK),
            'looked': 'LOOK' in states,
            'carried': 'CARRY' in states,
            'robot': robot,
            'block': block}


def evaluate(summary):
    """The three levels for a run's summary."""
    reached = summary['distance'] <= NEAR_M and summary['facing'] <= FACING_DEG
    if not reached and not summary['carried']:
        return [Level(1, 'fail', summary), Level(2, 'skip', {'after': 1}),
                Level(3, 'skip', {'after': 1})]
    if not summary['carried']:
        return [Level(1, 'pass', summary), Level(2, 'fail', summary),
                Level(3, 'skip', {'after': 2})]
    block = summary['block']
    on_pad = (block is not None and block[2] < FLOOR_Z
              and math.dist(block[:2], PAD) <= PAD_TOL)
    return [Level(1, 'pass', summary), Level(2, 'pass', summary),
            Level(3, 'pass' if on_pad else 'fail', summary)]


def _side(robot, point):
    """Where `point` lies from the robot, in Thai: in front / behind, left / right."""
    x, y, yaw = robot
    dx, dy = point[0] - x, point[1] - y
    ahead = math.cos(yaw) * dx + math.sin(yaw) * dy
    left = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return ('ทางซ้าย' if left > 0 else 'ทางขวา') if abs(left) >= abs(ahead) else \
        ('ข้างหน้า' if ahead > 0 else 'ข้างหลัง')


_TITLES = {1: 'เดินไปถึงบล็อกสีน้ำเงิน', 2: 'หยิบบล็อกขึ้นมา', 3: 'วางบนแผ่นสีเหลือง'}


def format_report(levels):
    """Thai text for the terminal. Hints describe what the robot did, never a parameter."""
    lines = []
    for level in levels:
        head = f'ด่าน {level.number} {_TITLES[level.number]}:'
        n = level.numbers
        if level.status == 'skip':
            lines.append(f'{head} ยังไม่ตรวจ (ผ่านด่าน {n["after"]} ก่อน)')
        elif level.status == 'pass':
            lines.append(f'{head} ผ่าน')
        elif level.number == 1:
            if not n['moved']:
                lines.append(f'{head} ไม่ผ่าน - หุ่นไม่ได้ออกเดิน')
                lines.append('  คำใบ้: บล็อกสีน้ำเงินอยู่ข้างตัวหุ่น ไกลเกินแขนเอื้อมจากจุดที่ยืน '
                             '- หุ่นต้องทำอะไรก่อนหยิบ')
            else:
                lines.append(f'{head} ไม่ผ่าน - หุ่นหยุดห่างบล็อก {n["distance"]:.2f} ม. '
                             f'หันเบี่ยงจากบล็อก {n["facing"]:.0f} องศา')
                lines.append('  คำใบ้: หุ่นหาบล็อกเจอไหม (ดูหน้าต่างของ track_and_grab)')
        elif level.number == 2:
            if n['looked']:
                lines.append(f'{head} ไม่ผ่าน - แขนพยายามหยิบแต่จับบล็อกไม่ได้ '
                             f'(หุ่นยืนห่างบล็อก {n["distance"]:.2f} ม.)')
                lines.append('  คำใบ้: แขนมองหาบล็อกตรงหน้าในระยะที่คุ้นเคย - '
                             'หุ่นเดินเข้าไปหยุดห่างบล็อกพอดีกับที่แขนถนัดไหม')
            else:
                lines.append(f'{head} ไม่ผ่าน - หุ่นเดินไปถึงแล้วแต่ไม่ยอมหยิบ '
                             f'(ยืนห่างบล็อก {n["distance"]:.2f} ม.)')
                lines.append('  คำใบ้: แขนเอื้อมได้ไกลแค่ไหน - หุ่นเดินเข้าไปหยุดห่างบล็อกเท่าไหร่')
        else:
            block = n['block']
            if block is None:
                lines.append(f'{head} ไม่ผ่าน - อ่านตำแหน่งบล็อกจาก Gazebo ไม่ได้')
            elif block[2] >= FLOOR_Z:
                lines.append(f'{head} ไม่ผ่าน - บล็อกไม่ได้อยู่บนพื้น (สูง {block[2]:.2f} ม.)')
            else:
                gap = math.dist(block[:2], PAD)
                lines.append(f'{head} ไม่ผ่าน - บล็อกอยู่ห่างกลางแผ่น {gap:.2f} ม. '
                             f'(ต้องไม่เกิน {PAD_TOL:.2f}) แผ่นอยู่{_side(n["robot"], PAD)}ของหุ่น '
                             f'แต่บล็อกไปอยู่{_side(n["robot"], block)}')
                lines.append('  คำใบ้: จุดวางนับจากตัวหุ่น - แกนไหนชี้ไปข้างหน้า แกนไหนชี้ไปทางซ้าย')
    if all(level.status == 'pass' for level in levels):
        lines.append('ผ่านครบทุกด่าน!')
    return '\n'.join(lines)
