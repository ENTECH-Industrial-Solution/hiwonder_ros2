"""Score the final mission (docs/superpowers/specs/2026-09-29-mission-challenge-design.md).

The scene's constants live here, so tools/make_mission_world.py, mission_challenge.launch.py and
scripts/check_mission.py share one source. The world frame is the map frame (the reference map
was recorded from the spawn at the origin) and the sim's odometry frame. Pure Python, unit tested.
"""

import math

from rospider_gazebo.grasp_check import side
from rospider_gazebo.slam_check import Level

#: tag id -> station model pose (x, y, yaw rad). Both stand against room B's north wall and face -y,
#: 0.6 m apart, so pick-here in front of one cannot reach the other's block.
STATIONS = {2: (3.2, 1.8, -math.pi / 2), 3: (3.8, 1.8, -math.pi / 2)}
BLOCK_OF_TAG = {2: 'red', 3: 'blue'}
TARGET_TAG = 3
TARGET_COLOR = 'blue'
#: Each block stands on a pick pedestal (models/pick_pedestal) of its own, PICK_PEDESTAL_AHEAD in
#: front of its station, BLOCK_BEYOND past the pedestal's centre -- the pick_place scene's layout.
#: Not on the station's own pedestal: there the pedestal top shows right behind the block, and
#: track_and_grab takes a blob with something that near behind it for a poster patch (measured: it
#: tracked blue and never picked).
PICK_PEDESTAL_AHEAD = 0.35
BLOCK_BEYOND = 0.035
BLOCK_Z = 0.105
#: apriltag_track's standoff that parks the robot with the block where pick-here grabs it
#: (0.20-0.27 m ahead of base_footprint). Measured in the plan's Task 3.
STOP_DISTANCE = 0.55
#: The yellow pad in room A (visual only), its size, and how far from its centre a block still
#: counts as on it (the centre over the pad).
PAD = (0.40, 0.55)
PAD_SIZE = 0.12
PAD_TOL = 0.06
#: Room B: east of the corridor wall at x = 2.0.
ROOM_B_X = 2.1
#: A block resting on its pedestal has its centre at 0.105 m, on the floor at 0.025 m; the moment
#: pick_and_place reports CARRY it has been lifted to ~0.155 m (measured 0.153-0.158).
LIFTED_Z = 0.13
FLOOR_Z = 0.05


def _ahead(tag, distance):
    x, y, yaw = STATIONS[tag]
    return x + math.cos(yaw) * distance, y + math.sin(yaw) * distance


def pick_pedestal(tag):
    """(x, y, yaw) of the pick pedestal in front of station `tag`."""
    return (*_ahead(tag, PICK_PEDESTAL_AHEAD), STATIONS[tag][2])


def block_spawn(color):
    """Where the block of `color` is spawned: on its pick pedestal, past the centre."""
    tag = next(t for t, c in BLOCK_OF_TAG.items() if c == color)
    return (*_ahead(tag, PICK_PEDESTAL_AHEAD - BLOCK_BEYOND), BLOCK_Z)


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
