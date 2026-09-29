"""Score the AprilTag Tracking exercise (docs/superpowers/specs/2026-09-29-apriltag-challenge-design.md).

scripts/check_tag.py switches apriltag_track on and feeds the robot's odometry (the sim's odometry
is the world pose) to TagRun until the robot has walked and then stood still; the stop is scored
against the tag boards of worlds/apriltag_challenge.sdf. Pure Python (no rclpy), unit tested.

Measured with apriltag_track (target tag 2, speed 0.05 m/s, sim seconds): stop_distance 0.30 /
0.35 / 0.40 stopped 0.42 / 0.47 / 0.53 m from the board, facing it within 3 degrees, after 16 /
15 / 14 s; target tag 1 stepped 0.1 m and stopped facing tag 1; stop_distance 0.8 stopped 0.94 m
away; speed 0.01 took 73 s.

Level 1 asks which board the robot ends up facing, not which is nearest: the yaw controller keeps
the followed tag centred whether the robot walks, stands or backs away, so a wrong standoff (the
right tag, a short or backward walk) is not mistaken for a wrong tag.
"""

import math

from rospider_gazebo.slam_check import Level
from rospider_gazebo.stations import BOARD_OFFSET_X

#: tag id -> station pose (x, y, yaw) in worlds/apriltag_challenge.sdf; all face the spawn.
STATIONS = {0: (1.0, 0.45, math.pi), 1: (1.2, 0.0, math.pi), 2: (1.0, -0.45, math.pi)}
TARGET = 2
#: Robot base to the target's board, metres: stop_distance 0.30-0.40 (the arm's reach) measured.
DISTANCE_BAND = (0.40, 0.55)
#: The robot must face the board within this many degrees.
FACING_DEG = 15.0
#: Level 3: stopped within this many sim seconds of the start (answer key 14-16 s).
TIME_LIMIT = 40.0
#: Walked: further than this from the start. Stopped: moved less than STILL_M in STILL_S.
MOVED_M = 0.05
STILL_M = 0.005
STILL_S = 5.0
#: Never moved in this long, or not stopped after RUN_LIMIT: the run ends.
NEVER_MOVED_S = 20.0
RUN_LIMIT = 120.0


def board(tag):
    """World (x, y) of a tag board's centre."""
    x, y, yaw = STATIONS[tag]
    return (x + math.cos(yaw) * BOARD_OFFSET_X, y + math.sin(yaw) * BOARD_OFFSET_X)


class TagRun:
    """Where the robot is and whether it has walked and stopped, one odometry sample at a time.

    Scored against the tag boards by default; `targets` ({name: (x, y)}) and `target` score it
    against other points (rospider_gazebo/track_check.py's balls)."""

    def __init__(self, targets=None, target=TARGET):
        self.targets = {tag: board(tag) for tag in STATIONS} if targets is None else targets
        self.target = target
        self.start = None
        self.start_yaw = None
        self.start_t = None
        self.moved_t = None
        self.anchor = None          # position the stillness is measured from
        self.anchor_t = None
        self.pose = None
        self.last_t = None

    def feed(self, x, y, yaw, t):
        self.pose, self.last_t = (x, y, yaw), t
        if self.start is None:
            self.start, self.start_yaw, self.start_t = (x, y), yaw, t
            self.anchor, self.anchor_t = (x, y), t
            return
        if self.moved_t is None and math.dist((x, y), self.start) > MOVED_M:
            self.moved_t = t
        if math.dist((x, y), self.anchor) > STILL_M:
            self.anchor, self.anchor_t = (x, y), t

    def stopped(self, t):
        return self.moved_t is not None and t - self.anchor_t >= STILL_S

    def done(self, t):
        if self.start_t is None:
            return False
        return (self.stopped(t) or t - self.start_t > RUN_LIMIT
                or (self.moved_t is None and t - self.start_t > NEVER_MOVED_S))

    def summary(self):
        x, y, yaw = self.pose
        goal = self.targets[self.target]
        facing = {name: _off_bearing((x, y, yaw), point) for name, point in self.targets.items()}
        return {'moved': self.moved_t is not None,
                'stopped': self.stopped(self.last_t),
                'faced': min(facing, key=facing.get),
                'distance': math.dist((x, y), goal),
                'closer': math.dist(self.start, goal) - math.dist((x, y), goal),
                'turned': abs(math.degrees(math.atan2(math.sin(yaw - self.start_yaw),
                                                      math.cos(yaw - self.start_yaw)))),
                'facing': facing[self.target],
                'elapsed': self.last_t - self.start_t,
                'seconds': (self.anchor_t - self.start_t) if self.moved_t is not None else None}


def _off_bearing(pose, point):
    """Degrees between the robot's heading and the direction to `point`."""
    x, y, yaw = pose
    bearing = math.atan2(point[1] - y, point[0] - x)
    return abs(math.degrees(math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))))


def evaluate(summary):
    """The three levels for a run's summary."""
    if summary['faced'] != TARGET:
        return [Level(1, 'fail', summary), Level(2, 'skip', {'after': 1}),
                Level(3, 'skip', {'after': 1})]
    placed = (summary['stopped'] and DISTANCE_BAND[0] <= summary['distance'] <= DISTANCE_BAND[1]
              and summary['facing'] <= FACING_DEG)
    if not placed:
        return [Level(1, 'pass', summary), Level(2, 'fail', summary),
                Level(3, 'skip', {'after': 2})]
    return [Level(1, 'pass', summary), Level(2, 'pass', summary),
            Level(3, 'pass' if summary['seconds'] <= TIME_LIMIT else 'fail', summary)]


_TITLES = {1: f'ไปหาป้ายหมายเลข {TARGET}', 2: 'หยุดระยะพอดี', 3: 'ทันเวลา'}


def format_report(levels):
    """Thai text for the terminal. Hints describe what the robot did, never a parameter."""
    lines = []
    for level in levels:
        head = f'ด่าน {level.number} {_TITLES[level.number]}:'
        n = level.numbers
        if level.status == 'skip':
            lines.append(f'{head} ยังไม่ตรวจ (ผ่านด่าน {n["after"]} ก่อน)')
        elif level.number == 3:
            verdict = 'ผ่าน' if level.status == 'pass' else 'ไม่ผ่าน'
            lines.append(f'{head} {verdict} ({n["seconds"]:.0f} วินาที ต้องไม่เกิน {TIME_LIMIT:.0f})')
            if level.status == 'fail':
                lines.append('  คำใบ้: หยุดถูกที่แล้วแต่ช้า หุ่นเดินเข้าหาป้ายเร็วแค่ไหน')
        elif level.status == 'pass':
            lines.append(f'{head} ผ่าน')
        elif level.number == 1:
            moved = '' if n['moved'] else ' (หุ่นไม่ขยับ)'
            lines.append(f'{head} ไม่ผ่าน - หุ่นหันไปหาป้ายหมายเลข {n["faced"]}{moved}')
            lines.append(f'  คำใบ้: หุ่นเดินตามป้ายหมายเลขไหนอยู่? ต้องไปหาหมายเลข {TARGET} '
                         '- ในหน้าต่าง image หุ่นเห็นป้ายนั้นไหม')
        elif not n['stopped']:
            lines.append(f'{head} ไม่ผ่าน - ยังไม่หยุดภายใน {n["elapsed"]:.0f} วินาที '
                         f'(ยังห่างป้าย {n["distance"]:.2f} ม.)')
            lines.append('  คำใบ้: หุ่นหันถูกป้ายแล้วแต่เดินช้ามากจนไปไม่ถึง - หุ่นเดินเร็วแค่ไหน')
        else:
            lines.append(f'{head} ไม่ผ่าน - หยุดห่างป้าย {n["distance"]:.2f} ม. '
                         f'(ต้อง {DISTANCE_BAND[0]:.2f}-{DISTANCE_BAND[1]:.2f} ม.), '
                         f'หันเบี่ยงจากป้าย {n["facing"]:.0f} องศา')
            if n['distance'] > DISTANCE_BAND[1]:
                lines.append('  คำใบ้: หยุดห่างป้ายเกินไป แขนหยิบไม่ถึง - หุ่นตั้งใจหยุดห่างป้ายเท่าไหร่')
            elif n['distance'] < DISTANCE_BAND[0]:
                lines.append('  คำใบ้: หยุดชิดป้ายเกินไป - หุ่นตั้งใจหยุดห่างป้ายเท่าไหร่')
            else:
                lines.append('  คำใบ้: ระยะพอดีแต่ไม่ได้หันหน้าตรงป้าย - หุ่นหมุนตัวตามป้ายได้เร็วพอไหม')
    if all(level.status == 'pass' for level in levels):
        lines.append('ผ่านครบทุกด่าน!')
    return '\n'.join(lines)
