"""Score the Line Following exercise (docs/superpowers/specs/2026-09-29-line-following-challenge-design.md).

scripts/check_line.py starts the line follower and feeds the robot's odometry here; LapTracker
follows it along worlds/line_track.sdf's loop (rospider_gazebo/line_track.py): how far along the
loop it got, whether it strayed off the line, and when it came round to the start again.
Pure Python and numpy (no rclpy), unit tested.

The sim's odometry is the robot's world pose (it includes the spawn pose), so it is compared with
the loop directly.
"""

import numpy as np
from rospider_gazebo.line_track import track_points
from rospider_gazebo.slam_check import Level

#: Off the line: further than this from it (the tape is 3 cm; the camera sees ~0.3 m of floor).
LOST_M = 0.2
#: Level 1 needs this share of the lap on the line.
QUARTER = 0.25
#: Level 3: the whole lap in at most this many sim seconds. Measured: upstream's values
#: (kp 1.1, 0.05 m/s, 0.35 rad/s) 144-145 s in 3 runs; speed 0.03 237 s.
LAP_LIMIT = 200.0
#: Standing still (less than STALL_STEP of the loop gained) this long ends the run.
STALL_S = 20.0
STALL_STEP = 0.01
#: The run ends after this long whatever happens.
RUN_LIMIT = 1.6 * LAP_LIMIT


class LapTracker:
    """Where a run is along the loop, fed one odometry sample at a time."""

    def __init__(self, track=None):
        self.track = track_points() if track is None else np.asarray(track)
        self.n = len(self.track)
        self.index = None
        self.steps = 0              # signed loop points gained since the start
        self.start_t = None
        self.reached = 0.0          # lap fraction reached while still on the line
        self.lost = False
        self.lap_time = None
        self.moved_t = None
        self.moved_steps = 0
        self.last_t = None

    def fraction(self):
        return abs(self.steps) / self.n

    def feed(self, x, y, t):
        distances = np.hypot(self.track[:, 0] - x, self.track[:, 1] - y)
        index = int(np.argmin(distances))
        self.last_t = t
        if self.index is None:
            self.index, self.start_t, self.moved_t = index, t, t
            return
        step = (index - self.index + self.n // 2) % self.n - self.n // 2
        self.index = index
        self.steps += step
        if abs(abs(self.steps) - self.moved_steps) >= STALL_STEP * self.n:
            self.moved_steps, self.moved_t = abs(self.steps), t
        if self.lost or self.lap_time is not None:
            return
        if float(distances[index]) > LOST_M:
            self.lost = True
            return
        self.reached = max(self.reached, self.fraction())
        if self.fraction() >= 1.0:
            self.lap_time = t - self.start_t

    def stalled(self, t):
        return self.moved_t is not None and t - self.moved_t > STALL_S

    def done(self, t):
        return (self.lost or self.lap_time is not None or self.stalled(t)
                or (self.start_t is not None and t - self.start_t > RUN_LIMIT))

    def summary(self):
        t = self.last_t if self.last_t is not None else 0.0
        return {'reached': self.reached, 'lost': self.lost, 'lap_time': self.lap_time,
                'stalled': self.stalled(t) and not self.lost and self.lap_time is None,
                'elapsed': 0.0 if self.start_t is None else t - self.start_t}


def evaluate(summary):
    """The three levels for a run's summary."""
    if summary['reached'] < QUARTER:
        return [Level(1, 'fail', summary), Level(2, 'skip', {'after': 1}),
                Level(3, 'skip', {'after': 1})]
    if summary['lap_time'] is None:
        return [Level(1, 'pass', summary), Level(2, 'fail', summary),
                Level(3, 'skip', {'after': 2})]
    return [Level(1, 'pass', summary), Level(2, 'pass', summary),
            Level(3, 'pass' if summary['lap_time'] <= LAP_LIMIT else 'fail', summary)]


_TITLES = {1: 'เกาะเส้นได้ (1/4 รอบ)', 2: 'วิ่งครบรอบ', 3: 'ครบรอบทันเวลา'}


def _why(n):
    if n['lost']:
        return f'หลุดจากเส้นที่ {n["reached"]:.0%} ของรอบ'
    if n['stalled']:
        return f'หุ่นหยุดนิ่งที่ {n["reached"]:.0%} ของรอบ'
    return f'หมดเวลาที่ {n["reached"]:.0%} ของรอบ'


def _timed_out(n):
    """On the line and moving, but the run's time was up before the lap."""
    return not n['lost'] and not n['stalled'] and n['lap_time'] is None


def format_report(levels):
    """Thai text for the terminal. Hints describe what the robot did, never a parameter."""
    lines = []
    for level in levels:
        head = f'ด่าน {level.number} {_TITLES[level.number]}:'
        n = level.numbers
        if level.status == 'skip':
            lines.append(f'{head} ยังไม่ตรวจ (ผ่านด่าน {n["after"]} ก่อน)')
        elif level.number == 3 and level.status == 'pass':
            lines.append(f'{head} ผ่าน ({n["lap_time"]:.0f} วินาที ต้องไม่เกิน {LAP_LIMIT:.0f})')
        elif level.status == 'pass':
            lines.append(f'{head} ผ่าน')
        elif level.number in (1, 2) and _timed_out(n):
            lines.append(f'{head} ไม่ผ่าน - {_why(n)}')
            lines.append(f'  คำใบ้: หุ่นยังเกาะเส้นอยู่แต่ช้าจนหมดเวลา ({RUN_LIMIT:.0f} วินาที) '
                         '- หุ่นเดินหน้าเร็วแค่ไหนตอนเกาะเส้น')
        elif level.number == 1:
            lines.append(f'{head} ไม่ผ่าน - {_why(n)}')
            if n['stalled'] and n['reached'] < 0.02:
                lines.append('  คำใบ้: หุ่นไม่ออกเดินเลย - ในหน้าต่าง image เห็นกรอบบนเส้นไหม? '
                             'ถ้าไม่เห็น หุ่นมองไม่เห็นเส้น (ช่วงสีของเส้นแคบไปหรือเลือกสีผิด)')
            else:
                lines.append('  คำใบ้: หุ่นเลี้ยวตามเส้นไม่ทันตั้งแต่โค้งแรกไหม? '
                             'ดูว่าหุ่นตอบสนองต่อเส้นที่เบี้ยวไปแรงแค่ไหน')
        elif level.number == 2:
            lines.append(f'{head} ไม่ผ่าน - {_why(n)}')
            lines.append('  คำใบ้: หลุดตรงโค้งที่หักแคบไหม? ตรงนั้นหุ่นต้องหมุนตัวเร็วกว่าโค้งอื่น '
                         'หุ่นหมุนได้เร็วสุดแค่ไหน')
        else:
            lines.append(f'{head} ไม่ผ่าน ({n["lap_time"]:.0f} วินาที ต้องไม่เกิน {LAP_LIMIT:.0f})')
            lines.append('  คำใบ้: วิ่งครบแล้วแต่ช้า หุ่นเดินหน้าเร็วแค่ไหนตอนเกาะเส้น')
    if all(level.status == 'pass' for level in levels):
        lines.append('ผ่านครบทุกด่าน!')
    return '\n'.join(lines)
