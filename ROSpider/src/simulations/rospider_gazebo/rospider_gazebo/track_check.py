"""Score the Color Tracking exercise (docs/superpowers/specs/2026-09-29-color-tracking-challenge-design.md).

scripts/check_track.py picks the red ball's colour in object_tracking (the ball's pixel at the
spawn), switches following on and feeds the odometry (the sim's odometry is the world pose) to a
tag_check.TagRun scored against the balls of worlds/track_challenge.sdf, until the robot has walked
and stood still. Pure Python (no rclpy), unit tested.

Measured with object_tracking (threshold 0.5, 0.05 m/s, 0.1 rad/s, sim seconds): stop_y 300 / 350 /
380 / 400 / 420 stopped 0.64 / 0.54 / 0.49 / 0.46 / 0.44 m from the red ball, facing it within
5 degrees, after 21 s whatever the stop_y; threshold 0.7 walked to the orange ball, 1.0 took in the
robot's own gripper and backed off; max_speed 0.01 took 74 s.
"""

from rospider_gazebo.slam_check import Level
from rospider_gazebo.tag_check import TagRun

#: Ball centres (x, y) in worlds/track_challenge.sdf; the robot must go to TARGET.
BALLS = {'red': (1.3, 0.45), 'orange': (0.9, -0.3)}
TARGET = 'red'
#: Where the red ball is in the picture at the spawn (object_tracking's look pose), px of 640x480:
#: the checker "clicks" there.
BALL_PIXEL = (143, 128)
#: Robot base to the ball, metres: close enough to reach for it, far enough to still see it.
DISTANCE_BAND = (0.43, 0.52)
FACING_DEG = 15.0
#: Level 1 also needs the robot at least this much closer to the ball than it started.
CLOSER_M = 0.2
#: Turned less than this and never walked: the robot did nothing (at the spawn the two balls sit
#: at nearly the same bearing, so the one it faces then says nothing).
TURNED_DEG = 10.0
#: Level 3: stopped within this many sim seconds (answer key 21 s).
TIME_LIMIT = 40.0

COLOURS = {'red': 'แดง', 'orange': 'ส้ม'}


def new_run():
    """A TagRun scored against the balls."""
    return TagRun(targets=BALLS, target=TARGET)


def evaluate(summary):
    """The three levels for a run's summary."""
    went = summary['faced'] == TARGET and summary['closer'] >= CLOSER_M
    if not went:
        return [Level(1, 'fail', summary), Level(2, 'skip', {'after': 1}),
                Level(3, 'skip', {'after': 1})]
    placed = (summary['stopped'] and DISTANCE_BAND[0] <= summary['distance'] <= DISTANCE_BAND[1]
              and summary['facing'] <= FACING_DEG)
    if not placed:
        return [Level(1, 'pass', summary), Level(2, 'fail', summary),
                Level(3, 'skip', {'after': 2})]
    return [Level(1, 'pass', summary), Level(2, 'pass', summary),
            Level(3, 'pass' if summary['seconds'] <= TIME_LIMIT else 'fail', summary)]


_TITLES = {1: f'เดินไปหาลูกบอลสี{COLOURS[TARGET]}', 2: 'หยุดหน้าลูกบอลพอดี', 3: 'ทันเวลา'}


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
                lines.append('  คำใบ้: หยุดถูกที่แล้วแต่ช้า หุ่นเดินเข้าหาลูกบอลเร็วแค่ไหน')
        elif level.status == 'pass':
            lines.append(f'{head} ผ่าน')
        elif level.number == 1:
            lines.extend(_level_one(head, n))
        elif not n['stopped']:
            lines.append(f'{head} ไม่ผ่าน - ยังไม่หยุดภายใน {n["elapsed"]:.0f} วินาที '
                         f'(ยังห่างลูกบอล {n["distance"]:.2f} ม.)')
            lines.append('  คำใบ้: หุ่นตามลูกบอลถูกลูกแล้วแต่เดินช้ามากจนไปไม่ถึง - หุ่นเดินเร็วแค่ไหน')
        else:
            lines.append(f'{head} ไม่ผ่าน - หยุดห่างลูกบอล {n["distance"]:.2f} ม. '
                         f'(ต้อง {DISTANCE_BAND[0]:.2f}-{DISTANCE_BAND[1]:.2f} ม.), '
                         f'หันเบี่ยงจากลูกบอล {n["facing"]:.0f} องศา')
            if n['distance'] > DISTANCE_BAND[1]:
                lines.append('  คำใบ้: หยุดห่างเกินไป - หุ่นหยุดเมื่อลูกบอลในภาพเลื่อนลงมาถึงจุดสีเหลือง '
                             'จุดนั้นอยู่สูงหรือต่ำแค่ไหนในภาพ')
            elif n['distance'] < DISTANCE_BAND[0]:
                lines.append('  คำใบ้: หยุดชิดเกินไป - หุ่นหยุดเมื่อลูกบอลในภาพเลื่อนลงมาถึงจุดสีเหลือง '
                             'จุดนั้นอยู่สูงหรือต่ำแค่ไหนในภาพ')
            else:
                lines.append('  คำใบ้: ระยะพอดีแต่ไม่ได้หันหน้าตรงลูกบอล - หุ่นหมุนตัวตามได้เร็วพอไหม')
    if all(level.status == 'pass' for level in levels):
        lines.append('ผ่านครบทุกด่าน!')
    return '\n'.join(lines)


def _level_one(head, n):
    if not n['moved'] and n['turned'] < TURNED_DEG:
        return [f'{head} ไม่ผ่าน - หุ่นไม่ขยับ',
                '  คำใบ้: ในหน้าต่าง image มีวงกลมล้อมลูกบอลไหม? ถ้าไม่มี หุ่นหาสีที่เลือกไม่เจอ']
    if n['faced'] != TARGET:
        verb = 'เดินไปหา' if n['moved'] else 'หันไปหา'
        return [f'{head} ไม่ผ่าน - หุ่น{verb}ลูกบอลสี{COLOURS[n["faced"]]}',
                '  คำใบ้: ในหน้าต่าง image วงกลมล้อมลูกไหน? ช่วงสีที่หุ่นจับกว้างจนลูกบอลสีอื่นเข้าข่ายไหม']
    return [f'{head} ไม่ผ่าน - หุ่นไม่ได้เข้าใกล้ลูกบอล (เข้าใกล้ขึ้น {n["closer"]:+.2f} ม.)',
            '  คำใบ้: หุ่นตามอะไรอยู่? ถ้าวงกลมในหน้าต่าง image ไปล้อมมือจับของหุ่นเอง '
            'ช่วงสีที่จับกว้างเกินไปจนรวมสีเทาเข้มของมือจับ']
