"""Score the Nav2 exercise's course run (docs/superpowers/specs/2026-09-28-nav-challenge-design.md).

scripts/check_nav.py drives the course and hands the per-goal results here.
Pure Python (no rclpy), so the level logic and the Thai report are unit
tested. The checker stops at the first failed goal, so `results` may be
shorter than COURSE.
"""

from dataclasses import dataclass

from rospider_gazebo.slam_check import Level

# Measured in the maze room (sim seconds, RPP): answer key (max_vel_x 0.15) 34 + 106 + 69 = 209
# and 33 + 96 + 87 = 216; the starting max_vel_x 0.05 (Hiwonder's real-robot value)
# 43 + 210 + 190 = 443. COURSE_LIMIT sits ~85 s above the answer key. Each goal's timeout lets
# the slow run finish its leg (so slowness fails level 3, not level 2), except G1's, which is
# short so a robot circling its first goal is reported within two minutes.
COURSE_LIMIT = 300.0    # sim seconds for the whole course; the checker stops once it is passed
PLAN_TIMEOUT = 15.0     # wall seconds to wait for the planner's answer

#: (name, Thai label, (x, y) in the map frame = world frame, timeout in sim seconds).
#: The robot spawns at the origin.
COURSE = (
    ('G1', 'มุมห้อง A', (-0.4, -1.3), 120.0),
    ('G2', 'มุมซ้ายบนห้อง B (ผ่านประตู ทางเดิน และทรงกระบอก)', (3.5, 1.5), 300.0),
    ('G3', 'กลับจุดเริ่ม', (0.0, 0.0), 300.0),
)


@dataclass
class GoalResult:
    name: str
    status: str         # succeeded, no_path, aborted, timeout, over_limit, canceled, rejected
    seconds: float


def evaluate(results):
    """The three levels for a course run."""
    def done(i):
        return len(results) > i and results[i].status == 'succeeded'

    def result(i):
        return results[i] if len(results) > i else None

    if not done(0):
        return [Level(1, 'fail', {'result': result(0)}),
                Level(2, 'skip', {'after': 1}), Level(3, 'skip', {'after': 1})]
    if not done(1):
        return [Level(1, 'pass', {'result': result(0)}),
                Level(2, 'fail', {'result': result(1)}), Level(3, 'skip', {'after': 2})]
    total = sum(r.seconds for r in results)
    passed = done(2) and total <= COURSE_LIMIT
    return [Level(1, 'pass', {'result': result(0)}), Level(2, 'pass', {'result': result(1)}),
            Level(3, 'pass' if passed else 'fail', {'result': result(2), 'total': total})]


_TITLES = {1: 'ถึงเป้าหมายแรก', 2: 'ผ่านประตูไปห้อง B', 3: 'วิ่งครบทันเวลา'}


def _failure(result):
    """Why a goal failed, and a hint that names the symptom, not the parameter."""
    if result is None:
        return 'ยังไม่ได้วิ่ง', 'ตัวตรวจหยุดก่อนถึงจุดนี้ - ลองตรวจใหม่อีกครั้ง'
    if result.status == 'no_path':
        return ('Nav2 หาเส้นทางไปไม่ได้',
                'ใน RViz ดู costmap ช่องทางที่ต้องผ่านตันไหม? '
                'หุ่นในแผนที่ตัวใหญ่แค่ไหนเทียบกับช่องประตู')
    if result.status == 'over_limit':
        return (f'เกินเวลารวม {COURSE_LIMIT:.0f} วินาทีระหว่างทาง',
                'หุ่นเดินช้าไปไหม? ดูว่ามันเดินเร็วแค่ไหนตอนวิ่งทางตรง')
    if result.status in ('timeout', 'aborted'):
        return (f'ไปไม่ถึงหรือไม่ยอมจบ ({result.seconds:.0f} วินาที)',
                'หุ่นไปถึงแถวเป้าแล้วแต่วนอยู่ไม่ยอมหยุดไหม? '
                'Nav2 ต้องเข้าใกล้เป้าแค่ไหนถึงนับว่าถึงแล้ว')
    return ('Nav2 ไม่รับหรือยกเลิกเป้าหมาย',
            'ปิด-เปิด launch ใหม่ รอให้ RViz ขึ้นแผนที่ แล้วตรวจอีกครั้ง')


def format_report(levels):
    """Thai text for the terminal."""
    lines = []
    for level in levels:
        head = f'ด่าน {level.number} {_TITLES[level.number]}:'
        n = level.numbers
        if level.status == 'skip':
            lines.append(f'{head} ยังไม่ตรวจ (ผ่านด่าน {n["after"]} ก่อน)')
        elif level.number == 3 and level.status == 'fail' and n['result'] is not None \
                and n['result'].status == 'succeeded':
            lines.append(f'{head} ไม่ผ่าน (วิ่งครบ {n["total"]:.0f} วินาที '
                         f'ต้องไม่เกิน {COURSE_LIMIT:.0f})')
            lines.append('  คำใบ้: หุ่นเดินช้าไปไหม? ดูว่ามันเดินเร็วแค่ไหนตอนวิ่งทางตรง')
        elif level.status == 'fail':
            why, hint = _failure(n['result'])
            lines.append(f'{head} ไม่ผ่าน - {why}')
            lines.append(f'  คำใบ้: {hint}')
        elif level.number == 3:
            lines.append(f'{head} ผ่าน ({n["total"]:.0f} วินาที ต้องไม่เกิน {COURSE_LIMIT:.0f})')
        else:
            lines.append(f'{head} ผ่าน ({n["result"].seconds:.0f} วินาที)')
    if all(level.status == 'pass' for level in levels):
        lines.append('ผ่านครบทุกด่าน!')
    return '\n'.join(lines)
