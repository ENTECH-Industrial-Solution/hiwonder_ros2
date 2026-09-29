"""Score the 3D Shape Recognition exercise (docs/superpowers/specs/2026-09-29-shape-recognition-challenge-design.md).

scripts/object_classification.py publishes what it recognised on ~/objects (JSON: the settings in
use, the nearest and median depth in the region of interest, every object's kind and pixel box,
and the index of the target -- the nearest object of a wanted shape). scripts/check_shape.py hands
one such report to evaluate(). Pure Python, unit tested.

EXPECTED are the answer key's boxes in the camera view of worlds/shape_challenge.sdf (robot at the
spawn, object_classification's look pose); a recognised object counts as one of them only when
its whole box lies inside that box grown by MARGIN_PX, so the floor recognised as one big object
never passes for the cylinder its box happens to surround.
"""

from rospider_gazebo.challenge_params import ChallengeConfigError
from rospider_gazebo.slam_check import Level

FRAME = (640, 480)
KINDS = ('sphere', 'cuboid', 'cylinder')
#: The solid the exercise asks the robot to pick out.
TARGET = 'sphere'
#: Answer key's boxes (x, y, w, h), measured live with config/shape_challenge_solved.yaml.
EXPECTED = {'cylinder': (294, 201, 57, 58), 'cuboid': (162, 201, 65, 58),
            'sphere': (331, 21, 52, 55)}
MARGIN_PX = 20
#: A box covering this share of the region of interest is the floor taken for an object.
FLOOR_SHARE = 0.5

THAI = {'sphere': 'ลูกบอลสีแดง (ทรงกลม)', 'cuboid': 'กล่องสีเขียว (ทรงสี่เหลี่ยม)',
        'cylinder': 'ทรงกระบอกสีน้ำเงิน'}
WHERE = {'sphere': 'ขอบบนของภาพ', 'cuboid': 'ล่างซ้ายของกลางภาพ', 'cylinder': 'กลางภาพ'}


def validate(values, source):
    """Refuse values the node would take but that cannot mean anything."""
    def refuse(key, why):
        raise ChallengeConfigError(f"{source}: '{key}' {why}")

    if values['plane_distance'] <= 0:
        refuse('plane_distance', 'ต้องมากกว่า 0 (มิลลิเมตร)')
    roi = values['roi']
    if len(roi) != 4:
        refuse('roi', 'ต้องมี 4 ค่า [y_min, y_max, x_min, x_max]')
    y0, y1, x0, x1 = roi
    if not (0 <= y0 < y1 <= FRAME[1] and 0 <= x0 < x1 <= FRAME[0]):
        refuse('roi', f'ต้องเป็น [y_min, y_max, x_min, x_max] ในภาพขนาด {FRAME[0]}x{FRAME[1]} '
                      '(ค่า min น้อยกว่า max)')
    unknown = [shape for shape in values['shapes'] if shape not in KINDS]
    if unknown:
        refuse('shapes', f'รู้จักแค่ {", ".join(KINDS)} (ได้ {", ".join(map(str, unknown))})')


def _inside(box, expected):
    x, y, w, h = box
    ex, ey, ew, eh = expected
    return (x >= ex - MARGIN_PX and y >= ey - MARGIN_PX
            and x + w <= ex + ew + MARGIN_PX and y + h <= ey + eh + MARGIN_PX)


def _match(report):
    """({expected name: object index}, [indices of objects that are none of them], floor seen)."""
    y0, y1, x0, x1 = report['roi']
    roi_area = max(1, (y1 - y0) * (x1 - x0))
    found, extras, floor = {}, [], False
    for index, obj in enumerate(report['objects']):
        _, _, w, h = obj['box']
        if w * h >= FLOOR_SHARE * roi_area:
            floor = True
            extras.append(index)
            continue
        name = next((n for n, box in EXPECTED.items() if n not in found and _inside(obj['box'], box)),
                    None)
        if name is None:
            extras.append(index)
        else:
            found[name] = index
    return found, extras, floor


def evaluate(report):
    """The three levels for one ~/objects report."""
    found, extras, floor = _match(report)
    objects = report['objects']
    numbers = {'near': report['near'], 'median': report['median'], 'floor': floor,
               'extras': len(extras), 'seen': len(objects)}
    if floor or not found:
        return [Level(1, 'fail', numbers), Level(2, 'skip', {'after': 1}),
                Level(3, 'skip', {'after': 1})]
    missing = [name for name in KINDS if name not in found]
    misnamed = [(name, objects[index]['kind']) for name, index in found.items()
                if objects[index]['kind'] != name]
    numbers.update(missing=missing, misnamed=misnamed)
    if missing or misnamed or extras:
        return [Level(1, 'pass', numbers), Level(2, 'fail', numbers),
                Level(3, 'skip', {'after': 2})]
    target = report['target']
    numbers['chosen'] = None if target is None else objects[target]['kind']
    ok = target is not None and found[TARGET] == target
    return [Level(1, 'pass', numbers), Level(2, 'pass', numbers),
            Level(3, 'pass' if ok else 'fail', numbers)]


_TITLES = {1: 'แยกวัตถุออกจากพื้น', 2: 'เห็นครบและเรียกชื่อถูก', 3: f'เลือก{THAI[TARGET]}'}


def format_report(levels):
    """Thai text for the terminal. Hints describe what the robot saw, never a parameter."""
    lines = []
    for level in levels:
        head = f'ด่าน {level.number} {_TITLES[level.number]}:'
        n = level.numbers
        if level.status == 'skip':
            lines.append(f'{head} ยังไม่ตรวจ (ผ่านด่าน {n["after"]} ก่อน)')
        elif level.status == 'pass':
            lines.append(f'{head} ผ่าน')
        elif level.number == 1:
            lines.extend(_level_one(head, n))
        elif level.number == 2:
            lines.extend(_level_two(head, n))
        else:
            chosen = 'ไม่ได้เลือกชิ้นไหน' if n['chosen'] is None else f'เลือก{THAI[n["chosen"]]}'
            lines.append(f'{head} ไม่ผ่าน - หุ่น{chosen} (กรอบสีแดงในหน้าต่าง depth)')
            lines.append('  คำใบ้: หุ่นเลือกชิ้นที่ใกล้กล้องที่สุดในบรรดารูปทรงที่มันสนใจ '
                         '- หุ่นสนใจรูปทรงอะไรบ้าง')
    if all(level.status == 'pass' for level in levels):
        lines.append('ผ่านครบทุกด่าน!')
    return '\n'.join(lines)


def _level_one(head, n):
    if n['floor']:
        return [f'{head} ไม่ผ่าน - หุ่นเห็นพื้นทั้งแผ่นเป็นวัตถุชิ้นใหญ่ชิ้นเดียว',
                f'  คำใบ้: ในกรอบค้นหา พื้นอยู่ห่างกล้องราว {n["median"]:.0f} มม. '
                '- หุ่นคิดว่าพื้นอยู่ไกลแค่ไหน? (อะไรที่ไม่ใกล้กว่าพื้นจะถูกนับเป็นพื้น)']
    if n['seen']:
        return [f'{head} ไม่ผ่าน - เห็น {n["seen"]} ชิ้นแต่ไม่ใช่วัตถุบนพื้นเลย '
                f'(สิ่งที่ใกล้กล้องที่สุดห่าง {n["near"]:.0f} มม.)',
                '  คำใบ้: ขาหุ่นอยู่ในกรอบค้นหาไหม (ขอบล่างซ้าย/ขวาของภาพ)? '
                'หุ่นสนใจเฉพาะสิ่งที่อยู่ใกล้กล้องพอๆ กับสิ่งที่ใกล้ที่สุด']
    return [f'{head} ไม่ผ่าน - ไม่เห็นวัตถุเลย',
            f'  คำใบ้: สิ่งที่ใกล้กล้องที่สุดในกรอบห่าง {n["near"]:.0f} มม. พื้นห่างราว '
            f'{n["median"]:.0f} มม. - หุ่นคิดว่าพื้นอยู่ตรงไหน ใกล้กว่ายอดวัตถุหรือเปล่า']


def _level_two(head, n):
    lines = []
    problems = [f'มองไม่เห็น{THAI[name]} ({WHERE[name]})' for name in n['missing']]
    problems += [f'{THAI[name]} ถูกเรียกเป็น{THAI[kind]}' for name, kind in n['misnamed']]
    if n['extras']:
        problems.append(f'เห็นสิ่งที่ไม่ใช่วัตถุอีก {n["extras"]} ชิ้น')
    lines.append(f'{head} ไม่ผ่าน - ' + ', '.join(problems))
    if n['missing']:
        lines.append('  คำใบ้: หุ่นค้นหาเฉพาะในกรอบสีฟ้า (ภาพสีครึ่งขวาของหน้าต่าง) '
                     '- กรอบครอบวัตถุทุกชิ้นไหม')
    if n['misnamed']:
        lines.append('  คำใบ้: หุ่นดูรูปทรงจากขอบและความลึกของวัตถุทั้งชิ้น - วัตถุถูกขอบกรอบตัด '
                     'หรือส่วนล่างของมันถูกนับเป็นพื้นไปหรือเปล่า')
    if n['extras']:
        lines.append('  คำใบ้: แถบพื้นที่ขอบภาพ = พื้นบางส่วนใกล้กว่าที่หุ่นคิด; '
                     'ขาหุ่น = กรอบค้นหากว้างไปถึงขา')
    return lines
