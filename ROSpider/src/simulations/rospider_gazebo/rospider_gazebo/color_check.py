"""Score the Color Threshold exercise (docs/superpowers/specs/2026-09-29-color-threshold-challenge-design.md).

scripts/check_color.py takes one camera frame of worlds/color_challenge.sdf and hands it here
with the participant's LAB bands. find_blobs is color_detect's own pipeline (LAB, 3x3 blur,
inRange, erode+dilate, external contours of at least min_area_px), so what the checker counts is
what the LAB_Tool window's mask shows. Unit tested against a frame recorded from the sim.

The robot spawns at the origin with the arm at `init`, so the camera always sees the scene the
same way; the target boxes below are the answer-key blobs' bounding boxes in that frame
(test/data/color_challenge.png) plus MARGIN_PX.
"""

import json
import os

import cv2
import numpy as np
import yaml
from rospider_gazebo import lab_settings
from rospider_gazebo.slam_check import Level

MARGIN_PX = 20


def _box(x0, y0, x1, y1):
    return (x0 - MARGIN_PX, y0 - MARGIN_PX, x1 + MARGIN_PX, y1 + MARGIN_PX)


#: Target name -> (colour, pixel box). Answer-key blobs in the recorded frame: red
#: (88, 225, 211, 338), green (205, 133, 290, 233), lit blue (268, 311, 377, 439), blue in
#: the block's shadow (353, 134, 438, 234); the orange decoy is at x 432-557, y 225-340.
TARGETS = {
    'red': ('red', _box(88, 225, 211, 338)),
    'green': ('green', _box(205, 133, 290, 233)),
    'blue': ('blue', _box(268, 311, 377, 439)),
    'blue_shade': ('blue', _box(353, 134, 438, 234)),
}
LIT = ('red', 'green', 'blue')
#: The colours scored; others added in LAB_Tool's Color list are ignored and named.
SCORED = ('red', 'green', 'blue')
SHADE = 'blue_shade'
_THAI = {'red': 'แดง', 'green': 'เขียว', 'blue': 'น้ำเงิน'}


def find_blobs(frame, settings):
    """{colour: [(area, (cx, cy), (x0, y0, x1, y1))]} for every blob color_detect would report."""
    lab = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2LAB), (3, 3), 3)
    size = max(1, int(settings['kernel_px']) | 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (size, size))
    blobs = {}
    for color in settings['colors']:
        lo, hi = lab_settings.band(settings, color)
        mask = cv2.inRange(lab, np.array(lo, dtype=np.uint8), np.array(hi, dtype=np.uint8))
        mask = cv2.dilate(cv2.erode(mask, kernel), kernel)
        found = []
        for contour in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]:
            area = cv2.contourArea(contour)
            moments = cv2.moments(contour)
            if area < int(settings['min_area_px']) or moments['m00'] == 0:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            found.append((area, (moments['m10'] / moments['m00'], moments['m01'] / moments['m00']),
                          (x, y, x + w, y + h)))
        blobs[color] = found
    return blobs


def _within(rect, box):
    """The whole blob inside the box: a band that whitens half the frame is not 'found'."""
    return box[0] <= rect[0] and box[1] <= rect[1] and rect[2] <= box[2] and rect[3] <= box[3]


def _covers(rect, box):
    """The blob spills over the whole target box: the band is too wide, not too narrow."""
    return rect[0] <= box[0] + MARGIN_PX and rect[1] <= box[1] + MARGIN_PX and \
        rect[2] >= box[2] - MARGIN_PX and rect[3] >= box[3] - MARGIN_PX


def _found(blobs, target):
    color, box = TARGETS[target]
    return any(_within(rect, box) for _, _, rect in blobs.get(color, []))


def evaluate(blobs):
    """The three levels for the blobs of one frame (only SCORED colours count)."""
    missing, wide, absent = [], [], []
    for name in LIT:
        if _found(blobs, name):
            continue
        color, box = TARGETS[name]
        if color not in blobs:
            absent.append(color)
        elif any(_covers(rect, box) for _, _, rect in blobs[color]):
            wide.append(color)
        else:
            missing.append(color)
    if missing or wide or absent:
        return [Level(1, 'fail', {'missing': missing, 'wide': wide, 'absent': absent}),
                Level(2, 'skip', {'after': 1}), Level(3, 'skip', {'after': 1})]
    extra = [(color, centre) for color in SCORED for _, centre, rect in blobs.get(color, [])
             if not any(_within(rect, box) for c, box in TARGETS.values() if c == color)]
    if extra:
        return [Level(1, 'pass', {}), Level(2, 'fail', {'extra': extra}),
                Level(3, 'skip', {'after': 2})]
    shade = _found(blobs, SHADE)
    return [Level(1, 'pass', {}), Level(2, 'pass', {}),
            Level(3, 'pass' if shade else 'fail', {})]


def ignored_colors(settings):
    return [color for color in settings['colors'] if color not in SCORED]


def scene_problem(frame):
    """A Thai reason this frame is not the recorded view, or None. The answer-key bands must
    pass on it: a robot nudged 2 cm or turned 3 degrees, the arm elsewhere, another world or
    camera size would otherwise be scored as a band mistake."""
    if frame.shape[:2] != (480, 640):
        return f'ภาพจากกล้องขนาด {frame.shape[1]}x{frame.shape[0]} ไม่ใช่ 640x480 ของโจทย์นี้'
    answer = evaluate(find_blobs(frame, lab_settings.defaults()))
    if not all(level.status == 'pass' for level in answer):
        return ('กล้องไม่ได้เห็นฉากของโจทย์ตามตำแหน่งเริ่มต้น (หุ่นหรือแขนขยับ หรือเปิดห้องอื่นอยู่) '
                '- ปิดแล้วเปิด color_challenge.launch.py ใหม่ โดยไม่ขับหุ่น')
    return None


def settings_from_json(text):
    """color_detect's live settings as it publishes them on ~/settings."""
    base = {key: value for key, value in lab_settings.defaults().items()
            if key in ('min_area_px', 'kernel_px')}
    base['colors'] = []
    return lab_settings.merge(base, json.loads(text))


def frame_problem(frame):
    """A Thai reason the frame cannot be scored, or None."""
    if frame is None or frame.size == 0:
        return 'ไม่ได้ภาพจากกล้อง'
    if float(frame.mean()) < 20.0:
        return 'ภาพจากกล้องมืดสนิท - รอให้ Gazebo ขึ้นภาพก่อนแล้วตรวจใหม่'
    return None


def load_settings(yaml_path, tuned_path):
    """The bands color_detect would use: the participant's file, then the tuned JSON over it."""
    with open(yaml_path) as handle:
        params = yaml.safe_load(handle)['color_detect']['ros__parameters']
    found = {'min_area_px': params.get('min_area_px', 300), 'kernel_px': params.get('kernel_px', 5),
             'colors': list(params.get('colors', ['red', 'green', 'blue']))}
    for color in found['colors']:
        found[color] = params[color]
    base = {key: value for key, value in lab_settings.defaults().items()
            if key in ('min_area_px', 'kernel_px')}
    base['colors'] = []
    settings = lab_settings.merge(base, found)
    if os.path.isfile(os.path.expanduser(tuned_path)):
        with open(os.path.expanduser(tuned_path)) as handle:
            settings = lab_settings.merge(settings, json.load(handle))
    return settings


_TITLES = {1: 'เจอครบทุกสี', 2: 'ไม่จับของหลอก', 3: 'เจอแม้อยู่ในเงา'}


def format_report(levels, ignored=()):
    """Thai text for the terminal. Hints describe what the mask shows, never a parameter."""
    lines = []
    if ignored:
        lines.append(f'(ไม่ได้ตรวจสี {", ".join(ignored)} เพราะไม่ได้อยู่ในโจทย์ '
                     '- ลบออกจาก Color list ได้)')
    for level in levels:
        head = f'ด่าน {level.number} {_TITLES[level.number]}:'
        n = level.numbers
        if level.status == 'skip':
            lines.append(f'{head} ยังไม่ตรวจ (ผ่านด่าน {n["after"]} ก่อน)')
        elif level.status == 'pass':
            lines.append(f'{head} ผ่าน')
        elif level.number == 1:
            lines.append(f'{head} ไม่ผ่าน')
            for color in n['absent']:
                lines.append(f'  ไม่มีสี{_THAI[color]}ใน Color list (ถูกลบไป?) - เพิ่มกลับ '
                             'หรือเริ่มโจทย์ใหม่')
            for color in n['wide']:
                lines.append(f'  mask ของสี{_THAI[color]}ขาวไปทั้งพื้น ไม่ใช่แค่ที่กล่อง - '
                             'ช่วงของสีนั้นกว้างเกินไป บีบให้เหลือแต่กล่อง')
            if n['missing']:
                names = ', '.join(_THAI[color] for color in n['missing'])
                lines.append(f'  ไม่เจอกล่องสี{names}')
                lines.append('  คำใบ้: ใน mask ของสีนั้นกล่องขึ้นเป็นสีขาวไหม? ถ้าไม่ขึ้นเลย '
                             'ช่วงของสีนั้นแคบเกินไปหรือไปอยู่ผิดที่')
        elif level.number == 2:
            names = ', '.join(sorted({_THAI.get(color, color) for color, _ in n['extra']}))
            lines.append(f'{head} ไม่ผ่าน - mask ของสี{names} ไปติดวัตถุอื่นด้วย '
                         f'({len(n["extra"])} จุด)')
            lines.append('  คำใบ้: ดู mask ของสีนั้นว่ามีกล่องอื่นขึ้นมาด้วยไหม เช่นกล่องที่สีคล้ายกัน '
                         '- ช่วงของสีกว้างเกินไป บีบให้แคบลงจนเหลือแต่กล่องที่ต้องการ')
        else:
            lines.append(f'{head} ไม่ผ่าน - ไม่เจอกล่องสีน้ำเงินที่อยู่ในเงา')
            lines.append('  คำใบ้: กล่องในเงามืดกว่ากล่องที่โดนแดด ช่วงความสว่างของสีนั้นลงไปถึงมืดพอไหม?')
    if all(level.status == 'pass' for level in levels):
        lines.append('ผ่านครบทุกด่าน!')
    return '\n'.join(lines)
