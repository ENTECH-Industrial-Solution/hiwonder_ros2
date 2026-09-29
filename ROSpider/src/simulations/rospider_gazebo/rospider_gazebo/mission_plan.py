"""The final mission's file (config/mission_challenge.yaml) -> a speed and a list of steps.

Refuses, with a Thai message naming the step's number, anything the runner could not execute:
unknown keys or steps, wrong types, a colour that does not exist, a place that is not three
numbers. It checks nothing about whether the plan makes sense -- that is the exercise.
Pure Python, unit tested.
"""

import math
from typing import NamedTuple

import yaml
from rospider_gazebo.challenge_params import ChallengeConfigError
from rospider_gazebo.grasp_check import parse_point

SPEED_RANGE = (0.05, 0.20)
COLORS = ('red', 'green', 'blue')
KINDS = ('go_to', 'go_to_tag', 'pick', 'place')


class Step(NamedTuple):
    kind: str
    value: object


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _step(index, item, source):
    where = f'{source} ขั้นที่ {index}'
    if not isinstance(item, dict) or len(item) != 1:
        raise ChallengeConfigError(
            f'{where}: แต่ละขั้นต้องเป็นคำสั่งเดียว เช่น "- pick: blue" (ได้ {item!r})')
    (kind, value), = item.items()
    if kind not in KINDS:
        raise ChallengeConfigError(f"{where}: ไม่รู้จักคำสั่ง '{kind}' (มี {', '.join(KINDS)})")
    if kind == 'go_to':
        if not (isinstance(value, list) and len(value) == 3 and all(map(_number, value))):
            raise ChallengeConfigError(
                f"{where}: 'go_to' ต้องเป็น [x, y, องศาที่หัน] เช่น [3.8, 0.7, 90] (ได้ {value!r})")
        x, y, yaw = (float(v) for v in value)
        return Step(kind, (x, y, math.radians(yaw)))
    if kind == 'go_to_tag':
        if not (isinstance(value, int) and not isinstance(value, bool) and value >= 0):
            raise ChallengeConfigError(f"{where}: 'go_to_tag' ต้องเป็นหมายเลขป้าย เช่น 3 (ได้ {value!r})")
        return Step(kind, value)
    if kind == 'pick':
        if value not in COLORS:
            raise ChallengeConfigError(
                f"{where}: 'pick' ต้องเป็นสี {', '.join(COLORS)} (ได้ {value!r})")
        return Step(kind, value)
    if not isinstance(value, str):
        raise ChallengeConfigError(
            f"{where}: 'place_point' ต้องเป็นข้อความในเครื่องหมายคำพูด เช่น '0.16 0.0 0.035' (ได้ {value!r})")
    return Step(kind, parse_point(value, where))


def parse(data, source):
    """(speed, steps) from the file's mapping."""
    if not isinstance(data, dict):
        raise ChallengeConfigError(f'{source}: ต้องมี speed: และ steps: เหมือนไฟล์ตั้งต้น')
    unknown = [key for key in data if key not in ('speed', 'steps')]
    if unknown:
        raise ChallengeConfigError(f"{source}: ไม่รู้จัก '{unknown[0]}' (มีแค่ speed และ steps)")
    speed = data.get('speed')
    if not _number(speed) or not SPEED_RANGE[0] <= speed <= SPEED_RANGE[1]:
        raise ChallengeConfigError(
            f"{source}: 'speed' ต้องเป็นตัวเลข {SPEED_RANGE[0]}-{SPEED_RANGE[1]} (เมตร/วินาที) "
            f"ได้ {speed!r}")
    steps = data.get('steps')
    if not isinstance(steps, list) or not steps:
        raise ChallengeConfigError(f"{source}: 'steps' ต้องเป็นรายการคำสั่งอย่างน้อยหนึ่งขั้น")
    return float(speed), [_step(i, item, source) for i, item in enumerate(steps, 1)]


def load(path):
    """(speed, steps) from the file at `path`."""
    try:
        with open(path) as handle:
            data = yaml.safe_load(handle)
    except yaml.YAMLError as err:
        mark = getattr(err, 'problem_mark', None)
        where = f' บรรทัด {mark.line + 1}' if mark is not None else ''
        raise ChallengeConfigError(f'{path}{where}: รูปแบบ YAML ผิด') from err
    except OSError as err:
        raise ChallengeConfigError(f'{path}: เปิดไฟล์ไม่ได้ ({err})') from err
    return parse(data, path)


def describe(step):
    """The step as one Thai line, for the runner's progress output."""
    if step.kind == 'go_to':
        x, y, yaw = step.value
        return f'go_to เดินไป ({x:.2f}, {y:.2f}) หันไป {math.degrees(yaw):.0f} องศา'
    if step.kind == 'go_to_tag':
        return f'go_to_tag เดินไปหยุดหน้าป้ายหมายเลข {step.value}'
    if step.kind == 'pick':
        return f'pick หยิบบล็อกสี {step.value} แล้วถอยออกมา'
    x, y, z = step.value
    return f'place วางที่ ({x:.2f}, {y:.2f}, {z:.3f}) นับจากตัวหุ่น'
