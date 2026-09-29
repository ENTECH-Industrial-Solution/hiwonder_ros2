"""The SLAM exercise's config: the participant's file over Hiwonder's slam.yaml.

Pure Python (no cv2: slam_challenge.launch.py imports this). On every run
the launch merges the participant's file (config/slam_challenge.yaml, or a
params:= path copied from it when missing) over slam/config/slam.yaml into a
temp file for slam_toolbox. Mistakes a participant can make would be silent
or baffling inside slam_toolbox, so they are caught here:
  - a misspelled key: slam_toolbox ignores unknown parameters;
  - a value of the wrong type or a blank one: rclcpp aborts the node
    (12 where the default is 12.0 is converted instead).
"""

import os
import shutil
import tempfile

import yaml


class ChallengeConfigError(Exception):
    """The participant's file cannot be used; the message says why, in Thai."""


def ensure_user_file(template, path):
    """Copy the starting config to `path` unless it is already there."""
    if os.path.exists(path):
        return False
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    shutil.copyfile(template, path)
    return True


def load_params(path):
    """The `/**: ros__parameters:` mapping of a slam_toolbox params file."""
    try:
        with open(path) as handle:
            data = yaml.safe_load(handle)
    except yaml.YAMLError as err:
        mark = getattr(err, 'problem_mark', None)
        where = f' บรรทัด {mark.line + 1}' if mark is not None else ''
        problem = getattr(err, 'problem', None) or err
        raise ChallengeConfigError(f'{path}{where}: รูปแบบ YAML ผิด ({problem})') from err
    except OSError as err:
        raise ChallengeConfigError(f'{path}: เปิดไฟล์ไม่ได้ ({err})') from err
    try:
        params = data['/**']['ros__parameters']
    except (TypeError, KeyError):
        params = None
    if not isinstance(params, dict):
        raise ChallengeConfigError(
            f"{path}: ต้องขึ้นต้นด้วย '/**:' ตามด้วย 'ros__parameters:' เหมือนไฟล์ตั้งต้น")
    return params


def merge(base, override, source):
    """`base` with `override`'s values; `source` names the file in errors."""
    merged = dict(base)
    for key, value in override.items():
        if key not in base:
            raise ChallengeConfigError(
                f"{source}: ไม่รู้จัก parameter '{key}' (พิมพ์ชื่อผิดหรือเปล่า?)")
        merged[key] = _typed(key, value, base[key], source)
    return merged


_KINDS = {bool: ' true/false', float: 'ตัวเลข', int: 'จำนวนเต็ม', str: 'ข้อความ'}


def _typed(key, value, default, source):
    """`value` as the type of Hiwonder's `default`: rclcpp aborts the node on
    a type mismatch, and an empty value (YAML null) is never a valid one."""
    def refuse(expected):
        raise ChallengeConfigError(f"{source}: '{key}' ต้องเป็น{expected} แต่ได้ {value!r}")

    is_number = isinstance(value, (int, float)) and not isinstance(value, bool)
    if isinstance(default, list):
        # Each element as the default's first; an empty list cannot be declared by rclpy.
        if not isinstance(value, list) or not value:
            refuse(' list ในวงเล็บ [ ] อย่างน้อยหนึ่งค่า')
        try:
            return [_typed(key, item, default[0], source) for item in value]
        except ChallengeConfigError:
            refuse(f' list ของ{_KINDS[type(default[0])]}')
    if isinstance(default, bool):
        if not isinstance(value, bool):
            refuse(' true หรือ false')
    elif isinstance(default, float):
        if not is_number:
            refuse('ตัวเลข')
        value = float(value)
    elif isinstance(default, int):
        if not isinstance(value, int) or isinstance(value, bool):
            refuse('จำนวนเต็ม')
    elif isinstance(default, str):
        if not isinstance(value, str) or not value:
            refuse('ข้อความ (เช่นชื่อ topic)')
    return value


def merged_params_file(base_path, user_path, fixed=None):
    """Write base <- fixed <- participant to a temp file; return its path.

    `fixed` holds values the exercise sets over Hiwonder's file without
    showing them to the participant, whose own file still wins.
    """
    base = load_params(base_path)
    if fixed:
        base = merge(base, fixed, 'slam_challenge.launch.py')
    params = merge(base, load_params(user_path), user_path)
    with tempfile.NamedTemporaryFile('w', prefix='slam_challenge_', suffix='.yaml',
                                     delete=False) as handle:
        yaml.safe_dump({'/**': {'ros__parameters': params}}, handle)
        return handle.name
