"""The V-SLAM exercises' participant files (docs/superpowers/specs/2026-09-28-vslam-challenge-design.md).

Both files have the other exercises' shape (/**: ros__parameters:). `camera_view` is a
friendly key: it becomes vslam.launch.py's arm_pose (the camera rides the arm; `init` tilts
it 52 degrees at the floor). In the mapping file every other key is an RTAB-Map parameter
already under rtabmap: in config/vslam.yaml; RTAB-Map declares its parameters as strings, so
a number typed without quotes is written as its text. In the navigation file every other key
is one of nav_params' friendly Nav2 keys. Pure Python (no rclpy): launch files import it.
"""

import copy
import tempfile

import yaml
from rospider_gazebo import challenge_params, maps, nav_params
from rospider_gazebo.challenge_params import ChallengeConfigError

CAMERA_VIEWS = {'floor': 'init', 'ahead': 'horizontal'}


def split_camera_view(params, source):
    """(arm_pose, the other keys). A missing camera_view means ahead."""
    rest = dict(params)
    view = rest.pop('camera_view', 'ahead')
    if not isinstance(view, str) or view not in CAMERA_VIEWS:
        raise ChallengeConfigError(
            f"{source}: 'camera_view' ต้องเป็น floor หรือ ahead แต่ได้ {view!r}")
    return CAMERA_VIEWS[view], rest


def _rtabmap_value(key, value, default, source):
    if not isinstance(default, str):
        return challenge_params._typed(key, value, default, source)
    if isinstance(value, bool) or value is None or value == '':
        raise ChallengeConfigError(
            f"{source}: '{key}' ต้องเป็นตัวเลขหรือข้อความ แต่ได้ {value!r}")
    return str(value) if isinstance(value, (int, float)) else value


def apply_rtabmap(vslam, overrides, source):
    """A copy of vslam.yaml's contents with `overrides` under rtabmap: ros__parameters:."""
    merged = copy.deepcopy(vslam)
    params = merged['rtabmap']['ros__parameters']
    for key, value in overrides.items():
        if key not in params:
            raise ChallengeConfigError(
                f"{source}: ไม่รู้จัก parameter '{key}' (พิมพ์ชื่อผิดหรือเปล่า?)")
        params[key] = _rtabmap_value(key, value, params[key], source)
    return merged


def _write(data, prefix):
    with tempfile.NamedTemporaryFile('w', prefix=prefix, suffix='.yaml', delete=False) as handle:
        yaml.safe_dump(data, handle)
        return handle.name


def mapping_params_file(vslam_path, user_path):
    """config/vslam.yaml with the participant's values -> (temp file, arm_pose)."""
    with open(vslam_path) as handle:
        vslam = yaml.safe_load(handle)
    arm_pose, rest = split_camera_view(challenge_params.load_params(user_path), user_path)
    return _write(apply_rtabmap(vslam, rest, user_path), 'vslam_challenge_'), arm_pose


def nav_params_file(nav2_path, user_path):
    """config/vslam_nav2_params.yaml through nav_params.apply -> (temp file, arm_pose)."""
    with open(nav2_path) as handle:
        nav2 = yaml.safe_load(handle)
    arm_pose, rest = split_camera_view(challenge_params.load_params(user_path), user_path)
    return _write(nav_params.apply(nav2, rest, user_path), 'vslam_nav_challenge_'), arm_pose


def require_map(value, workspace_maps, cwd):
    """The participant's V-SLAM database for `value`, or a Thai error naming the mapping exercise."""
    path = maps.find_map(value, '.db', workspace_maps, cwd)
    if path is None:
        raise ChallengeConfigError(
            f"ไม่พบแผนที่ V-SLAM '{value}' (หาใน {workspace_maps}) - ทำโจทย์สร้างแผนที่ก่อน: "
            f'ros2 launch rospider_gazebo vslam_challenge.launch.py map:={value} '
            'ขับให้ทั่วแล้วปิด launch (Ctrl+C)')
    return path
