import math

import cv2
import numpy as np
import pytest
from rospider_gazebo import line_follower
from rospider_gazebo.color_picker import lab_band
from rospider_gazebo.line_follower import LineControl, ObstacleStop, find_line

FLOOR = (190, 190, 185)              # BGR, the room's grey floor
LINE = (20, 20, 20)
W, H = 640, 480


def _lab(bgr):
    pixel = np.array([[bgr]], dtype=np.uint8)
    return tuple(int(v) for v in cv2.cvtColor(pixel, cv2.COLOR_BGR2LAB)[0, 0])


def _floor(line_x=None, width=24):
    """A floor frame with a vertical line at column `line_x`, if given."""
    frame = np.empty((H, W, 3), dtype=np.uint8)
    frame[:] = FLOOR
    if line_x is not None:
        frame[:, line_x - width // 2:line_x + width // 2] = LINE
    return frame


BAND = lab_band(_lab(LINE), 0.5)
AIM_X = W / 1.8


# ------------------------------------------------------------ find_line

def test_no_line_gives_no_angle_and_no_boxes():
    angle, boxes = find_line(_floor(), *BAND)
    assert angle is None
    assert boxes == []


def test_a_line_is_found_in_every_roi_strip():
    angle, boxes = find_line(_floor(320), *BAND)
    assert angle is not None
    assert len(boxes) == len(line_follower.ROIS)
    for (box, (cx, cy)), roi in zip(boxes, line_follower.ROIS):
        assert abs(cx - 320) < 3
        # Boxes are in frame pixels: each sits inside its own strip.
        assert roi[0] * H - 1 <= cy <= roi[1] * H + 1
        assert box.shape == (4, 2)


def test_angle_is_upstreams_atan_of_the_offset_from_w_over_1_8():
    angle, _ = find_line(_floor(320), *BAND)
    assert angle == pytest.approx(-math.atan((320 - AIM_X) / (H / 2)), abs=0.01)


def test_line_to_the_right_of_the_aim_point_gives_a_negative_angle():
    right, _ = find_line(_floor(500), *BAND)
    left, _ = find_line(_floor(100), *BAND)
    assert right < 0 < left


def test_lower_strips_weigh_more():
    """Upstream weights the three strips 0.7 / 0.2 / 0.1, nearest first."""
    frame = _floor()
    frame[int(0.81 * H):int(0.83 * H), 100:124] = LINE       # nearest strip
    frame[int(0.57 * H):int(0.59 * H), 500:524] = LINE       # farthest strip
    angle, boxes = find_line(frame, *BAND)
    assert len(boxes) == 2
    centre = (0.7 * 112 + 0.1 * 512) / 1.0
    assert angle == pytest.approx(-math.atan((centre - AIM_X) / (H / 2)), abs=0.02)


def test_specks_smaller_than_min_area_are_ignored():
    frame = _floor()
    frame[int(0.81 * H) + 2:int(0.81 * H) + 5, 300:303] = LINE
    angle, boxes = find_line(frame, *BAND)
    assert angle is None and boxes == []


# ------------------------------------------------------------ LineControl

def test_centred_line_stops_the_robot_as_upstream_does():
    control = LineControl()
    linear, angular = control.update(0.0)
    assert (linear, angular) == (0.0, 0.0)


def test_line_to_the_right_turns_clockwise_while_walking():
    control = LineControl()
    linear, angular = control.update(-0.3)
    assert linear == 0.05
    assert angular == pytest.approx(-1.1 * 0.3 / 5)
    linear, angular = control.update(0.3)
    assert angular > 0


def test_turn_rate_is_clamped_to_upstreams_ceiling():
    control = LineControl()
    _, angular = control.update(-3.0)
    assert angular == -0.35


def test_clear_resets_the_pid():
    control = LineControl()
    control.update(0.5)
    control.clear()
    assert control.pid.output == 0.0


# ------------------------------------------------------------ ObstacleStop

def _scan(front=None, behind=None, n=360):
    """Ranges for a scan with angle_min -pi, one reading per degree."""
    ranges = [5.0] * n
    inc = 2 * math.pi / n
    if front is not None:
        ranges[int((0 + math.pi) / inc)] = front
    if behind is not None:
        ranges[0] = behind
    return ranges, -math.pi, inc


def test_something_close_in_front_stops():
    stop = ObstacleStop()
    assert stop.update(*_scan(front=0.3)) is True


def test_something_close_behind_does_not():
    stop = ObstacleStop()
    assert stop.update(*_scan(behind=0.3)) is False


def test_stop_releases_after_five_clear_scans():
    stop = ObstacleStop()
    stop.update(*_scan(front=0.3))
    for _ in range(5):
        assert stop.update(*_scan()) is True
    assert stop.update(*_scan()) is False


def test_a_scan_with_nothing_finite_in_front_keeps_the_last_answer():
    stop = ObstacleStop()
    stop.update(*_scan(front=0.3))
    ranges, amin, inc = _scan()
    ranges = [math.inf] * len(ranges)
    assert stop.update(ranges, amin, inc) is True


def test_reading_zero_is_no_reading():
    """Upstream drops zeros: the driver reports a miss as 0."""
    stop = ObstacleStop()
    assert stop.update(*_scan(front=0.0)) is False


def test_line_control_takes_gain_speed_and_turn_limit():
    control = LineControl(kp=2.2, speed=0.1, max_turn=0.2)
    linear, angular = control.update(0.6)
    assert linear == pytest.approx(0.1)
    assert angular == pytest.approx(0.2)           # 2.2 * 0.6 / 5 = 0.264, clamped to 0.2
    control = LineControl(kp=2.2, speed=0.1, max_turn=0.5)
    assert control.update(0.3)[1] == pytest.approx(2.2 * 0.3 / 5)


def test_line_control_defaults_are_upstreams():
    default, upstream = LineControl(), LineControl(kp=1.1, speed=0.05, max_turn=0.35)
    assert default.update(0.4) == upstream.update(0.4)
