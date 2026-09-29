import cv2
import numpy as np
import pytest
from rospider_gazebo import color_picker
from rospider_gazebo.color_picker import (
    ColorPicker, FollowControl, find_blob, lab_band)

GREY = (128, 128, 128)
RED = (0, 0, 255)                    # BGR


def _frame(colour=GREY, size=(480, 640)):
    frame = np.empty((*size, 3), dtype=np.uint8)
    frame[:] = colour
    return frame


def _lab(bgr):
    pixel = np.array([[bgr]], dtype=np.uint8)
    return tuple(int(v) for v in cv2.cvtColor(pixel, cv2.COLOR_BGR2LAB)[0, 0])


# ------------------------------------------------------------ ColorPicker

def test_picker_needs_repeat_frames_before_it_answers():
    picker = ColorPicker(320, 240, repeat=3)
    frame = _frame(RED)
    assert picker.sample(frame) is None
    assert picker.sample(frame) is None
    lab, bgr = picker.sample(frame)
    assert bgr == RED
    assert lab == _lab(RED)


def test_picker_count_grows_to_repeat_and_stops():
    """The node draws a circle whose radius is the count, as upstream."""
    picker = ColorPicker(10, 10, repeat=2)
    frame = _frame(RED)
    assert picker.count == 0
    picker.sample(frame)
    assert picker.count == 1
    picker.sample(frame)
    picker.sample(frame)
    assert picker.count == 2


def test_picker_averages_the_pixels_under_the_click():
    picker = ColorPicker(100, 100, repeat=1)
    frame = _frame(GREY)
    frame[99:101, 99:101] = RED           # the 2x2 patch upstream reads
    _lab_value, bgr = picker.sample(frame)
    assert bgr == RED


def test_picker_survives_a_click_on_the_frame_edge():
    picker = ColorPicker(0, 0, repeat=1)
    assert picker.sample(_frame(RED))[1] == RED
    picker = ColorPicker(639, 479, repeat=1)
    assert picker.sample(_frame(RED))[1] == RED


# ------------------------------------------------------------ lab_band

def test_lab_band_is_upstreams_threshold_scaling():
    lower, upper = lab_band((100, 128, 128), 0.5)
    assert lower == (50, 103, 103)
    assert upper == (150, 153, 153)


def test_lab_band_is_clipped_to_eight_bits():
    lower, upper = lab_band((250, 5, 128), 0.5)
    assert lower == (200, 0, 103)
    assert upper == (255, 30, 153)


# ------------------------------------------------------------ find_blob

def test_find_blob_returns_the_square_in_frame_pixels():
    frame = _frame(GREY)
    frame[200:280, 400:480] = RED         # 80 px square centred (440, 240)
    lower, upper = lab_band(_lab(RED), 0.3)
    x, y, r = find_blob(frame, lower, upper)
    assert x == pytest.approx(440, abs=4)
    assert y == pytest.approx(240, abs=4)
    # Enclosing circle of the square after the 320x200 downscale and the
    # 3x3 open, scaled back by the x factor as upstream does.
    assert 35 < r < 60


def test_find_blob_ignores_specks_and_empty_frames():
    frame = _frame(GREY)
    lower, upper = lab_band(_lab(RED), 0.3)
    assert find_blob(frame, lower, upper) is None
    frame[240:242, 320:322] = RED         # 2x2 px: under MIN_AREA at 320x200
    assert find_blob(frame, lower, upper) is None


def test_find_blob_takes_the_largest():
    frame = _frame(GREY)
    frame[100:130, 100:130] = RED
    frame[300:400, 400:500] = RED
    lower, upper = lab_band(_lab(RED), 0.3)
    x, y, _r = find_blob(frame, lower, upper)
    assert x == pytest.approx(450, abs=4)
    assert y == pytest.approx(350, abs=4)


def test_find_blob_sticks_to_the_last_patch_when_a_bigger_one_appears():
    frame = _frame(GREY)
    frame[100:130, 100:130] = RED           # the tracked object
    frame[300:400, 400:500] = RED           # a poster on the wall
    lower, upper = lab_band(_lab(RED), 0.3)
    x, y, _r = find_blob(frame, lower, upper, last=(118, 112, 20))
    assert x == pytest.approx(115, abs=4)
    assert y == pytest.approx(115, abs=4)


def test_find_blob_falls_back_to_the_largest_when_the_last_is_far():
    frame = _frame(GREY)
    frame[100:130, 100:130] = RED
    frame[300:400, 400:500] = RED
    lower, upper = lab_band(_lab(RED), 0.3)
    x, _y, _r = find_blob(frame, lower, upper, last=(600, 40, 20))
    assert x == pytest.approx(450, abs=4)


# ------------------------------------------------------------ FollowControl

def _control():
    control = FollowControl()
    # The PIDs stamp construction time; the tests pass their own clock.
    control.pid_yaw.last_time = control.pid_dist.last_time = 0.0
    return control


def test_target_to_the_right_turns_right():
    control = _control()
    _linear, angular = control.update(500, color_picker.STOP_POINT[1], now=1.0)
    assert angular < 0
    _linear, angular = control.update(100, color_picker.STOP_POINT[1], now=2.0)
    assert angular > 0


def test_target_far_up_the_frame_drives_forward_close_backs_up():
    control = _control()
    linear, _angular = control.update(320, 100, now=1.0)
    assert linear == pytest.approx(0.05)
    linear, _angular = control.update(320, 470, now=2.0)
    assert linear == pytest.approx(-0.05)


def test_near_the_stop_point_the_speed_drops_then_stops():
    control = _control()
    x_stop, y_stop = color_picker.STOP_POINT
    linear, angular = control.update(x_stop, y_stop - 45, now=1.0)
    assert 0 < linear <= 0.01
    linear, angular = control.update(x_stop, y_stop - 10, now=2.0)
    assert linear == 0.0
    assert angular == 0.0


def test_clear_zeroes_the_outputs():
    control = _control()
    control.update(600, 50, now=1.0)
    control.clear()
    assert (control.pid_yaw.output, control.pid_dist.output) == (0.0, 0.0)


def test_speed_and_turn_limits_are_settable():
    control = FollowControl(max_speed=0.12, max_turn=0.3)
    control.pid_yaw.last_time = control.pid_dist.last_time = 0.0
    linear, angular = control.update(600, 50, now=1.0)
    assert linear == pytest.approx(0.12)
    assert angular == pytest.approx(-0.3)


def test_a_slow_limit_also_caps_the_final_approach():
    control = FollowControl(max_speed=0.005)
    control.pid_yaw.last_time = control.pid_dist.last_time = 0.0
    x_stop, y_stop = color_picker.STOP_POINT
    linear, _angular = control.update(x_stop, y_stop - 45, now=1.0)
    assert linear == pytest.approx(0.005)
