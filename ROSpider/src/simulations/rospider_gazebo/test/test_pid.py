import pytest
from rospider_gazebo.pid import PID, set_range, val_map


def test_proportional_term_only():
    pid = PID(0.5)
    pid.SetPoint = 10.0
    pid.last_time = 0.0
    pid.update(4.0, current_time=1.0)
    assert pid.output == pytest.approx(0.5 * 6.0)


def test_error_sign_follows_the_setpoint():
    """Every tracking demo relies on this: measurement above the set point
    gives a negative output, which is what turns the robot the right way."""
    pid = PID(0.1)
    pid.SetPoint = 320.0
    pid.last_time = 0.0
    pid.update(400.0, current_time=1.0)
    assert pid.output < 0
    pid.update(240.0, current_time=2.0)
    assert pid.output > 0


def test_integral_accumulates_over_time():
    pid = PID(0.0, 1.0, 0.0)
    pid.SetPoint = 1.0
    pid.last_time = 0.0
    pid.update(0.0, current_time=1.0)
    assert pid.output == pytest.approx(1.0)
    pid.update(0.0, current_time=2.0)
    assert pid.output == pytest.approx(2.0)


def test_integral_is_clamped_by_the_windup_guard():
    pid = PID(0.0, 1.0, 0.0)
    pid.windup_guard = 2.0
    pid.SetPoint = 1.0
    pid.last_time = 0.0
    for second in range(1, 11):
        pid.update(0.0, current_time=float(second))
    assert pid.ITerm == pytest.approx(2.0)


def test_derivative_reacts_to_a_step():
    pid = PID(0.0, 0.0, 1.0)
    pid.SetPoint = 0.0
    pid.last_time = 0.0
    pid.update(0.0, current_time=1.0)
    assert pid.output == pytest.approx(0.0)
    pid.update(1.0, current_time=2.0)          # error goes 0 -> -1 over 1 s
    assert pid.output == pytest.approx(-1.0)


def test_clear_forgets_the_integral_but_keeps_the_setpoint():
    pid = PID(0.0, 1.0, 0.0)
    pid.SetPoint = 5.0
    pid.last_time = 0.0
    pid.update(0.0, current_time=1.0)
    pid.clear()
    assert pid.ITerm == 0.0
    assert pid.output == 0.0
    assert pid.SetPoint == 5.0


def test_sample_time_holds_the_output():
    pid = PID(1.0)
    pid.sample_time = 10.0
    pid.SetPoint = 1.0
    pid.last_time = 0.0
    assert pid.update(0.0, current_time=1.0) == 0.0     # too soon
    assert pid.update(0.0, current_time=20.0) == pytest.approx(1.0)


def test_time_running_backwards_holds_the_output():
    """The clock is read at construction; a test (or a sim clock that has
    not started) can hand update() an earlier time, and upstream's guard
    leaves the output alone rather than computing a negative dt."""
    pid = PID(1.0)
    pid.SetPoint = 1.0
    pid.last_time = 100.0
    assert pid.update(0.0, current_time=1.0) == 0.0


def test_set_range_and_val_map():
    assert set_range(5, 0, 10) == 5
    assert set_range(-5, 0, 10) == 0
    assert set_range(50, 0, 10) == 10
    assert val_map(5, 0, 10, 0, 100) == pytest.approx(50)
    assert val_map(0, 0, 10, 20, 30) == pytest.approx(20)
