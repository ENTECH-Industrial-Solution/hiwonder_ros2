import pytest
from rospider_gazebo.approach import Approach


def test_arrived_inside_tolerance_stops():
    vx, vy, wz, done = Approach().step(0.25, 0.02)
    assert done and (vx, vy, wz) == (0.0, 0.0, 0.0)


def test_off_to_the_side_slides_back_and_sideways_while_turning():
    """The block a metre to the left: the body must not push forward
    into what is in front of it, so it backs off and slides left as it
    turns towards the block."""
    vx, vy, wz, done = Approach().step(0.0, 1.0)
    assert not done
    assert vx < 0.0 and vy > 0.0
    assert wz == pytest.approx(0.4)                  # clamped to max_angular


def test_far_and_roughly_ahead_walks_while_steering():
    vx, vy, wz, done = Approach().step(1.0, 0.1)
    assert not done
    assert vx == pytest.approx(0.10)                 # clamped to max_linear
    assert vy == pytest.approx(0.05)
    assert 0.0 < wz < 0.4


def test_overshoot_backs_up():
    vx, _vy, _wz, done = Approach().step(0.15, 0.0)
    assert not done and vx < 0.0


def test_small_errors_still_move_at_the_minimum_speed():
    a = Approach()
    vx, vy, _wz, _done = a.step(a.target_x + 0.05, 0.0)
    assert vx == pytest.approx(a.min_linear)
    assert vy == 0.0
