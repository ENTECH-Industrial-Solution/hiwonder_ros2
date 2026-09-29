import math

import pytest
from rospider_gazebo.approach import Approach


def test_arrived_centred_stops():
    vx, vy, wz, done = Approach().step(0.25, 0.01)
    assert done and (vx, vy, wz) == (0.0, 0.0, 0.0)


def test_at_the_distance_but_off_centre_is_not_there_yet():
    """3 cm to the left at the pedestal distance: the arm would have to
    reach round at 7 degrees, so the body slides and turns to square up."""
    vx, vy, wz, done = Approach().step(0.235, 0.03)
    assert not done
    assert vx == 0.0
    assert vy > 0.0 and wz > 0.0


def test_off_to_the_side_turns_in_place():
    """The block a metre to the left: face it before walking, rather than
    crabbing sideways at it."""
    vx, vy, wz, done = Approach().step(0.0, 1.0)
    assert not done
    assert (vx, vy) == (0.0, 0.0)
    assert wz == pytest.approx(0.4)                  # clamped to max_angular


def test_behind_turns_rather_than_backing_into_it():
    vx, vy, wz, done = Approach().step(-0.5, -0.1)
    assert not done
    assert (vx, vy) == (0.0, 0.0)
    assert wz == pytest.approx(-0.4)


def test_facing_it_walks_straight_in():
    vx, vy, wz, done = Approach().step(1.0, 0.0)
    assert not done
    assert vx == pytest.approx(0.10)                 # clamped to max_linear
    assert (vy, wz) == (0.0, 0.0)


def test_nearly_facing_it_walks_while_steering():
    vx, vy, wz, done = Approach().step(1.0, 0.05)
    assert not done
    assert 0.05 < vx < 0.10                          # a little slower off-axis
    assert 0.0 < wz < 0.4
    assert 0.0 < vy < vx                             # a trim, not a crab


def test_the_further_off_axis_the_slower_it_walks():
    a = Approach()
    speeds = [a.step(1.0, math.tan(math.radians(deg)))[0] for deg in (0, 5, 10, 20, 30)]
    assert speeds == sorted(speeds, reverse=True)
    assert speeds[-2] == speeds[-1] == 0.0           # past turn_first: turns in place


def test_overshoot_backs_up():
    vx, _vy, _wz, done = Approach().step(0.15, 0.0)
    assert not done and vx < 0.0


def test_small_errors_still_move_at_the_minimum_speed():
    a = Approach()
    vx, vy, _wz, _done = a.step(a.target_x + 0.05, 0.0)
    assert vx == pytest.approx(a.min_linear)
    assert vy == 0.0
