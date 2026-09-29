import numpy as np
import pytest
from rospider_gazebo import depth_probe
from rospider_gazebo.depth_probe import BridgePolicy, FallPolicy


FLOOR = 0.35            # metres, the distance the policies are built around


def _frame(value_mm, shape=(480, 640), dtype=np.float64):
    return np.full(shape, value_mm, dtype=dtype)


# ------------------------------------------------------------ depth units

def test_gazebo_metres_become_millimetres():
    """The simulator publishes 32FC1 metres where the robot publishes 16UC1
    millimetres; every threshold in this module assumes millimetres."""
    frame = np.full((4, 4), 0.35, dtype=np.float32)
    assert depth_probe.to_millimetres(frame)[0, 0] == pytest.approx(350.0)


def test_robot_millimetres_are_left_alone():
    frame = np.full((4, 4), 350, dtype=np.uint16)
    assert depth_probe.to_millimetres(frame)[0, 0] == pytest.approx(350.0)


def test_no_return_becomes_zero():
    frame = np.array([[np.inf, np.nan, 0.35]], dtype=np.float32)
    assert depth_probe.to_millimetres(frame).tolist() == [[0.0, 0.0,
                                                           pytest.approx(350.0)]]


# ------------------------------------------------------------ ROI reading

def test_roi_distance_is_metres():
    assert depth_probe.roi_distance(_frame(350.0),
                                    depth_probe.FALL_ROIS['center']) == 0.35


def test_roi_distance_ignores_holes_and_the_far_field():
    frame = _frame(350.0)
    roi = depth_probe.FALL_ROIS['center']
    frame[roi[0]:roi[1], roi[2]:roi[2] + 5] = 0.0        # no reading
    frame[roi[0]:roi[1], roi[2] + 5:roi[2] + 7] = 40000  # out of range
    assert depth_probe.roi_distance(frame, roi) == 0.35


def test_an_empty_roi_reads_zero():
    assert depth_probe.roi_distance(_frame(0.0),
                                    depth_probe.FALL_ROIS['left']) == 0.0


def test_roi_center_is_inside_the_roi():
    for roi in depth_probe.FALL_ROIS.values():
        x, y = depth_probe.roi_center(roi)
        assert roi[2] <= x <= roi[3] and roi[0] <= y <= roi[1]


# --------------------------------------------------------- falling policy

def test_flat_floor_walks_forward():
    policy = FallPolicy(FLOOR)
    assert policy.update([FLOOR, FLOOR, FLOOR], 1.0) == (0.05, 0.0)
    assert policy.update([FLOOR, FLOOR, FLOOR], 2.0) == (0.05, 0.0)
    assert not policy.turning


def test_an_edge_under_any_probe_turns():
    policy = FallPolicy(FLOOR)
    for probes in ([1.2, FLOOR, FLOOR], [FLOOR, 1.2, FLOOR],
                   [FLOOR, FLOOR, 1.2]):
        assert policy.update(probes, 1.0) == (0.0, 0.2)
        assert policy.turning


def test_the_turn_is_held_for_its_own_time_after_the_edge_clears():
    policy = FallPolicy(FLOOR)
    policy.update([1.2, FLOOR, FLOOR], 1.0)            # edge seen at t=1.0
    assert policy.update([FLOOR] * 3, 1.1) is None     # still turning
    assert policy.update([FLOOR] * 3, 1.4) == (0.0, 0.0)   # 0.3 s later: stop
    assert not policy.turning
    assert policy.update([FLOOR] * 3, 1.5) is None     # settling
    assert policy.update([FLOOR] * 3, 1.7) == (0.05, 0.0)  # and off again


def test_a_missing_reading_counts_as_an_edge():
    """roi_distance returns 0.0 for an ROI it cannot read, which is a long
    way from the floor distance -- so the robot turns rather than walking on
    into whatever it cannot see."""
    assert FallPolicy(FLOOR).update([0.0, FLOOR, FLOOR], 1.0) == (0.0, 0.2)


# ---------------------------------------------------------- bridge policy

def test_off_the_bridge_and_centred_stands_still():
    policy = BridgePolicy(FLOOR)
    assert policy.update(FLOOR, FLOOR, FLOOR, FLOOR, FLOOR) == (0.0, 0.0)
    assert not policy.on_bridge
    assert policy.wanted_pose == BridgePolicy.DEFAULT


def test_air_under_both_outer_probes_is_the_bridge():
    policy = BridgePolicy(FLOOR)
    linear, angular = policy.update(1.2, FLOOR, FLOOR, FLOOR, 1.2)
    assert policy.on_bridge
    assert policy.wanted_pose == BridgePolicy.NARROW
    assert (linear, angular) == (0.02, 0.0)


def test_drifting_left_steers_back():
    policy = BridgePolicy(FLOOR)
    # Left inner probe off the floor, centre still on it: a gentle correction.
    assert policy.update(FLOOR, 1.2, FLOOR, FLOOR, FLOOR)[1] == -0.1
    # Centre gone too: a harder one.
    assert policy.update(FLOOR, 1.2, 1.2, FLOOR, FLOOR)[1] == -0.2


def test_drifting_right_steers_the_other_way():
    policy = BridgePolicy(FLOOR)
    assert policy.update(FLOOR, FLOOR, FLOOR, 1.2, FLOOR)[1] == 0.1
    assert policy.update(FLOOR, FLOOR, 1.2, 1.2, FLOOR)[1] == 0.2


def test_nothing_under_any_inner_probe_stops():
    policy = BridgePolicy(FLOOR)
    assert policy.update(FLOOR, 1.2, 1.2, 1.2, FLOOR) == (0.0, 0.0)


# ------------------------------------------------------------ pixel -> point

K = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]


def test_patch_depth_is_the_median_ignoring_holes():
    frame = np.full((10, 10), 2.0)
    frame[4:7, 4:7] = [[1.0, 1.0, np.inf], [1.0, 0.0, np.nan], [1.0, 1.0, 1.0]]
    assert depth_probe.patch_depth(frame, 5, 5, window=3) == pytest.approx(1.0)


def test_patch_depth_with_no_reading_is_none():
    frame = np.zeros((10, 10))
    assert depth_probe.patch_depth(frame, 5, 5) is None


def test_camera_point_at_the_principal_point_is_straight_ahead():
    assert depth_probe.camera_point(320, 240, 2.0, K).tolist() == [0.0, 0.0, 2.0]


def test_camera_point_scales_with_depth_over_focal_length():
    x, y, z = depth_probe.camera_point(420, 140, 1.0, K)
    assert (x, y, z) == pytest.approx((0.2, -0.2, 1.0))


def test_transform_point_rotates_then_translates():
    # 90 deg about z: (1, 0, 0) -> (0, 1, 0), then shifted by (0, 0, 0.3)
    s = np.sqrt(0.5)
    point = depth_probe.transform_point([1.0, 0.0, 0.0], (0.0, 0.0, 0.3), (0.0, 0.0, s, s))
    assert point.tolist() == pytest.approx([0.0, 1.0, 0.3])
