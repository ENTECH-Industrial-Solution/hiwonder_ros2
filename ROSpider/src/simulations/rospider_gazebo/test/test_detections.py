from types import SimpleNamespace

import pytest
from rospider_gazebo import detections


def _object(box, name='red'):
    return SimpleNamespace(class_name=name, box=box)


def test_axis_aligned_box():
    assert detections.box_centroid([10, 20, 30, 60]) == (20.0, 40.0, 800)


def test_oriented_box_uses_all_four_corners():
    """An OBB model publishes eight numbers. Reading them as [x1, y1, x2, y2]
    would give the midpoint of two corners instead of the centre."""
    square = [10, 20, 30, 20, 30, 60, 10, 60]
    assert detections.box_centroid(square) == (20.0, 40.0, 800)


def test_a_box_of_another_length_is_rejected():
    assert detections.box_centroid([1, 2, 3]) is None
    assert detections.box_corners([]) is None


def test_corners_are_axis_aligned_either_way():
    assert detections.box_corners([30, 60, 10, 20]) == (10, 20, 30, 60)
    assert detections.box_corners([10, 20, 30, 20, 30, 60, 10, 60]) == (
        10, 20, 30, 60)


def test_largest_picks_the_biggest_box():
    small = _object([0, 0, 10, 10], 'red')
    big = _object([0, 0, 100, 100], 'green')
    obj, (u, v, area) = detections.largest([small, big])
    assert obj is big
    assert (u, v) == (50.0, 50.0)
    assert area == pytest.approx(10000)


def test_largest_skips_unreadable_boxes():
    obj, _ = detections.largest([_object([1, 2, 3]), _object([0, 0, 4, 4])])
    assert obj.box == [0, 0, 4, 4]


def test_largest_of_nothing_is_none():
    assert detections.largest([]) is None
    assert detections.largest([_object([1, 2, 3])]) is None
