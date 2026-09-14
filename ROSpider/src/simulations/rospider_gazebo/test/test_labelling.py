import math

import numpy as np

from rospider_gazebo import labelling


def _camera_matrix():
    fx = 320.0 / math.tan(0.6)
    return np.array([[fx, 0.0, 320.0],
                     [0.0, fx, 240.0],
                     [0.0, 0.0, 1.0]], dtype=np.float64)


def test_box_corners_are_the_eight_vertices():
    corners = labelling.box_corners((0.05, 0.05, 0.05), (1.0, 2.0, 3.0))
    assert corners.shape == (8, 3)
    np.testing.assert_allclose(corners.min(axis=0), [0.975, 1.975, 2.975])
    np.testing.assert_allclose(corners.max(axis=0), [1.025, 2.025, 3.025])


def test_box_corners_rotate_about_z():
    # A 45-degree yaw widens the footprint of a square by sqrt(2).
    half = math.radians(45.0) / 2.0
    corners = labelling.box_corners(
        (0.10, 0.10, 0.10), (0.0, 0.0, 0.0),
        (0.0, 0.0, math.sin(half), math.cos(half)))
    assert math.isclose(corners[:, 0].max(), 0.05 * math.sqrt(2), abs_tol=1e-9)
    # Height is unaffected by a yaw.
    assert math.isclose(corners[:, 2].max(), 0.05, abs_tol=1e-9)


def test_box_corners_handle_a_tipped_object():
    # A cube dropped in a physics sim tips onto an edge. Rotating 45 degrees
    # about x raises its top corner to half the diagonal of a face, not half
    # its height -- a yaw-only model would describe a box the image does not
    # contain.
    half = math.radians(45.0) / 2.0
    corners = labelling.box_corners(
        (0.10, 0.10, 0.10), (0.0, 0.0, 0.0),
        (math.sin(half), 0.0, 0.0, math.cos(half)))
    assert math.isclose(corners[:, 2].max(), 0.05 * math.sqrt(2), abs_tol=1e-9)
    assert math.isclose(corners[:, 0].max(), 0.05, abs_tol=1e-9)


def test_project_points_drops_what_is_behind_the_camera():
    # An optical frame has +z forward. A point at z <= 0 has no projection,
    # and projecting it anyway puts a phantom box in the label file.
    points = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, -1.0], [0.0, 0.0, 0.0]])
    pixels = labelling.project_points(points, _camera_matrix())
    assert pixels.shape == (1, 2)
    np.testing.assert_allclose(pixels[0], [320.0, 240.0], atol=1e-9)


def test_bounding_box_clamps_to_the_image():
    pixels = np.array([[-50.0, -20.0], [700.0, 500.0]])
    box = labelling.bounding_box(pixels, 640, 480, min_area_px=1.0)
    assert box == (0.0, 0.0, 640.0, 480.0)


def test_bounding_box_rejects_a_sliver():
    # An object almost entirely out of frame leaves a few pixels at the edge.
    # Labelling that teaches the model that a 2-pixel strip is a cube.
    pixels = np.array([[638.0, 200.0], [645.0, 203.0]])
    assert labelling.bounding_box(pixels, 640, 480, min_area_px=300.0) is None


def test_bounding_box_rejects_an_empty_projection():
    assert labelling.bounding_box(
        np.empty((0, 2)), 640, 480, min_area_px=1.0) is None


def test_yolo_line_is_normalised_centre_and_size():
    line = labelling.yolo_line(2, (160.0, 120.0, 480.0, 360.0), 640, 480)
    assert line == '2 0.500000 0.500000 0.500000 0.500000'


def test_transform_points_applies_rotation_and_translation():
    matrix = np.eye(4)
    matrix[:3, 3] = [1.0, 2.0, 3.0]
    out = labelling.transform_points(np.array([[0.0, 0.0, 0.0]]), matrix)
    np.testing.assert_allclose(out, [[1.0, 2.0, 3.0]])
