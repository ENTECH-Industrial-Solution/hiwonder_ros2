import math

import cv2
import numpy as np
import pytest
from rospider_gazebo import shape_detect

PLANE = 350.0                       # floor distance, millimetres
ROI = [50, 350, 150, 500]
#: A plausible 640x480 pinhole, flat CameraInfo.k.
INTRINSICS = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]


def _floor():
    return np.full((480, 640), PLANE, dtype=np.float64)


def _box(depth, centre=(320, 200), size=60, top=300.0):
    """A square standing `PLANE - top` mm off the floor, flat on top."""
    x, y = centre
    half = size // 2
    depth[y - half:y + half, x - half:x + half] = top
    return depth


def _dome(depth, centre=(320, 200), radius=30, top=300.0):
    """A round cap whose depth varies across it, as a real sphere's does."""
    x, y = centre
    ys, xs = np.mgrid[0:depth.shape[0], 0:depth.shape[1]]
    inside = (xs - x) ** 2 + (ys - y) ** 2 <= radius ** 2
    bulge = np.sqrt(np.clip(radius ** 2 - (xs - x) ** 2 - (ys - y) ** 2, 0,
                            None))
    depth[inside] = (top + (radius - bulge))[inside]
    return depth


def _flat_disc(depth, centre=(320, 200), radius=30, top=300.0):
    """A round cap at one single depth -- a cylinder seen end-on."""
    x, y = centre
    ys, xs = np.mgrid[0:depth.shape[0], 0:depth.shape[1]]
    depth[(xs - x) ** 2 + (ys - y) ** 2 <= radius ** 2] = top
    return depth


# --------------------------------------------------------------- distances

def test_stable_distance_is_the_median_inside_the_roi():
    depth = _box(_floor())
    # Mostly floor, one box: the median is still the floor.
    assert shape_detect.stable_distance(depth, ROI, PLANE) == PLANE


def test_nearest_distance_finds_the_closest_surface():
    depth = _box(_floor(), top=300.0)
    assert shape_detect.nearest_distance(depth, ROI) == 300.0


def test_nearest_distance_falls_back_on_an_empty_roi():
    depth = np.zeros((480, 640), dtype=np.float64)
    assert shape_detect.nearest_distance(depth, ROI, fallback=PLANE) == PLANE


def test_pixel_to_camera_puts_the_principal_point_on_the_axis():
    assert shape_detect.pixel_to_camera(320, 240, 0.5, INTRINSICS) == (
        0.0, 0.0, 0.5)
    x, y, z = shape_detect.pixel_to_camera(420, 240, 0.5, INTRINSICS)
    assert (x, y, z) == (pytest.approx(0.1), 0.0, 0.5)


# ---------------------------------------------------------------- contours

def test_nothing_on_an_empty_floor():
    depth = _floor()
    near = shape_detect.nearest_distance(depth, ROI, PLANE)
    assert shape_detect.recognise(depth, np.zeros((480, 640, 3), np.uint8),
                                  INTRINSICS, PLANE, near, ROI) == []


def test_an_object_outside_the_roi_is_ignored():
    depth = _box(_floor(), centre=(600, 440))
    found = shape_detect.recognise(depth, np.zeros((480, 640, 3), np.uint8),
                                   INTRINSICS, PLANE, 300.0, ROI)
    assert found == []


def test_specks_are_ignored():
    """Under MIN_CONTOUR_AREA a blob is depth noise, not an object."""
    depth = _box(_floor(), size=8)
    found = shape_detect.recognise(depth, np.zeros((480, 640, 3), np.uint8),
                                   INTRINSICS, PLANE, 300.0, ROI)
    assert found == []


# ------------------------------------------------------------ recognition

def test_a_flat_square_is_a_cuboid():
    depth = _box(_floor())
    found = shape_detect.recognise(depth, np.zeros((480, 640, 3), np.uint8),
                                   INTRINSICS, PLANE, 300.0, ROI)
    assert [o.kind for o in found] == ['cuboid']
    assert found[0].name == 'cuboid_1'
    assert found[0].depth == pytest.approx(300.0)
    assert found[0].height == pytest.approx(50.0)     # 350 - 300


def test_a_flat_disc_is_a_cylinder():
    depth = _flat_disc(_floor())
    found = shape_detect.recognise(depth, np.zeros((480, 640, 3), np.uint8),
                                   INTRINSICS, PLANE, 300.0, ROI)
    assert [o.kind for o in found] == ['cylinder']


def test_a_curved_dome_is_a_sphere():
    depth = _dome(_floor())
    found = shape_detect.recognise(depth, np.zeros((480, 640, 3), np.uint8),
                                   INTRINSICS, PLANE, 300.0, ROI)
    assert [o.kind for o in found] == ['sphere']


def test_two_objects_are_numbered_separately():
    depth = _box(_floor(), centre=(250, 200))
    depth = _box(depth, centre=(400, 200))
    found = shape_detect.recognise(depth, np.zeros((480, 640, 3), np.uint8),
                                   INTRINSICS, PLANE, 300.0, ROI)
    assert sorted(o.name for o in found) == ['cuboid_1', 'cuboid_2']


def test_the_sampled_colour_comes_from_the_object_centre():
    depth = _box(_floor())
    frame = np.zeros((480, 640, 3), np.uint8)
    frame[190:210, 310:330] = (255, 0, 0)          # blue, BGR
    found = shape_detect.recognise(depth, frame, INTRINSICS, PLANE, 300.0,
                                   ROI)
    assert shape_detect.colour_name(found[0].bgr) == 'blue'


def test_nothing_nearer_than_the_floor_recognises_nothing():
    depth = _box(_floor())
    assert shape_detect.recognise(depth, np.zeros((480, 640, 3), np.uint8),
                                  INTRINSICS, PLANE, PLANE + 10, ROI) == []


# --------------------------------------------------------------- volumes

def test_cuboid_volume_is_width_by_height_by_thickness():
    # 20 x 30 x 10 mm = 6000 mm^3 = 6 cm^3
    assert shape_detect.volume_cm3('cuboid', 20, 30, 10) == pytest.approx(6.0)


def test_sphere_volume_uses_the_mean_radius():
    assert shape_detect.volume_cm3('sphere', 20, 20, 999) == pytest.approx(
        4 / 3 * math.pi * 10 ** 3 / 1000)


def test_cylinder_volume_uses_the_width_as_its_diameter():
    assert shape_detect.volume_cm3('cylinder', 20, 99, 10) == pytest.approx(
        math.pi * 100 * 10 / 1000)


def test_an_unknown_shape_has_no_volume():
    assert shape_detect.volume_cm3('pyramid', 20, 30, 10) == 0.0


def test_colour_name_picks_the_leading_channel():
    assert shape_detect.colour_name((0, 0, 255)) == 'red'
    assert shape_detect.colour_name((0, 255, 0)) == 'green'
    assert shape_detect.colour_name((255, 0, 0)) == 'blue'


# ---------------------------------------------------- drawn trajectories

def _polygon(sides, radius=120, centre=(200, 200)):
    """Points around a regular polygon, as a fingertip would trace it."""
    points = []
    for step in range(sides * 20 + 1):
        angle = 2 * math.pi * step / (sides * 20)
        corner = angle / (2 * math.pi) * sides
        # Walk edge by edge so the path has straight sides, not an arc.
        first = int(corner) % sides
        second = (first + 1) % sides
        t = corner - int(corner)
        def vertex(i):
            a = 2 * math.pi * i / sides
            return (centre[0] + radius * math.cos(a),
                    centre[1] + radius * math.sin(a))
        x0, y0 = vertex(first)
        x1, y1 = vertex(second)
        points.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
    return points


def test_a_traced_triangle_is_named():
    assert shape_detect.trajectory_shape(_polygon(3))[0] == 'Triangle'


def test_a_traced_square_is_named():
    assert shape_detect.trajectory_shape(_polygon(4))[0] == 'Square'


def test_a_traced_circle_is_named():
    assert shape_detect.trajectory_shape(_polygon(40))[0] == 'Circle'


def test_too_few_points_are_not_a_shape():
    name, view = shape_detect.trajectory_shape([(0, 0), (1, 1)])
    assert name is None
    assert view.ndim == 3


def test_a_scribble_too_small_to_enclose_anything_is_not_a_shape():
    assert shape_detect.trajectory_shape(
        [(0, 0), (3, 0), (3, 3), (0, 3), (0, 0)])[0] is None


def test_area_max_contour_filters_by_threshold():
    canvas = np.zeros((100, 100), np.uint8)
    cv2.rectangle(canvas, (10, 10), (40, 40), 255, -1)
    found = cv2.findContours(canvas, cv2.RETR_EXTERNAL,
                             cv2.CHAIN_APPROX_NONE)[-2]
    contour, area = shape_detect.area_max_contour(found, 100)
    assert contour is not None and area > 100
    assert shape_detect.area_max_contour(found, 10_000)[0] is None
