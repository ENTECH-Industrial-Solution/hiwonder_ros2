"""Shape recognition and volume estimation from a depth image.

Used by scripts/object_classification.py, ported from example/rgbd_example/include/{object_volume_measurement,
object_classification}.py. Every threshold below is upstream's.

Distances are millimetres throughout -- use depth_probe.to_millimetres() on
the incoming frame, which also covers the simulator publishing 32FC1 metres
where the robot publishes 16UC1 millimetres.

cv2 and numpy only, no ROS, so test/test_shape_detect.py can run it on
synthetic depth frames.
"""

import math
from typing import NamedTuple

import cv2
import numpy as np

MIN_CONTOUR_AREA = 300      # px^2; smaller contours are noise
APPROX_EPSILON = 0.035      # fraction of the perimeter, for approxPolyDP
SPHERE_DEPTH_STD = 2.0      # mm; a curved top varies more than a flat one
BACKGROUND_MARGIN = 40      # mm behind the nearest surface still counts
FLOOR_MARGIN = 10           # mm in front of the floor plane already counts


class DepthObject(NamedTuple):
    """One recognised object."""

    name: str                   # 'sphere_1', 'cuboid_2', ...
    kind: str                   # 'sphere', 'cuboid' or 'cylinder'
    box: tuple                  # bounding rect (x, y, w, h) in pixels
    rect: tuple                 # minAreaRect ((cx, cy), (w, h), angle)
    angle: float                # rect angle after upstream's corrections
    depth: float                # distance to the object, mm
    height: float               # how far it stands off the floor, mm
    volume: float               # cm^3
    position: tuple             # (x, y, z) in the camera frame, metres
    bgr: tuple                  # the pixel colour at the object's centre


def pixel_to_camera(u, v, depth_m, intrinsics):
    """A pixel plus its depth in metres as a point in the camera frame.

    `intrinsics` is the flat 9-element CameraInfo.k.
    """
    fx, fy = intrinsics[0], intrinsics[4]
    cx, cy = intrinsics[2], intrinsics[5]
    return ((u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m)


def stable_distance(depth_mm, roi, fallback):
    """Median distance inside the ROI, ignoring readings past a metre.

    This is the demo's idea of "how far away is whatever is on the table";
    `fallback` (the floor plane) stands in when the ROI is empty.
    """
    patch = depth_mm[roi[0]:roi[1], roi[2]:roi[3]]
    valid = patch[(patch > 0) & (patch < 1000)]
    if valid.size == 0:
        return float(fallback)
    return float(np.median(valid))


def nearest_distance(depth_mm, roi, fallback=1000.0):
    """Distance to the nearest surface inside the ROI, millimetres.

    object_classification's view of "how far away is the thing on the table",
    where object_volume takes the median instead. Upstream masks everything
    outside the ROI to 1000 mm and every hole to 55555 before taking the
    minimum; the same thing, written as a mask.
    """
    patch = depth_mm[roi[0]:roi[1], roi[2]:roi[3]]
    valid = patch[patch > 0]
    return float(valid.min()) if valid.size else float(fallback)


def contours(depth_mm, plane_distance, near_distance, roi=None):
    """Outlines of everything standing between the camera and the floor.

    Pixels behind `plane_distance - FLOOR_MARGIN` (the floor and beyond) and
    further than `near_distance + BACKGROUND_MARGIN` (behind the nearest
    surface) are dropped, and what remains is thresholded.
    """
    ceiling = plane_distance - FLOOR_MARGIN
    depth = np.where(depth_mm > ceiling, 0, depth_mm)
    depth = np.where(depth > near_distance + BACKGROUND_MARGIN, 0, depth)
    if roi is not None:
        mask = np.zeros(depth.shape[:2], dtype=np.uint8)
        mask[roi[0]:roi[1], roi[2]:roi[3]] = 255
        depth = np.where(mask == 255, depth, 0)
    scaled = np.clip(depth, 0, ceiling).astype(np.float64) / ceiling * 255
    _, binary = cv2.threshold(scaled.astype(np.uint8), 1, 255,
                              cv2.THRESH_BINARY)
    return cv2.findContours(binary, cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_NONE)[-2]


def volume_cm3(kind, width_mm, height_mm, thickness_mm):
    """Volume of a recognised solid, in cubic centimetres.

    The camera only sees the top of an object, so upstream estimates the
    third dimension from how far the top stands off the floor.
    """
    if kind == 'sphere':
        radius = (width_mm + height_mm) / 4.0
        volume = (4.0 / 3.0) * math.pi * radius ** 3
    elif kind == 'cuboid':
        volume = width_mm * height_mm * thickness_mm
    elif kind == 'cylinder':
        radius = width_mm / 2.0
        volume = math.pi * radius ** 2 * thickness_mm
    else:
        volume = 0.0
    return volume / 1000.0


def recognise(depth_mm, bgr_image, intrinsics, plane_distance,
              near_distance, roi=None):
    """Every object the depth frame shows standing on the floor plane.

    Returns a list of DepthObject, ordered as cv2.findContours found them.
    `roi` limits the search to [y_min, y_max, x_min, x_max].
    """
    found = []
    if near_distance > plane_distance:
        # Nothing nearer than the floor: an empty table, or a bad reading.
        return found

    counts = {'sphere': 0, 'cuboid': 0, 'cylinder': 0}
    height, width = depth_mm.shape[:2]
    fx, fy = intrinsics[0], intrinsics[4]

    for contour in contours(depth_mm, plane_distance, near_distance, roi):
        if cv2.contourArea(contour) < MIN_CONTOUR_AREA:
            continue
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, APPROX_EPSILON * perimeter, True)
        corners = len(approx)
        if corners < 4:
            continue

        (cx, cy), _ = cv2.minEnclosingCircle(contour)
        centre, (rect_w, rect_h), angle = cv2.minAreaRect(contour)
        if angle < -45:
            angle += 90
        if rect_h > 0 and rect_w > rect_h and rect_w / rect_h > 1.5:
            angle += 90

        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, cv2.FILLED)
        depths = depth_mm[mask == 255]
        depths = depths[depths > 0]
        if depths.size == 0:
            continue
        depth = float(np.median(depths))
        spread = float(np.std(depths))

        if corners > 4:
            kind = 'sphere' if spread > SPHERE_DEPTH_STD else 'cylinder'
            angle = 0.0
        else:
            kind = 'cuboid'
        counts[kind] += 1

        real_w = rect_w * depth / fx
        real_h = rect_h * depth / fy
        thickness = max(0.0, plane_distance - depth)
        row = min(max(int(centre[1]), 0), height - 1)
        column = min(max(int(centre[0]), 0), width - 1)
        found.append(DepthObject(
            name=f'{kind}_{counts[kind]}',
            kind=kind,
            box=tuple(int(v) for v in cv2.boundingRect(approx)),
            rect=(centre, (rect_w, rect_h), angle),
            angle=float(angle),
            depth=depth,
            height=thickness,
            volume=volume_cm3(kind, real_w, real_h, thickness),
            position=pixel_to_camera(cx, cy, depth / 1000.0, intrinsics),
            bgr=tuple(int(v) for v in bgr_image[row, column][:3]),
        ))
    return found


def colour_name(bgr):
    """'red', 'green' or 'blue' for a sampled pixel, whichever channel leads.

    Upstream's color_comparison only knows red and blue, and its blue test
    compares the green channel with itself; green is added here and the typo
    fixed, because the simulation's cubes come in all three.
    """
    blue, green, red = (int(v) for v in bgr[:3])
    if red >= green and red >= blue:
        return 'red'
    if green >= red and green >= blue:
        return 'green'
    return 'blue'


# ------------------------------------------------- finger-drawn trajectories

#: How many polygon corners the drawn shape is called by, as upstream's
#: finger_trajectory.py reads them off approxPolyDP.
TRAJECTORY_SHAPES = ((3, 3, 'Triangle'), (4, 5, 'Square'),
                     (6, 9, 'Circle'), (10, 10, 'Star'))

TRAJECTORY_MARGIN = 50      # px of white space around the drawing
TRAJECTORY_MIN_AREA = 300   # px^2; a smaller scribble is not a shape


def area_max_contour(contours_, threshold=50):
    """The largest contour and its area, or (None, area) below `threshold`.

    sdk.common.get_area_max_contour.
    """
    best, best_area = None, 0.0
    for contour in contours_:
        area = abs(cv2.contourArea(contour))
        if area > best_area:
            best_area = area
            if area > threshold:
                best = contour
    return best, best_area


def track_image(points, margin=TRAJECTORY_MARGIN):
    """A black canvas with the drawn path on it in white, cropped to fit."""
    points = np.array(points, dtype=np.int32)
    low = points.min(axis=0)
    high = points.max(axis=0)
    size = high - low + 2 * margin
    canvas = np.zeros((int(size[1]), int(size[0])), dtype=np.uint8)
    shifted = points - low + margin
    for start, end in zip(shifted, shifted[1:]):
        cv2.line(canvas, tuple(start), tuple(end), 255, 1)
    return canvas


def trajectory_shape(points):
    """(shape name or None, the outline drawn for the window).

    Upstream's recipe: fill the drawn path, open it three times to close the
    gaps a moving fingertip leaves, then name the result by how many corners
    approxPolyDP finds.
    """
    if len(points) < 3:
        return None, np.zeros((1, 1, 3), dtype=np.uint8)

    canvas = track_image(points)
    found = cv2.findContours(canvas, cv2.RETR_EXTERNAL,
                             cv2.CHAIN_APPROX_NONE)[-2]
    contour, _ = area_max_contour(found, TRAJECTORY_MIN_AREA)
    if contour is None:
        return None, cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)

    cv2.fillPoly(canvas, [contour], 255)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    for _ in range(3):
        canvas = cv2.dilate(cv2.erode(canvas, kernel), kernel)

    found = cv2.findContours(canvas, cv2.RETR_EXTERNAL,
                             cv2.CHAIN_APPROX_NONE)[-2]
    contour, _ = area_max_contour(found, TRAJECTORY_MIN_AREA)
    if contour is None:
        return None, cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)

    approx = cv2.approxPolyDP(contour, 0.026 * cv2.arcLength(contour, True),
                              True)
    view = np.zeros(canvas.shape + (3,), dtype=np.uint8)
    cv2.drawContours(view, [contour], -1, (0, 255, 0), 2)
    cv2.drawContours(view, [approx], -1, (0, 0, 255), 2)

    name = next((n for low, high, n in TRAJECTORY_SHAPES
                 if low <= len(approx) <= high), None)
    if name is not None:
        cv2.putText(view, name, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    (255, 255, 0), 2)
    return name, view
