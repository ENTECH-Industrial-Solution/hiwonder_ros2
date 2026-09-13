"""AprilTag geometry, rendering, detection and pose solving, without ROS.

scripts/apriltag_detect.py is a thin ROS wrapper around this module. Keeping
the maths here is what lets test/test_apriltag.py run under plain pytest with
no simulator -- the same split rospider_gazebo/arm_ik.py and
scripts/pick_and_place.py already use.

Tag frame convention: x right, y up, z out of the tag face toward the viewer.
That is OpenCV's own marker convention and what SOLVEPNP_IPPE_SQUARE requires
of the object points below; changing it silently breaks pose solving.
"""

import cv2
import numpy as np

FAMILY = 'tag36h11'
_DICT_ID = cv2.aruco.DICT_APRILTAG_36h11

# A 36h11 marker is 10 modules across including its own black border.
_TAG_MODULES = 10

# Edge of the black square in metres, excluding the quiet zone. Must match the
# physical board through BOARD_FACE below, or every distance solve_tag_pose
# reports is wrong by the ratio between them.
TAG_SIZE = 0.15

# White margin around the marker, in modules. The detector needs light around
# the tag and finds nothing at all without it -- the most common way a rendered
# tag fails, and it fails silently.
QUIET_MODULES = 2

# The board carries the whole texture, marker plus quiet zone, so its face is
# larger than the tag by exactly that ratio. Derived rather than chosen: a
# hand-picked board size would make the black square some other size and put a
# constant scale error into every pose.
BOARD_FACE = TAG_SIZE * (_TAG_MODULES + 2 * QUIET_MODULES) / _TAG_MODULES

# Station model, all in the model frame, which faces +x: a robot approaching
# from +x sees the tag.
#   - pedestal, identical to models/pick_pedestal so its top face is at
#     z = 0.08 and pick_place.yaml's drop_slots z of 0.135 lands a cube on it
#   - a post holding the board up
#   - the board, BOARD_FACE square, behind the pedestal
PEDESTAL_SIZE = (0.14, 0.22, 0.08)
BOARD_THICKNESS = 0.01
BOARD_OFFSET_X = -0.12
TAG_CENTRE_HEIGHT = 0.25
POST_SIZE = (0.03, 0.03, TAG_CENTRE_HEIGHT - BOARD_FACE / 2.0)


def object_points(size=TAG_SIZE):
    """The tag's four corners in the tag frame, metres, float32.

    Order and orientation are fixed by SOLVEPNP_IPPE_SQUARE: top-left,
    top-right, bottom-right, bottom-left, with y up. This is also the order
    cv2.aruco.detectMarkers returns image corners in, so the two line up
    element by element.
    """
    half = size / 2.0
    return np.array([[-half, half, 0.0],
                     [half, half, 0.0],
                     [half, -half, 0.0],
                     [-half, -half, 0.0]], dtype=np.float32)


def generate_tag_image(tag_id, module_px=40):
    """A square tag36h11 image with its white quiet zone, grayscale uint8."""
    tag_px = _TAG_MODULES * module_px
    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(_DICT_ID), tag_id, tag_px)
    pad = QUIET_MODULES * module_px
    image = np.full((tag_px + 2 * pad, tag_px + 2 * pad), 255, np.uint8)
    image[pad:pad + tag_px, pad:pad + tag_px] = marker
    return image


def detect_tags(gray):
    """[(tag_id, corners)] for every tag36h11 in a grayscale image.

    corners is (4, 2) float32 in the same order as object_points().
    """
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(_DICT_ID),
        cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return []
    return [(int(tag_id), corner.reshape(4, 2).astype(np.float32))
            for tag_id, corner in zip(ids.ravel(), corners)]


def solve_tag_pose(corners, camera_matrix, dist_coeffs, size=TAG_SIZE):
    """(rvec, tvec, reprojection error in px) for one tag, or None.

    solvePnPGeneric with IPPE_SQUARE, not solvePnP: a planar square has two
    poses that reproject almost identically when viewed near-obliquely, and
    solvePnP returns one of them arbitrarily -- the published tag frame then
    visibly flips back and forth between frames. IPPE_SQUARE returns both with
    their reprojection errors, so the ambiguity is resolved rather than
    guessed. Measured on a synthetic 60-degree view the two errors are
    1.1e-06 px and 3.27 px, so the choice is not marginal.
    """
    count, rvecs, tvecs, errors = cv2.solvePnPGeneric(
        object_points(size), corners, camera_matrix, dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not count:
        return None
    # Each entry of `errors` is a 1x1 array, not a scalar; float() on it is
    # deprecated in NumPy and will raise in a future release.
    scalar_errors = [float(np.asarray(e).ravel()[0]) for e in errors]
    best = int(np.argmin(scalar_errors))
    return rvecs[best], tvecs[best], scalar_errors[best]


def tag_to_pedestal_top():
    """The pedestal top centre, expressed in the tag frame, metres.

    Unused by the detector; this is the natural home for the number and a
    consumer that wants to place an object on the station's pedestal needs it.

    The model faces +x, so tag x = model +y, tag y = model +z, tag z = model
    +x. The pedestal top centre is (0, 0, PEDESTAL_SIZE[2]) in the model frame
    and the tag origin is (BOARD_OFFSET_X, 0, TAG_CENTRE_HEIGHT).
    """
    return np.array([0.0,
                     PEDESTAL_SIZE[2] - TAG_CENTRE_HEIGHT,
                     -BOARD_OFFSET_X])
