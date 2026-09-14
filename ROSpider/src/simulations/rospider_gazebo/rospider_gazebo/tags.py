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

# The aruco DetectorParameters the tuner exposes, with OpenCV's stock values.
# Names are snake_case so they can be YAML keys and ROS parameters; the
# mapping to aruco's camelCase attributes is in detector_parameters().
DETECTOR_DEFAULTS = {
    'adaptive_thresh_win_size_min': 3,
    'adaptive_thresh_win_size_max': 23,
    'adaptive_thresh_win_size_step': 10,
    'adaptive_thresh_constant': 7.0,
    'min_marker_perimeter_rate': 0.03,
    'polygonal_approx_accuracy_rate': 0.03,
    'corner_refinement': 'none',
}

_CORNER_REFINEMENT = {
    'none': cv2.aruco.CORNER_REFINE_NONE,
    'subpix': cv2.aruco.CORNER_REFINE_SUBPIX,
}


def _odd_window(value):
    """aruco's adaptive threshold wants an odd window of at least 3.

    OpenCV bumps an even value itself, silently; doing it here means the
    number the GUI shows is the number the detector uses.
    """
    value = max(3, int(value))
    return value if value % 2 else value + 1


def detector_parameters(values=None):
    """A cv2.aruco.DetectorParameters from a DETECTOR_DEFAULTS-shaped dict.

    Missing keys take the stock value; unknown keys raise, because a typo in
    the YAML would otherwise be accepted and do nothing, which in a tuner
    looks like "the slider is broken".
    """
    values = dict(DETECTOR_DEFAULTS, **(values or {}))
    unknown = set(values) - set(DETECTOR_DEFAULTS)
    if unknown:
        raise KeyError(f'unknown detector parameter(s): {sorted(unknown)}')
    refinement = str(values['corner_refinement'])
    if refinement not in _CORNER_REFINEMENT:
        raise ValueError(
            f'corner_refinement must be one of '
            f'{sorted(_CORNER_REFINEMENT)}, not {refinement!r}')

    win_min = _odd_window(values['adaptive_thresh_win_size_min'])
    win_max = _odd_window(values['adaptive_thresh_win_size_max'])
    if win_min > win_max:
        # aruco steps the adaptive threshold window from min to max; a min
        # above max makes it try zero scales, so the tag just silently stops
        # being found rather than raising anywhere.
        raise ValueError(
            f'adaptive_thresh_win_size_min ({win_min}) must not exceed '
            f'adaptive_thresh_win_size_max ({win_max})')

    params = cv2.aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMin = win_min
    params.adaptiveThreshWinSizeMax = win_max
    params.adaptiveThreshWinSizeStep = max(
        1, int(values['adaptive_thresh_win_size_step']))
    params.adaptiveThreshConstant = float(values['adaptive_thresh_constant'])
    params.minMarkerPerimeterRate = float(values['min_marker_perimeter_rate'])
    params.polygonalApproxAccuracyRate = float(
        values['polygonal_approx_accuracy_rate'])
    params.cornerRefinementMethod = _CORNER_REFINEMENT[refinement]
    return params


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


def detect_tags(gray, params=None):
    """[(tag_id, corners)] for every tag36h11 in a grayscale image.

    corners is (4, 2) float32 in the same order as object_points().
    `params` is a cv2.aruco.DetectorParameters, see detector_parameters();
    None means OpenCV's stock values.
    """
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(_DICT_ID),
        params if params is not None else detector_parameters())
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


def quaternion_from_rvec(rvec):
    """Rodrigues rotation vector -> (x, y, z, w), the order ROS uses.

    Written out rather than pulled from tf_transformations, which is not a
    declared dependency of this package and is not needed for four lines of
    algebra. The branch on the trace is not an optimisation: the direct
    formula divides by sqrt(trace + 1), which goes to zero for rotations near
    180 degrees and loses all precision there.
    """
    matrix, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 0.5 / np.sqrt(trace + 1.0)
        return (float((matrix[2, 1] - matrix[1, 2]) * scale),
                float((matrix[0, 2] - matrix[2, 0]) * scale),
                float((matrix[1, 0] - matrix[0, 1]) * scale),
                float(0.25 / scale))
    i = int(np.argmax(np.diag(matrix)))
    j, k = (i + 1) % 3, (i + 2) % 3
    scale = float(np.sqrt(matrix[i, i] - matrix[j, j] - matrix[k, k] + 1.0) * 2.0)
    quaternion = [0.0, 0.0, 0.0, 0.0]
    quaternion[i] = 0.25 * scale
    quaternion[j] = float((matrix[j, i] + matrix[i, j]) / scale)
    quaternion[k] = float((matrix[k, i] + matrix[i, k]) / scale)
    quaternion[3] = float((matrix[k, j] - matrix[j, k]) / scale)
    return tuple(quaternion)
