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

# The geometry lives in stations.py, which imports no cv2: a launch file
# needs station_sdf(), and importing cv2 inside `ros2 launch` sets Qt's
# plugin path for every child process, which crashes the Gazebo GUI.
from rospider_gazebo.stations import (  # noqa: F401  (re-exported)
    BOARD_FACE, BOARD_OFFSET_X, BOARD_THICKNESS, MARKER_CENTRE_HEIGHT,
    MARKER_SIZE, PEDESTAL_SIZE, POST_SIZE, QUIET_MODULES, TAG_CENTRE_HEIGHT,
    TAG_MODULES, TAG_SIZE, board_face, station_sdf, tag_to_pedestal_top)

FAMILY = 'tag36h11'
_DICT_ID = cv2.aruco.DICT_APRILTAG_36h11

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
    tag_px = TAG_MODULES * module_px
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
