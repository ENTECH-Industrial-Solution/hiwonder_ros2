"""Hand and body geometry for the MediaPipe demos.

Ported from example/mediapipe_example/include/{hand_gesture,pose_control}.py
and the helpers they take from driver/sdk/sdk/common.py. Only the geometry
lives here -- no MediaPipe, no cv2, no ROS -- so test/test_gestures.py can
check the classifier against hand-built landmark sets.

Landmarks are pixel coordinates: an (N, 2) array, y growing downwards, in
MediaPipe's own index order (21 hand points, 33 pose points).
"""

import math

import numpy as np

# Angle thresholds separating a bent finger from a straight one, as upstream
# tuned them in h_gesture().
THR_ANGLE = 65.0
THR_ANGLE_THUMB = 53.0
THR_ANGLE_STRAIGHT = 49.0

# The finger joint triples hand_angle() measures, thumb first.
_FINGER_VECTORS = (
    ((3, 4), (0, 2)),
    ((0, 6), (7, 8)),
    ((0, 10), (11, 12)),
    ((0, 14), (15, 16)),
    ((0, 18), (19, 20)),
)


def distance(point_1, point_2):
    """Distance between two points (sdk.common.distance)."""
    return math.hypot(point_1[0] - point_2[0], point_1[1] - point_2[1])


def box_center(box):
    """Centre of an (x1, y1, x2, y2) box (sdk.common.box_center)."""
    return (box[0] + box[2]) / 2, (box[1] + box[3]) / 2


def vector_2d_angle(v1, v2):
    """Signed angle from v1 to v2 in degrees, -180 to 180.

    sdk.common.vector_2d_angle uses np.cross on two 2-vectors, which numpy 2
    removed. The 2-D cross product is written out instead; the result is the
    same number.
    """
    v1 = np.asarray(v1, dtype=float)
    v2 = np.asarray(v2, dtype=float)
    norms = np.linalg.norm(v1) * np.linalg.norm(v2)
    if norms == 0:
        return 0.0
    cos = float(np.dot(v1, v2)) / norms
    sin = float(v1[0] * v2[1] - v1[1] * v2[0]) / norms
    return float(np.degrees(np.arctan2(sin, cos)))


def get_angle(p1, p2, p3):
    """Angle the path p1 -> p2 -> p3 turns through, in degrees."""
    p1, p2, p3 = (np.asarray(p, dtype=float) for p in (p1, p2, p3))
    return vector_2d_angle(p2 - p1, p3 - p2)


def landmarks_to_pixels(size, landmarks):
    """MediaPipe's normalised landmarks as an (N, 2) pixel array.

    `size` is (width, height); `landmarks` is any iterable of objects with
    .x and .y, which is what both the hands and the pose solution return.
    """
    width, height = size
    return np.array([(lm.x * width, lm.y * height) for lm in landmarks],
                    dtype=float)


def hand_angle(landmarks):
    """How far each of the five fingers is bent, in degrees, thumb first."""
    return [abs(vector_2d_angle(landmarks[a] - landmarks[b],
                                landmarks[c] - landmarks[d]))
            for (a, b), (c, d) in _FINGER_VECTORS]


def h_gesture(angle_list):
    """The gesture name for a set of finger angles.

    Same decision tree as upstream's h_gesture(): 'fist', 'gun', 'hand_heart',
    'one', 'two', 'three', 'four', 'five', 'six', 'OK', or 'none'.
    """
    thumb, index, middle, ring, pink = angle_list
    bent = [a > THR_ANGLE for a in angle_list]
    straight = [a < THR_ANGLE_STRAIGHT for a in angle_list]
    thumb_bent = thumb > THR_ANGLE_THUMB

    if thumb_bent and all(bent[1:]):
        return 'fist'
    if straight[0] and straight[1] and bent[2] and bent[3] and bent[4]:
        return 'gun'
    if straight[0] and bent[1] and bent[2] and bent[3] and bent[4]:
        return 'hand_heart'
    if thumb > 5 and straight[1] and bent[2] and bent[3] and bent[4]:
        return 'one'
    if thumb_bent and straight[1] and straight[2] and bent[3] and bent[4]:
        return 'two'
    if thumb_bent and straight[1] and straight[2] and straight[3] and bent[4]:
        return 'three'
    if thumb_bent and bent[1] and straight[2] and straight[3] and straight[4]:
        return 'OK'
    if thumb_bent and straight[1] and straight[2] and straight[3] and straight[4]:
        return 'four'
    if all(straight):
        return 'five'
    if straight[0] and bent[1] and bent[2] and bent[3] and straight[4]:
        return 'six'
    return 'none'


def hand_gesture(landmarks):
    """The gesture a hand's pixel landmarks are making."""
    return h_gesture(hand_angle(landmarks))


# --------------------------------------------------------------------- pose

def is_level(landmarks, angle_threshold=15):
    """True while the shoulder line is horizontal in the image."""
    p0 = landmarks[12].copy()
    p0[0] = 0
    return abs(get_angle(p0, landmarks[12], landmarks[11])) <= angle_threshold


def is_pentagon(landmarks):
    """True while both hands are raised above the head, arms out to the sides.

    The pose that starts imitation mode upstream: wrists closer together than
    the shoulders are wide, both above every head landmark, and both upper
    arms lifted more than 40 degrees off the shoulder line.
    """
    shoulder_width = distance(landmarks[12], landmarks[11])
    if distance(landmarks[16], landmarks[15]) > shoulder_width:
        return False
    for p in landmarks[:7]:     # nose, eyes and ears
        if landmarks[15][1] > p[1] or landmarks[16][1] > p[1]:
            return False
    if get_angle(landmarks[11], landmarks[12], landmarks[14]) < 40:
        return False
    if get_angle(landmarks[13], landmarks[11], landmarks[12]) < 40:
        return False
    return True


def is_flat(landmarks, angle_threshold=30):
    """True while both arms are stretched straight out along the shoulders."""
    arm_marks = (15, 13, 11, 12, 14, 16)
    for i in range(3):
        angle = get_angle(landmarks[arm_marks[i]],
                          landmarks[arm_marks[i + 1]],
                          landmarks[arm_marks[i + 2]])
        if abs(angle) > angle_threshold:
            return False
    return is_level(landmarks, angle_threshold)


def is_cross(landmarks):
    """True while the raised arms are crossed -- upstream's 'stop' pose."""
    if landmarks[16][0] <= landmarks[15][0]:
        return False
    return is_pentagon(landmarks)
