"""Turn a known object pose into a YOLO label, without ROS.

tools/capture_dataset.py is the ROS wrapper. The maths is here because a
mislabelled dataset is the worst kind of bug in this pipeline: nothing errors,
training succeeds, and the model is confidently wrong. Tested under plain
pytest with no simulator.
"""

import numpy as np

_UNIT_CORNERS = np.array([[sx, sy, sz]
                          for sx in (-0.5, 0.5)
                          for sy in (-0.5, 0.5)
                          for sz in (-0.5, 0.5)], dtype=np.float64)


def rotation_matrix(quaternion):
    """3x3 rotation from an (x, y, z, w) quaternion."""
    x, y, z, w = (float(v) for v in quaternion)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def box_corners(size, position, quaternion=(0.0, 0.0, 0.0, 1.0)):
    """The eight vertices of an oriented box, in the world frame.

    `size` is (sx, sy, sz) in metres, `position` is the box centre.

    The orientation is a full quaternion, not a yaw. An object dropped into a
    physics simulation does not stay upright: it tips onto an edge or a face,
    and a yaw-only model would then describe a box the image does not contain.
    """
    corners = _UNIT_CORNERS * np.asarray(size, dtype=np.float64)
    return (corners @ rotation_matrix(quaternion).T
            + np.asarray(position, dtype=np.float64))


def transform_points(points, matrix):
    """Apply a 4x4 homogeneous transform to (n, 3) points."""
    points = np.asarray(points, dtype=np.float64)
    matrix = np.asarray(matrix)
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def project_points(points_cam, camera_matrix):
    """(n, 2) pixels for the points in front of an optical-frame camera.

    Points at z <= 0 are behind or in the plane of the camera and have no
    projection. Projecting them anyway flips their sign and plants a phantom
    box somewhere in the image, which is exactly the kind of label that
    silently poisons a dataset.
    """
    points_cam = np.asarray(points_cam, dtype=np.float64).reshape(-1, 3)
    points_cam = points_cam[points_cam[:, 2] > 0.0]
    if points_cam.size == 0:
        return np.empty((0, 2))
    projected = points_cam @ np.asarray(camera_matrix).T
    return projected[:, :2] / projected[:, 2:3]


def bounding_box(pixels, width, height, min_area_px):
    """(x1, y1, x2, y2) clamped to the image, or None if too little is visible.

    The threshold matters: an object mostly out of frame leaves a few pixels
    at the edge, and labelling that teaches the model that a thin strip is the
    object.
    """
    pixels = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
    if pixels.size == 0:
        return None
    x1 = float(np.clip(pixels[:, 0].min(), 0.0, width))
    y1 = float(np.clip(pixels[:, 1].min(), 0.0, height))
    x2 = float(np.clip(pixels[:, 0].max(), 0.0, width))
    y2 = float(np.clip(pixels[:, 1].max(), 0.0, height))
    if (x2 - x1) * (y2 - y1) < min_area_px:
        return None
    return x1, y1, x2, y2


def yolo_line(class_index, box, width, height):
    """One line of a YOLO label file: class, centre and size, all normalised."""
    x1, y1, x2, y2 = box
    return (f'{class_index} '
            f'{(x1 + x2) / 2.0 / width:.6f} '
            f'{(y1 + y2) / 2.0 / height:.6f} '
            f'{(x2 - x1) / width:.6f} '
            f'{(y2 - y1) / height:.6f}')
