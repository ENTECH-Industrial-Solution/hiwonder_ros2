"""The line on the floor of worlds/line_track.sdf, as a curve.

tools/make_line_track.py draws it into the floor texture and
line_following.launch.py spawns the robot on it, so both read the curve
from here rather than each keeping a copy.

Pure Python and numpy on purpose (the launch file imports it, and cv2
inside `ros2 launch` breaks the Gazebo GUI -- see stations.py).
"""

import math

import numpy as np

#: Floor drawn by the texture, metres: the room's inside.
FLOOR_SIZE = (4.0, 3.0)
#: Width of the black line, metres -- about the tape on Hiwonder's mat.
LINE_WIDTH = 0.03
#: Half-axes of the loop, metres, and the depth of its two inward bends.
LOOP_X, LOOP_Y, BEND = 1.3, 0.85, 0.25


def track_points(n=720):
    """The loop as an (n, 2) array of world x, y, counter-clockwise from
    its east-most point: an ellipse pinched in at north and south, so the
    robot meets bends both ways."""
    t = np.linspace(0, 2 * math.pi, n, endpoint=False)
    x = LOOP_X * np.cos(t)
    y = LOOP_Y * np.sin(t) + BEND * np.sin(3 * t)
    return np.column_stack([x, y])


def start_pose():
    """(x, y, yaw) on the line at its east-most point, facing along it."""
    points = track_points()
    x, y = points[0]
    dx, dy = points[1] - points[-1]
    return float(x), float(y), float(math.atan2(dy, dx))


def min_turn_radius(n=720):
    """Tightest bend of the loop, metres."""
    points = track_points(n)
    d1 = np.gradient(points, axis=0)
    d2 = np.gradient(d1, axis=0)
    curvature = np.abs(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / \
        np.hypot(d1[:, 0], d1[:, 1]) ** 3
    return float(1.0 / curvature.max())
