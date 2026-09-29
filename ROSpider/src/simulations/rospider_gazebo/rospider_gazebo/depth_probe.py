"""Depth-image probes and the two driving policies built on them.

Ported from example/rgbd_example/include/{prevent_falling_node,
cross_bridge_node}.py; the probes are used by pick_and_place, track_and_grab
and shape_detect.

One thing had to change for the simulation. The real depth camera publishes
16UC1 in millimetres; Gazebo's publishes 32FC1 in metres, with inf where the
ray hits nothing. `to_millimetres` normalises both to a float array of
millimetres with 0 for "no reading", so every ROI threshold below keeps
upstream's units and upstream's numbers.

No cv2 and no ROS here, so test/test_depth_probe.py can drive the policies
frame by frame.
"""

import numpy as np

# ROI boxes as [y_min, y_max, x_min, x_max], straight from upstream. They
# assume a 640x480 depth image, which is what config/gz_bridge.yaml delivers.
FALL_ROIS = {
    'left': [245, 255, 95, 105],
    'center': [245, 255, 315, 325],
    'right': [245, 255, 535, 545],
}
BRIDGE_ROIS = {
    'left_1': [245, 255, 115, 125],
    'left': [245, 255, 185, 195],
    'center': [245, 255, 315, 325],
    'right': [245, 255, 445, 455],
    'right_1': [245, 255, 515, 525],
}


def to_millimetres(depth_image):
    """A depth frame as float millimetres, 0 where there is no reading.

    Accepts the simulator's 32FC1 metres and the robot's 16UC1 millimetres.
    """
    depth = np.asarray(depth_image)
    if np.issubdtype(depth.dtype, np.floating):
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0) * 1000.0
    return depth.astype(np.float64)


def roi_distance(depth_mm, roi):
    """Mean distance in metres over an ROI, or 0.0 when nothing is in range.

    Upstream's get_roi_distance: readings outside (0, 30000) mm are dropped,
    an empty ROI comes back as 0, and the result is rounded to millimetres.
    """
    patch = depth_mm[roi[0]:roi[1], roi[2]:roi[3]]
    valid = patch[np.logical_and(patch > 0, patch < 30000)]
    if valid.size == 0:
        return 0.0
    return round(float(np.mean(valid)) / 1000.0, 3)


def roi_center(roi):
    """The (x, y) pixel at the middle of an ROI, for drawing."""
    return (int((roi[2] + roi[3]) / 2), int((roi[0] + roi[1]) / 2))


class FallPolicy:
    """Walk forward; turn away when any probe stops reading the floor.

    Ported from prevent_falling_node.move_policy. `update` returns the twist
    to publish as (linear_x, angular_z), or None while the policy wants the
    previous command left alone -- upstream publishes nothing in that case,
    and the robot keeps doing what it was doing.
    """

    def __init__(self, plane_distance, edge_tolerance=0.04, turn_speed=0.2,
                 forward_speed=0.05):
        self.plane_distance = plane_distance
        self.edge_tolerance = edge_tolerance
        self.turn_speed = turn_speed
        self.forward_speed = forward_speed
        self.turning = False
        self.timestamp = 0.0

    def update(self, distances, now):
        """distances: the floor distance under each of FALL_ROIS, in metres."""
        off_floor = any(abs(d - self.plane_distance) > self.edge_tolerance
                        for d in distances)
        if off_floor:
            self.turning = True
            self.timestamp = now + 0.3
            return 0.0, self.turn_speed
        if self.turning:
            if self.timestamp < now:
                self.turning = False
                self.timestamp = now + 0.2
                return 0.0, 0.0
            return None
        if self.timestamp < now:
            return self.forward_speed, 0.0
        return None


class BridgePolicy:
    """Creep along a narrow bridge, steering back when an outer probe drops off.

    Ported from cross_bridge_node.move_policy, minus two things the simulation
    has no equivalent for: the NARROW_POSE / DEFAULT_POSE body changes on
    /step_controller/cmd_param (there is no step_controller in the sim, and
    sim_gait walks at a fixed body height), and the 1.5 s sleeps upstream runs
    inside the policy. `update` reports the pose it would have asked for in
    `wanted_pose` so the caller can show it.
    """

    NARROW = 'NARROW_POSE'
    DEFAULT = 'DEFAULT_POSE'

    def __init__(self, plane_distance, edge_tolerance=0.04,
                 steer_tolerance=0.06, forward_speed=0.02):
        self.plane_distance = plane_distance
        self.edge_tolerance = edge_tolerance
        self.steer_tolerance = steer_tolerance
        self.forward_speed = forward_speed
        self.wanted_pose = self.DEFAULT
        self.on_bridge = False

    def _off(self, distance, tolerance):
        return abs(distance - self.plane_distance) > tolerance

    def update(self, left_1, left, center, right, right_1):
        """The five probe distances in metres -> (linear_x, angular_z)."""
        # Both outer probes off the floor: the robot is over the bridge, with
        # nothing but air to the sides. Upstream also tests right_1 against a
        # 0.0 tolerance, which is true whenever that probe reads anything at
        # all; kept, because dropping it would change when the demo creeps.
        self.on_bridge = (self._off(left_1, self.edge_tolerance)
                          and self._off(right_1, 0.0))
        if self.on_bridge:
            linear = self.forward_speed
            self.wanted_pose = self.NARROW
        else:
            linear = 0.0
            self.wanted_pose = self.DEFAULT

        angular = 0.0
        if not self.on_bridge:
            if self._off(left, self.steer_tolerance):
                angular = -0.2 if self._off(center, self.steer_tolerance) else -0.1
            elif self._off(right, self.steer_tolerance):
                angular = 0.2 if self._off(center, self.steer_tolerance) else 0.1

        # Nothing under any of the three inner probes: stop rather than walk
        # off whatever the robot is standing on.
        if all(self._off(d, self.steer_tolerance) for d in (left, center, right)):
            return 0.0, 0.0
        return linear, angular


# ------------------------------------------------------------ pixel -> point

def patch_depth(depth_image, u, v, window=5):
    """The median depth in a `window` pixel square round (u, v), in the
    image's own units, or None when no pixel there has a reading."""
    u, v = int(u), int(v)
    half = window // 2
    depth = np.asarray(depth_image)
    patch = depth[max(0, v - half):v + half + 1, max(0, u - half):u + half + 1]
    patch = patch[np.isfinite(patch) & (patch > 0)]
    if patch.size == 0:
        return None
    return float(np.median(patch))


def camera_point(u, v, depth, intrinsics):
    """Pixel (u, v) at `depth` metres as (x, y, z) in the optical frame.

    `intrinsics` is CameraInfo.k, the 3x3 matrix row by row.
    """
    fx, fy = intrinsics[0], intrinsics[4]
    cx, cy = intrinsics[2], intrinsics[5]
    return np.array([(u - cx) * depth / fx, (v - cy) * depth / fy, depth])


def quaternion_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def transform_point(point, translation, rotation):
    """`point` moved by a transform given as (x, y, z) and (x, y, z, w) --
    the fields of a geometry_msgs Transform, kept ROS-free for the tests."""
    return quaternion_matrix(*rotation) @ np.asarray(point, dtype=float) + np.asarray(translation, dtype=float)
