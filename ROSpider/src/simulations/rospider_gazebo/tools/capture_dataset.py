#!/usr/bin/env python3
"""Capture a labelled YOLO dataset from a running simulation.

Not installed and not part of any launch file. Start the simulation first,
then run this from the package source tree:

    ros2 launch rospider_gazebo pick_place.launch.py auto_start:=false tags:=false
    python3 tools/capture_dataset.py --samples 400 --out ~/datasets/cubes

The tool places the objects with Gazebo's own set_pose service and then READS
BACK where they actually ended up. Trusting the commanded pose does not work:
the objects are dynamic bodies, so a cube told to go to (0.28, 0.11, 0.105)
was measured settling at (0.299, 0.110, 0.025) -- it slid 2 cm and fell off
the pedestal onto the floor. Labels built from the commanded pose are
systematically wrong in a way that looks fine in the label file and trains a
confidently wrong model.

The read-back goes through the gz CLI, not through config/gz_bridge.yaml. The
bridge stays as it is: pose/info is a high-rate topic, this tool runs offline
by hand, and SIMULATION.md already records what the camera bridge alone costs
in CPU here.

Labels are geometric, not hand-drawn: the object's eight corners are projected
through the live camera_info and the live TF, and the depth image is used to
throw away anything that turns out to be occluded.

ASSUMPTION: the robot has not been driven since the simulation started. Object
poses are set in the world frame, and the tool reads the camera through the
`odom` frame, which coincides with the world origin at spawn. The tool never
drives the robot; if you have, restart the simulation before capturing.
"""

import argparse
import os
import random
import re
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
import rclpy.duration
import tf2_ros
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rospider_gazebo import labelling  # noqa: E402  (needs the sys.path above)

# model name -> (class name, size in metres). The cubes pick_place.launch.py
# spawns; change this to capture something else. The class names must also
# appear in config/pick_place.yaml's `colors` for pick_and_place to accept
# them.
OBJECTS = {
    'pick_cube_red': ('red', (0.05, 0.05, 0.05)),
    'pick_cube_green': ('green', (0.05, 0.05, 0.05)),
    'pick_cube_blue': ('blue', (0.05, 0.05, 0.05)),
}

CAMERA_FRAME = 'depth_cam_frame'
WORLD_FRAME = 'odom'


def set_pose(world, name, position, yaw, attempts=3):
    """Move a model. Raises once the retries are exhausted.

    A pose that did not take would produce a correctly formatted label for the
    wrong place, which is worse than a crash: nothing downstream can detect
    it. So failure is still fatal -- but not on the first timeout. gz's
    service call times out occasionally under load, and losing a 400-sample
    run to one transient timeout is its own kind of failure.
    """
    request = (f'name: "{name}", position: {{x: {position[0]}, '
               f'y: {position[1]}, z: {position[2]}}}, '
               f'orientation: {{z: {np.sin(yaw / 2):.6f}, '
               f'w: {np.cos(yaw / 2):.6f}}}')
    for attempt in range(attempts):
        result = subprocess.run(
            ['gz', 'service', '-s', f'/world/{world}/set_pose',
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '5000', '--req', request],
            capture_output=True, text=True)
        if result.returncode == 0 and 'true' in result.stdout:
            return
        if attempt + 1 < attempts:
            time.sleep(1.0)
    raise RuntimeError(
        f'set_pose failed for {name} after {attempts} attempts: '
        f'{result.stdout}{result.stderr}\n'
        'Is the simulation running, and is install/local_setup.bash '
        'sourced so gz can find its config?')


_POSE_BLOCK = re.compile(
    r'name:\s*"(?P<name>[^"]+)".*?'
    r'position\s*{(?P<position>[^}]*)}.*?'
    r'orientation\s*{(?P<orientation>[^}]*)}',
    re.DOTALL)


def _fields(text):
    return {k: float(v) for k, v in re.findall(r'([xyzw]):\s*(\S+)', text)}


def read_poses(world, names):
    """{name: (position, quaternion)} for the models that are currently moving.

    dynamic_pose/info only carries bodies physics is simulating, which is
    exactly the set whose commanded pose cannot be trusted.
    """
    result = subprocess.run(
        ['gz', 'topic', '-e', '-t', f'/world/{world}/dynamic_pose/info', '-n', '1'],
        capture_output=True, text=True, timeout=15)
    if result.returncode != 0:
        raise RuntimeError(f'could not read dynamic_pose/info: {result.stderr}')
    poses = {}
    for match in _POSE_BLOCK.finditer(result.stdout):
        if match.group('name') not in names:
            continue
        position = _fields(match.group('position'))
        orientation = _fields(match.group('orientation'))
        poses[match.group('name')] = (
            np.array([position.get('x', 0.0), position.get('y', 0.0),
                      position.get('z', 0.0)]),
            (orientation.get('x', 0.0), orientation.get('y', 0.0),
             orientation.get('z', 0.0), orientation.get('w', 1.0)))
    return poses


def wait_until_settled(node, world, names, tolerance=0.001, timeout=8.0):
    """Poll until two consecutive reads agree, then return the settled poses.

    A teleported cube keeps moving: it drops onto whatever is under it, slides
    and can tip onto an edge. Capturing before it stops pairs the image with a
    pose the object has already left.

    `node` is spun between polls rather than plain sleeping. Settling takes a
    couple of seconds per sample, and a node that stops spinning stops filling
    its TF buffer; the next lookup then fails with ExtrapolationException
    because the buffer has a hole where the wait was.
    """
    deadline = time.monotonic() + timeout
    previous = read_poses(world, names)
    while time.monotonic() < deadline:
        spin_until = time.monotonic() + 0.25
        while time.monotonic() < spin_until:
            rclpy.spin_once(node, timeout_sec=0.05)
        current = read_poses(world, names)
        if set(current) == set(names) and all(
                np.linalg.norm(current[n][0] - previous[n][0]) < tolerance
                for n in names):
            return current
        previous = current
    return previous


def _rotation_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class CaptureNode(Node):

    def __init__(self, args):
        # use_sim_time so this node's view of time matches the stamps on the
        # camera frames and TF it reads. Set here rather than left to the
        # caller: the tool is run by hand and forgetting --ros-args would make
        # every lookup fail in a way that reads as a TF problem.
        super().__init__('capture_dataset',
                         parameter_overrides=[
                             Parameter('use_sim_time', value=True)])
        self.args = args
        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        self.camera_matrix = None

        # 30 s rather than the 10 s default: a sample takes a couple of
        # seconds and a slow one must not age its own frame out of the cache.
        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Image, '/depth_cam/rgb/image_raw',
                                 self._on_rgb, 1)
        self.create_subscription(Image, '/depth_cam/depth/image_raw',
                                 self._on_depth, 1)
        self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info',
                                 self._on_info, 1)

    def _on_rgb(self, msg):
        self.rgb = (msg.header.stamp, self.bridge.imgmsg_to_cv2(msg, 'bgr8'))

    def _on_depth(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg)

    def _on_info(self, msg):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def wait_for_tf(self, timeout=30.0):
        """Spin until the camera can be resolved in the world frame.

        robot_state_publisher's /tf_static is latched but does not arrive the
        instant this node starts, and the gz odometry bridge takes a moment
        too. Looking up straight away raises ConnectivityException -- "two or
        more unconnected trees" -- which reads like a broken TF tree rather
        than a node that asked too early.
        """
        deadline = time.monotonic() + timeout
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.tf_buffer.can_transform(
                    CAMERA_FRAME, WORLD_FRAME, rclpy.time.Time()):
                return
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f'no transform {WORLD_FRAME} -> {CAMERA_FRAME} after '
                    f'{timeout:.0f}s; is the simulation running with the '
                    'robot spawned?')

    def wait_for_fresh_frame(self, timeout=10.0):
        """Block until an RGB frame newer than the last one arrives.

        Not a sleep: the camera renders lazily (config/gz_bridge.yaml sets
        lazy: true) and its first frame after a new subscriber attaches is
        black. Sleeping a fixed amount would silently pair a stale image with
        the new poses.
        """
        previous = self.rgb[0] if self.rgb else None
        deadline = time.monotonic() + timeout
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.rgb is not None and self.rgb[0] != previous
                    and self.depth is not None
                    and self.camera_matrix is not None):
                return self.rgb
            if time.monotonic() > deadline:
                raise TimeoutError(
                    'no new camera frame; is the simulation running and is '
                    'something subscribed to the camera?')

    def camera_from_world(self, stamp=None):
        """The world -> camera transform, at the stamp of the frame being
        labelled.

        Not "latest": the label describes one image, so it has to use the
        camera pose at the moment that image was taken. Falls back to latest
        if the buffer no longer holds that instant, which is correct enough
        here because the robot and the arm do not move during a capture run.
        """
        when = rclpy.time.Time.from_msg(stamp) if stamp is not None \
            else rclpy.time.Time()
        if stamp is not None and not self.tf_buffer.can_transform(
                CAMERA_FRAME, WORLD_FRAME, when):
            when = rclpy.time.Time()
        tf = self.tf_buffer.lookup_transform(
            CAMERA_FRAME, WORLD_FRAME, when)
        t = tf.transform.translation
        r = tf.transform.rotation
        matrix = np.eye(4)
        matrix[:3, :3] = _rotation_matrix(r.x, r.y, r.z, r.w)
        matrix[:3, 3] = [t.x, t.y, t.z]
        return matrix

    def label_for(self, size, position, quaternion, matrix, frame):
        """The pixel box for this object, or None if it is not usefully visible."""
        corners_cam = labelling.transform_points(
            labelling.box_corners(size, position, quaternion), matrix)
        pixels = labelling.project_points(corners_cam, self.camera_matrix)
        box = labelling.bounding_box(
            pixels, frame.shape[1], frame.shape[0], self.args.min_area_px)
        if box is None:
            return None
        if self.occluded(box, corners_cam):
            return None
        return box

    def occluded(self, box, corners_cam):
        """True when the depth image says something nearer is in the way.

        The projection has no idea what is in front of what. Without this a
        cube behind the pedestal is labelled as if it were visible, and the
        model learns to hallucinate it.
        """
        u = int((box[0] + box[2]) / 2.0)
        v = int((box[1] + box[3]) / 2.0)
        patch = self.depth[max(0, v - 2):v + 3, max(0, u - 2):u + 3]
        patch = patch[np.isfinite(patch) & (patch > 0.0)]
        if patch.size == 0:
            return True
        expected = float(np.min(corners_cam[:, 2]))
        return float(np.median(patch)) < expected - self.args.occlusion_tol


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=int, default=200)
    parser.add_argument('--out', required=True)
    parser.add_argument('--world', default='rospider_room')
    parser.add_argument('--val-split', type=float, default=0.2)
    parser.add_argument('--min-area-px', type=float, default=300.0)
    parser.add_argument('--occlusion-tol', type=float, default=0.03)
    # The pedestal is 0.14 x 0.22 centred at x = 0.200, so its top spans
    # x 0.13..0.27 and y -0.11..0.11. Staying inside that, minus half a cube,
    # keeps most drops on the pedestal instead of on the floor. Labels are
    # correct either way now -- this is about the data being varied rather
    # than every cube ending up in the same place on the floor.
    parser.add_argument('--x-range', type=float, nargs=2, default=[0.16, 0.25])
    parser.add_argument('--y-range', type=float, nargs=2, default=[-0.08, 0.08])
    # Dropped from just above the pedestal top (0.08) so the cube settles onto
    # it rather than being spawned interpenetrating it.
    parser.add_argument('--z', type=float, default=0.12)
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args()

    random.seed(args.seed)
    classes = [name for name, _ in OBJECTS.values()]
    root = os.path.expanduser(args.out)
    for split in ('train', 'val'):
        os.makedirs(os.path.join(root, 'images', split), exist_ok=True)
        os.makedirs(os.path.join(root, 'labels', split), exist_ok=True)

    rclpy.init()
    node = CaptureNode(args)
    written = 0
    try:
        node.wait_for_tf()
        for index in range(args.samples):
            for model in OBJECTS:
                set_pose(args.world,
                         model,
                         (random.uniform(*args.x_range),
                          random.uniform(*args.y_range),
                          args.z),
                         random.uniform(-np.pi, np.pi))

            # Where they ACTUALLY are, once they have stopped moving. The
            # commanded pose is a request, not a fact.
            settled = wait_until_settled(node, args.world, list(OBJECTS))
            stamp, frame = node.wait_for_fresh_frame()
            matrix = node.camera_from_world(stamp)

            lines = []
            for model, (class_name, size) in OBJECTS.items():
                if model not in settled:
                    continue
                position, quaternion = settled[model]
                box = node.label_for(size, position, quaternion, matrix, frame)
                if box is not None:
                    lines.append(labelling.yolo_line(
                        classes.index(class_name), box,
                        frame.shape[1], frame.shape[0]))
            if not lines:
                continue

            split = 'val' if random.random() < args.val_split else 'train'
            stem = f'{index:05d}'
            cv2.imwrite(os.path.join(root, 'images', split, stem + '.jpg'), frame)
            with open(os.path.join(root, 'labels', split, stem + '.txt'), 'w') as f:
                f.write('\n'.join(lines) + '\n')
            written += 1
            if written % 20 == 0:
                print(f'{written} samples')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    with open(os.path.join(root, 'data.yaml'), 'w') as f:
        f.write(f'path: {root}\ntrain: images/train\nval: images/val\nnames:\n')
        for i, name in enumerate(classes):
            f.write(f'  {i}: {name}\n')
    print(f'wrote {written} samples and data.yaml to {root}')


if __name__ == '__main__':
    main()
