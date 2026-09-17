#!/usr/bin/env python3
"""Stand something on top of a tag -- the AR window.

A port of example/opencv_example/include/ar.py. Every tag36h11 in the camera
view gets a model drawn on it, in the tag's own pose: a wireframe cube by
default (`model:=rectangle`), or a Wavefront .obj given by `model_path`.

Differences from upstream, all forced by where the two run:

  * Detection and pose come from rospider_gazebo/tags.py -- OpenCV's aruco
    detector and IPPE_SQUARE -- the same pair scripts/apriltag_detect.py uses,
    rather than the `apriltag` module, which is not installed on a PC and
    whose object points are in half tag widths rather than metres.
  * Intrinsics come from /depth_cam/rgb/camera_info. Upstream hardcodes a
    real camera's matrix, which does not describe the simulated one.
  * The .obj models are not in this package. Upstream keeps five under
    example/example/opencv_example/include/3d_model/, which the example
    package does not install, so point `model_path` at one in the source
    tree and give it the scale upstream used for it:
    bicycle 50, fox 4, chair 400, cow 0.4, wolf 0.6.

    ros2 launch rospider_gazebo gazebo.launch.py
    ros2 launch rospider_gazebo vision_demo.launch.py demo:=ar_view

Nothing in the stock world carries a tag; spawn the tag stations with
`ros2 launch rospider_gazebo pick_place.launch.py scene:=false` first, or add
one from models/.
"""

import math

import cv2
import numpy as np
from rospider_gazebo import tags, vision_demo
from rospider_gazebo.objloader import OBJ
from rospider_gazebo.vision_demo import VisionDemo, banner

CUBE_EDGE = (0, 255, 0)
CUBE_TOP = (0, 0, 255)


def cube_points(size):
    """The eight corners of a cube standing on a tag of `size` metres."""
    half = size / 2.0
    return np.float32([[-half, -half, 0.0], [-half, half, 0.0],
                       [half, half, 0.0], [half, -half, 0.0],
                       [-half, -half, size], [-half, half, size],
                       [half, half, size], [half, -half, size]])


def draw_cube(image, imgpts):
    """Upstream's draw_rectangle(): filled base, pillars, outlined top."""
    imgpts = np.int32(imgpts).reshape(-1, 2)
    cv2.drawContours(image, [imgpts[:4]], -1, CUBE_EDGE, -3)
    for i, j in zip(range(4), range(4, 8)):
        cv2.line(image, tuple(imgpts[i]), tuple(imgpts[j]), (255, 255, 255), 3)
    cv2.drawContours(image, [imgpts[4:]], -1, CUBE_TOP, 3)
    return image


def load_model(path, scale, yaw_deg, tag_size):
    """An .obj as [(points in the tag frame, BGR colour or None), ...].

    Upstream scales the raw vertices by a per-model constant and then divides
    by 100 when projecting, which leaves the model in units of half a tag
    width. The same factor is folded in here so the numbers above stay
    meaningful while the model comes out in metres.
    """
    obj = OBJ(path, swapyz=True)
    metres = scale / 100.0 * (tag_size / 2.0)
    yaw = math.radians(yaw_deg)
    rotation = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                         [math.sin(yaw), math.cos(yaw), 0.0],
                         [0.0, 0.0, 1.0]])
    faces = []
    for vertices, _normals in reversed(obj.faces):
        points, colors = [], []
        for index in vertices:
            vertex = obj.vertices[index - 1]
            points.append(vertex[:3])
            if len(vertex) >= 6:
                colors.append(vertex[3:6])
        points = (np.array(points, dtype=float) * metres) @ rotation.T
        color = (tuple(255 * c for c in colors[0][::-1]) if colors else None)
        faces.append((points.astype(np.float32), color))
    return faces


class ArViewNode(VisionDemo):

    def __init__(self):
        super().__init__('ar_view', camera_info='/depth_cam/rgb/camera_info')
        self.tag_size = float(self.param('tag_size', tags.TAG_SIZE))
        self.model = str(self.param('model', 'rectangle'))
        self.model_color = tuple(int(v) for v in
                                 self.param('model_color', [255, 255, 0]))
        self.cube = cube_points(self.tag_size)
        self.faces = None
        path = str(self.param('model_path', ''))
        if path:
            self.faces = load_model(path,
                                    float(self.param('model_scale', 1.0)),
                                    float(self.param('model_yaw_deg', 0.0)),
                                    self.tag_size)
        elif self.model != 'rectangle':
            raise ValueError(
                f'model:={self.model} needs model_path:=<path to a .obj>; '
                'the .obj files live in the example package\'s source tree, '
                'which is not installed')

    def on_start(self):
        what = 'a cube' if self.faces is None else self.param('model_path', '')
        self.get_logger().info(f'drawing {what} on every {tags.FAMILY} tag')

    def process(self, frame):
        if self.intrinsics is None:
            return banner(frame, 'WAITING FOR camera_info',
                          scale=0.7, color=(200, 200, 200))
        camera_matrix = np.array(self.intrinsics, dtype=np.float64).reshape(3, 3)
        dist_coeffs = np.zeros(5)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found = tags.detect_tags(gray)
        for tag_id, corners in found:
            for corner in corners:
                cv2.circle(frame, (int(corner[0]), int(corner[1])), 2,
                           (0, 255, 255), -1)
            pose = tags.solve_tag_pose(corners, camera_matrix, dist_coeffs,
                                       self.tag_size)
            if pose is None:
                continue
            rvec, tvec, _error = pose
            if self.faces is None:
                imgpts, _ = cv2.projectPoints(self.cube, rvec, tvec,
                                              camera_matrix, dist_coeffs)
                draw_cube(frame, imgpts)
            else:
                self._draw_model(frame, rvec, tvec, camera_matrix, dist_coeffs)
        if not found:
            banner(frame, 'NO TAG', color=(200, 200, 200))
        return frame

    def _draw_model(self, frame, rvec, tvec, camera_matrix, dist_coeffs):
        for points, color in self.faces:
            projected, _ = cv2.projectPoints(points.reshape(-1, 1, 3), rvec,
                                             tvec, camera_matrix, dist_coeffs)
            cv2.fillConvexPoly(frame, projected.astype(np.int32),
                               color if color is not None else self.model_color)


def main():
    vision_demo.main(ArViewNode)


if __name__ == '__main__':
    main()
