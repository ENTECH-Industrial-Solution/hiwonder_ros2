#!/usr/bin/env python3
"""Write worlds/textures/line_track.png, the floor of worlds/line_track.sdf.

Not a ROS node and not installed: run it from the package source tree and
commit what it produces.

    python3 tools/make_line_track.py

The floor is the room's 4 m x 3 m at PX_PER_M, the same grey as
rospider_room's ground plane, with one closed black line LINE_WIDTH wide.
The image is stored transposed -- image columns run along world y and rows
along world x -- because that is how Gazebo (ogre2) lays a texture over a
box's top face: drawn the natural way round, the loop came out turned 90
degrees and stretched 4:3. The loop is symmetric in both axes, so the
sense of each axis does not matter, only which is which. track_points()
is the curve itself, so the launch file that spawns the robot on it and
this drawing agree.
"""

import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rospider_gazebo.line_track import (  # noqa: E402  (needs the sys.path above)
    FLOOR_SIZE, LINE_WIDTH, track_points)

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PX_PER_M = 400
FLOOR_BGR = (185, 190, 190)
LINE_BGR = (20, 20, 20)


def to_pixels(points):
    w, h = FLOOR_SIZE
    px = np.empty((len(points), 2), dtype=np.int32)
    px[:, 0] = np.round((points[:, 1] + h / 2) * PX_PER_M)      # column: world y
    px[:, 1] = np.round((points[:, 0] + w / 2) * PX_PER_M)      # row: world x
    return px


def main():
    w, h = FLOOR_SIZE
    image = np.empty((int(w * PX_PER_M), int(h * PX_PER_M), 3), dtype=np.uint8)
    image[:] = FLOOR_BGR
    points = track_points()
    cv2.polylines(image, [to_pixels(points)], True, LINE_BGR,
                  int(round(LINE_WIDTH * PX_PER_M)), cv2.LINE_AA)
    path = os.path.join(PKG_ROOT, 'worlds', 'textures', 'line_track.png')
    cv2.imwrite(path, image)
    print(f'wrote {os.path.relpath(path, PKG_ROOT)}')


if __name__ == '__main__':
    main()
