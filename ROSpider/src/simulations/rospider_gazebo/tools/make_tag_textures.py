#!/usr/bin/env python3
"""Write the texture and the station model for one or more AprilTag ids.

Not a ROS node and not installed: run it from the package source tree, and
commit what it produces.

    python3 tools/make_tag_textures.py 0 1 2

Both outputs come from rospider_gazebo/tags.py (tags.station_sdf writes the
model text), so the rendered tag and the board it is stretched across can
never disagree about scale.
"""

import argparse
import os
import sys

import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rospider_gazebo import tags  # noqa: E402  (needs the sys.path above)

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def write_station(tag_id, module_px):
    texture_dir = os.path.join(PKG_ROOT, 'worlds', 'textures')
    model_dir = os.path.join(PKG_ROOT, 'models', f'tag_station_{tag_id}')
    os.makedirs(model_dir, exist_ok=True)

    texture = os.path.join(texture_dir, f'tag_{tag_id}.png')
    cv2.imwrite(texture, tags.generate_tag_image(tag_id, module_px))

    model = os.path.join(model_dir, 'model.sdf')
    with open(model, 'w') as handle:
        handle.write(tags.station_sdf(tag_id))

    return texture, model


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ids', type=int, nargs='+', help='tag36h11 ids')
    parser.add_argument('--module-px', type=int, default=40,
                        help='pixels per tag module (default 40)')
    args = parser.parse_args()
    for tag_id in args.ids:
        for path in write_station(tag_id, args.module_px):
            print(f'wrote {os.path.relpath(path, PKG_ROOT)}')


if __name__ == '__main__':
    main()
