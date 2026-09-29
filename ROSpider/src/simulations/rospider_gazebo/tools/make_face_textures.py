#!/usr/bin/env python3
"""Draw the per-face wall posters of worlds/slam_challenge.sdf (worlds/textures/face_*.png).

V-SLAM needs every wall face to look different: with one partition.png on all seven inner
walls, both faces, RTAB-Map accepted 4-5 wrong loop closures per run (room A matched onto the
corridor, 1.2 m off). Fixed seeds, so re-running draws the same images.

    python3 tools/make_face_textures.py
"""
import pathlib
import random

from PIL import Image, ImageDraw, ImageFont

PKG = pathlib.Path(__file__).resolve().parents[1]
WALLS = ('partition_n', 'partition_s', 'stub', 'maze_1', 'maze_2', 'maze_3', 'maze_4')
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'


def poster(path, seed, label):
    rnd = random.Random(seed)
    image = Image.new('RGB', (768, 512), tuple(rnd.randint(120, 235) for _ in range(3)))
    draw = ImageDraw.Draw(image)
    for _ in range(14):
        colour = tuple(rnd.randint(0, 255) for _ in range(3))
        x, y, s = rnd.randint(0, 740), rnd.randint(0, 480), rnd.randint(40, 170)
        kind = rnd.choice('rct')
        if kind == 'r':
            draw.rectangle([x, y, x + s, y + int(s * rnd.uniform(0.5, 1.5))], fill=colour)
        elif kind == 'c':
            draw.ellipse([x, y, x + s, y + s], fill=colour)
        else:
            draw.polygon([(x, y + s), (x + s // 2, y), (x + s, y + s)], fill=colour)
    draw.text((16, 430), label, fill=(15, 15, 15), font=ImageFont.truetype(FONT, 64))
    image.save(path)


def main():
    out = PKG / 'worlds' / 'textures'
    for number, (wall, side) in enumerate(((w, s) for w in WALLS for s in 'ab'), start=1):
        poster(out / f'face_{wall}_{side}.png', 1000 + number, f'W{number:02d}')


if __name__ == '__main__':
    main()
