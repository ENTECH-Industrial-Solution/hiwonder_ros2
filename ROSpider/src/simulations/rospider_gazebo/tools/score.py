#!/usr/bin/env python3
"""Score a finished mini-game run.

    python3 tools/score.py --seed 3

Reads every pastel cube's pose straight from Gazebo (the same `gz model`
read-back tools/capture_dataset.py uses -- the judges score where the cube
actually is, not where the robot thinks it put it) and the mission's elapsed
time from /mission/summary. A cube on the station whose marker matches its
colour scores 10, on any other station 3, anywhere else 0.

The seed must be the one the round was launched with: it decides which
marker colour is on which station (mini_game.launch.py).
"""

import argparse
import importlib.util
import math
import os
import re
import subprocess
import sys

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG_ROOT)

from rospider_gazebo import stations  # noqa: E402

# Same permutation as the launch file: import it rather than repeat it.
_spec = importlib.util.spec_from_file_location(
    'mini_game_launch', os.path.join(PKG_ROOT, 'launch', 'mini_game.launch.py'))
_launch = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_launch)

ON_STATION_RADIUS = 0.12     # m, around the pedestal top centre
ON_STATION_HEIGHT = 0.03     # m, tolerance on the cube's centre height


def cube_pose(world, name):
    out = subprocess.run(['gz', 'model', '-m', name, '-p'],
                         capture_output=True, text=True, timeout=10).stdout
    match = re.search(r'\[([-\d.e]+) ([-\d.e]+) ([-\d.e]+)\]', out)
    if not match:
        return None
    return tuple(float(v) for v in match.groups())


def station_tops(assignment):
    """{tag id: (colour, (x, y, z) of the pedestal top centre)}."""
    tops = {}
    for tag_id, (x, y, yaw) in _launch.STATIONS.items():
        # The pedestal is at the model origin; its top is PEDESTAL_SIZE[2] up.
        tops[tag_id] = (assignment[tag_id], (x, y, stations.PEDESTAL_SIZE[2]))
    return tops


def elapsed_seconds():
    try:
        out = subprocess.run(
            ['ros2', 'topic', 'echo', '/mission/summary', '--once', '--field', 'data'],
            capture_output=True, text=True, timeout=5).stdout
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None
    match = re.search(r'finished in (\d+) s', out)
    return int(match.group(1)) if match else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--world', default='arena')
    args = parser.parse_args()

    names = list(_launch.COLOURS)
    import random
    random.Random(args.seed).shuffle(names)
    assignment = dict(zip(sorted(_launch.STATIONS), names))
    tops = station_tops(assignment)

    total = 0
    print(f'seed {args.seed}: ' + ', '.join(f'tag {t} = {c}' for t, c in assignment.items()))
    for colour in _launch.COLOURS:
        pose = cube_pose(args.world, f'pick_cube_{colour}')
        if pose is None:
            print(f'  {colour:7s} not found in Gazebo                       0')
            continue
        where, points = 'floor', 0
        cube_centre_z = stations.PEDESTAL_SIZE[2] + 0.025
        for tag_id, (marker, (sx, sy, sz)) in tops.items():
            if math.hypot(pose[0] - sx, pose[1] - sy) <= ON_STATION_RADIUS \
                    and abs(pose[2] - cube_centre_z) <= ON_STATION_HEIGHT:
                where = f'station {tag_id} ({marker})'
                points = 10 if marker == colour else 3
        total += points
        print(f'  {colour:7s} at ({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f}) '
              f'{where:22s} {points:2d}')
    seconds = elapsed_seconds()
    time_text = f'{seconds // 60}:{seconds % 60:02d}' if seconds is not None else 'unknown'
    print(f'TOTAL {total} points, mission time {time_text}')


if __name__ == '__main__':
    main()
