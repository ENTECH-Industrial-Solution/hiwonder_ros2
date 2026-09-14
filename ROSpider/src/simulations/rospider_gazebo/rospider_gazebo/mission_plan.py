"""The mini game's mission file and the small pieces of maths it needs.

scripts/mission.py is the ROS side: it runs the steps this module parses,
one block at a time, against Nav2, pick_and_place and apriltag_detect. What
is here has no ROS in it so test/test_mission_plan.py can pin it: a mission
file that fails to validate is the most common way a team's afternoon goes
wrong, and it should fail at load, naming the block, not twenty minutes into
a run.

A mission file:

    mission:
      waypoints:            # name: [x, y, yaw] in the map frame
        home: [0.0, 0.0, 0.0]
      standoff: 0.30        # metres from base_footprint at which place fires
      dock: 0.5             # metres crept along pick_table's heading before a pick, and back after
      on_fail: skip         # skip | retry | stop
      steps:
        - survey: survey    # goto, then read tags + marker colours
        - goto: pick_table
        - pick: pink
        - deliver: by_marker  # or a tag id
        - place: here
        - say: done
"""

import math

import cv2
import numpy as np

BLOCKS = {
    'goto': 'a waypoint name',
    'survey': 'a waypoint name',
    'pick': 'a colour / class name',
    'deliver': "'by_marker' or a tag id",
    'place': "'here'",
    'say': 'text',
}
ON_FAIL = ('skip', 'retry', 'stop')

# How far behind the standoff Nav2 is asked to stop, so the final
# approach by tag memory has something to do and Nav2's 5 cm tolerance
# cannot leave the robot already past the standoff.
NAV_MARGIN = 0.30


class Step:
    __slots__ = ('block', 'arg')

    def __init__(self, block, arg):
        self.block = block
        self.arg = arg

    def __repr__(self):
        return f'{self.block}: {self.arg}'


class Mission:

    def __init__(self, waypoints, steps, standoff=0.30, on_fail='skip',
                 dock=0.5):
        self.waypoints = waypoints
        self.steps = steps
        self.standoff = standoff
        self.on_fail = on_fail
        self.dock = dock


def parse(data):
    """A Mission from the loaded YAML dict; raises ValueError naming the
    first thing that is wrong."""
    if not isinstance(data, dict) or 'mission' not in data:
        raise ValueError("the file must have a top-level 'mission:' key")
    body = data['mission'] or {}
    waypoints = {}
    for name, pose in (body.get('waypoints') or {}).items():
        try:
            x, y, yaw = (float(v) for v in pose)
        except (TypeError, ValueError):
            raise ValueError(
                f'waypoint {name!r} must be [x, y, yaw], not {pose!r}')
        waypoints[str(name)] = (x, y, yaw)

    on_fail = str(body.get('on_fail', 'skip'))
    if on_fail not in ON_FAIL:
        raise ValueError(f'on_fail must be one of {ON_FAIL}, not {on_fail!r}')
    standoff = float(body.get('standoff', 0.30))
    if not 0.1 <= standoff <= 1.0:
        raise ValueError(f'standoff {standoff} m is outside 0.1..1.0')
    dock = float(body.get('dock', 0.5))
    if not 0.0 <= dock <= 1.0:
        raise ValueError(f'dock {dock} m is outside 0..1.0')

    raw_steps = body.get('steps')
    if not raw_steps:
        raise ValueError("'steps:' is empty")
    steps = []
    for index, raw in enumerate(raw_steps, 1):
        if not isinstance(raw, dict) or len(raw) != 1:
            raise ValueError(
                f'step {index} must be one "block: value" pair, not {raw!r}')
        (block, arg), = raw.items()
        if block not in BLOCKS:
            raise ValueError(
                f'step {index}: unknown block {block!r}; blocks are '
                f'{sorted(BLOCKS)}')
        if block in ('goto', 'survey'):
            if arg not in waypoints:
                raise ValueError(
                    f'step {index}: {block} {arg!r} is not a waypoint '
                    f'({sorted(waypoints)})')
        elif block == 'deliver':
            if arg != 'by_marker':
                try:
                    arg = int(arg)
                except (TypeError, ValueError):
                    raise ValueError(
                        f"step {index}: deliver wants 'by_marker' or a tag "
                        f'id, not {arg!r}')
        elif block == 'place':
            if arg != 'here':
                raise ValueError(f"step {index}: place wants 'here'")
        elif block in ('pick', 'say'):
            arg = str(arg)
        steps.append(Step(block, arg))
    return Mission(waypoints, steps, standoff, on_fail, dock)


def goal_in_front_of_tag(position, rotation, distance):
    """(x, y, yaw) in the tag's frame's parent, `distance` metres in front of
    the tag face and facing it.

    The tag frame's z axis points out of the face toward the viewer
    (rospider_gazebo/tags.py), so "in front" is +z, projected onto the
    floor. Facing the tag means yaw toward -z.
    """
    normal = np.asarray(rotation, dtype=float)[:, 2]
    flat = np.array([normal[0], normal[1]])
    length = float(np.linalg.norm(flat))
    if length < 1e-6:
        raise ValueError('the tag faces straight up or down')
    flat /= length
    x = float(position[0] + flat[0] * distance)
    y = float(position[1] + flat[1] * distance)
    yaw = math.atan2(-flat[1], -flat[0])
    return x, y, yaw


# HSV bands (OpenCV: H 0-179) for the stations' colour markers. The panels
# are emissive, so they render at nearly their nominal colour from any
# distance and any lighting -- unlike the cubes, whose bands are the teams'
# problem. The survey segments the panels itself with these rather than
# asking the team's detector, so a YOLO model trained on cubes (which
# only sometimes fires on a flat colour square) cannot break the survey.
# Measured from the survey point: the panels render at S 33-47, V 212-215;
# the wall posters of the same hues at V 104-124, so the V floor is what
# keeps them out.
MARKER_BANDS = {
    'pink': ((150, 25, 160), (179, 255, 255)),
    'yellow': ((15, 25, 160), (45, 255, 255)),
    'sky': ((88, 25, 160), (118, 255, 255)),
}


def marker_boxes(bgr, bands=MARKER_BANDS, min_area=300):
    """[(colour, x1, y1, x2, y2)] of the colour blobs in a BGR frame."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    boxes = []
    for colour, (lower, upper) in bands.items():
        mask = cv2.inRange(hsv, np.array(lower, np.uint8),
                           np.array(upper, np.uint8))
        count, _, stats, _ = cv2.connectedComponentsWithStats(mask)
        for i in range(1, count):
            x, y, w, h, area = stats[i]
            if area >= min_area:
                boxes.append((colour, float(x), float(y),
                              float(x + w), float(y + h)))
    return boxes


def pair_tags_with_markers(tag_pixels, boxes, max_dx=80.0, max_dy=200.0):
    """{tag id: colour} from one camera frame.

    `tag_pixels` is {tag id: (x, y)} of the tag centres and `boxes` is
    [(colour, x1, y1, x2, y2)] of the colour blobs. A marker is the panel
    just above its tag, so the box's centre must sit above the tag centre,
    within `max_dx` pixels of it horizontally and `max_dy` vertically (the
    wall posters behind the stations carry the same hues, higher up); the
    nearest such box wins.
    """
    pairs = {}
    for tag_id, (tx, ty) in tag_pixels.items():
        best = None
        for colour, x1, y1, x2, y2 in boxes:
            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            if cy >= ty or ty - cy > max_dy or abs(cx - tx) > max_dx:
                continue
            score = abs(cx - tx)
            if best is None or score < best[0]:
                best = (score, colour)
        if best is not None:
            pairs[tag_id] = best[1]
    return pairs


def majority(votes):
    """{tag id: colour} from {tag id: [colour, colour, ...]} by count."""
    result = {}
    for tag_id, colours in votes.items():
        if colours:
            result[tag_id] = max(set(colours), key=colours.count)
    return result
