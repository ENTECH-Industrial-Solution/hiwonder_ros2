
import numpy as np
import pytest
import yaml

from rospider_gazebo import mission_plan

BASIC = """
mission:
  waypoints:
    home: [0.0, 0.0, 0.0]
    pick_table: [0.35, 0.0, 0.0]
    survey: [2.2, 0.0, 0.0]
  standoff: 0.3
  on_fail: skip
  steps:
    - survey: survey
    - goto: pick_table
    - pick: pink
    - deliver: by_marker
    - goto: home
    - deliver: 2
    - place: here
    - say: done
"""


def test_parse_basic():
    m = mission_plan.parse(yaml.safe_load(BASIC))
    assert m.waypoints['survey'] == (2.2, 0.0, 0.0)
    assert m.standoff == 0.3 and m.on_fail == 'skip'
    assert [s.block for s in m.steps] == [
        'survey', 'goto', 'pick', 'deliver', 'goto', 'deliver', 'place', 'say']
    assert m.steps[3].arg == 'by_marker'
    assert m.steps[5].arg == 2          # a tag id, coerced to int


@pytest.mark.parametrize('bad, message', [
    ({'steps': []}, 'empty'),
    ({'steps': [{'goto': 'nowhere'}]}, 'not a waypoint'),
    ({'steps': [{'dance': 'x'}]}, 'unknown block'),
    ({'steps': [{'deliver': 'somewhere'}]}, "'by_marker' or a tag"),
    ({'steps': [{'place': 'there'}]}, "place wants 'here'"),
    ({'steps': [{'goto': 'home', 'pick': 'pink'}]}, 'one "block: value"'),
    ({'on_fail': 'panic', 'steps': [{'say': 'x'}]}, 'on_fail'),
    ({'standoff': 5, 'steps': [{'say': 'x'}]}, 'standoff'),
    ({'waypoints': {'home': [1, 2]}, 'steps': [{'say': 'x'}]}, '[x, y, yaw]'),
])
def test_parse_names_the_problem(bad, message):
    data = {'mission': {'waypoints': {'home': [0, 0, 0]}}}
    data['mission'].update(bad)
    with pytest.raises(ValueError, match=message):
        mission_plan.parse(data)


def test_goal_in_front_of_tag():
    # A station at x 4.3 facing -x (its tag normal points toward -x): the
    # goal is 0.6 m back down the hall, facing +x, i.e. facing the tag.
    rotation = np.array([[0, 0, -1],
                         [-1, 0, 0],
                         [0, 1, 0]], dtype=float)    # z axis -> -x
    x, y, yaw = mission_plan.goal_in_front_of_tag((4.3, 0.5, 0.25), rotation, 0.6)
    assert (round(x, 3), round(y, 3)) == (3.7, 0.5)
    assert abs(yaw) < 1e-9
    with pytest.raises(ValueError):
        mission_plan.goal_in_front_of_tag((0, 0, 0), np.eye(3), 0.6)  # faces up


def test_pair_tags_with_markers():
    tags = {0: (100, 300), 1: (320, 300), 2: (540, 300)}
    boxes = [('pink', 80, 150, 130, 200),      # above tag 0
             ('sky', 300, 150, 350, 200),      # above tag 1
             ('yellow', 520, 360, 570, 400),   # BELOW tag 2: a cube, not a marker
             ('yellow', 700, 150, 750, 200),   # too far right for tag 2
             ('yellow', 530, 10, 560, 40)]      # a wall poster far above tag 2
    assert mission_plan.pair_tags_with_markers(tags, boxes) == {0: 'pink', 1: 'sky'}


def test_majority():
    assert mission_plan.majority({0: ['pink', 'pink', 'sky'], 1: [], 2: ['yellow']}) \
        == {0: 'pink', 2: 'yellow'}


def test_marker_boxes_segments_the_three_panel_colours():
    # The launch's pastel triples, as BGR, on a grey floor with a small
    # speck below the area threshold.
    frame = np.full((240, 320, 3), 190, np.uint8)
    frame[20:60, 20:60] = (204, 184, 250)      # pink  (0.98, 0.72, 0.80)
    frame[20:60, 120:160] = (158, 240, 250)    # yellow (0.98, 0.94, 0.62)
    frame[20:60, 220:260] = (242, 217, 173)    # sky   (0.68, 0.85, 0.95)
    frame[200:205, 10:15] = (204, 184, 250)    # too small
    boxes = mission_plan.marker_boxes(frame)
    assert sorted(b[0] for b in boxes) == ['pink', 'sky', 'yellow']
    by_colour = {b[0]: b[1:] for b in boxes}
    assert by_colour['pink'] == (20.0, 20.0, 60.0, 60.0)
    assert by_colour['sky'][0] == 220.0
