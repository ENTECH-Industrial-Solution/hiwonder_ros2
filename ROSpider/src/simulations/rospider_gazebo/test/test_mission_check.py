import math

from rospider_gazebo import mission_check as mc

FORBIDDEN = ('speed', 'go_to', 'go_to_tag', 'pick', 'place', 'steps')
PAD_ROBOT = (0.56, 0.55, math.pi)          # the answer key's return pose, facing the pad


def _facts(**over):
    facts = {'in_room_b': True, 'robot': PAD_ROBOT, 'parked_tag': 3, 'lifted': ['blue'],
             'blocks': {'red': mc.block_spawn('red'), 'blue': (0.41, 0.56, 0.025)},
             'failure': None, 'failed_step': None}
    facts.update(over)
    return facts


def _run(**over):
    facts = _facts(**over)
    levels = mc.evaluate(facts)
    return [level.status for level in levels], mc.format_report(levels, facts)


def test_answer_key_passes():
    statuses, text = _run()
    assert statuses == ['pass'] * 3 and 'ผ่านครบทุกด่าน' in text


def test_never_in_room_b_fails_level_one_and_says_where_it_ended():
    statuses, text = _run(in_room_b=False, robot=(0.6, 1.5, 0.0), parked_tag=None, lifted=[],
                          blocks={'red': mc.block_spawn('red'), 'blue': mc.block_spawn('blue')},
                          failure='มองไม่เห็นป้ายหมายเลข 2', failed_step=2)
    assert statuses == ['fail', 'skip', 'skip']
    assert '0.60' in text and 'ห้อง B' in text and 'ขั้นที่ 2' in text


def test_parking_at_the_red_station_fails_level_two_and_names_the_tag():
    statuses, text = _run(parked_tag=2, lifted=[],
                          blocks={'red': mc.block_spawn('red'), 'blue': mc.block_spawn('blue')},
                          failure='หยิบบล็อกสี blue ไม่ได้', failed_step=3)
    assert statuses == ['pass', 'fail', 'skip']
    assert 'หมายเลข 2' in text and 'แดง' in text


def test_lifting_red_fails_level_two_and_says_so():
    statuses, text = _run(parked_tag=2, lifted=['red'])
    assert statuses == ['pass', 'fail', 'skip'] and 'สีแดง' in text


def test_blue_put_down_at_the_spawn_fails_level_three_with_distance_and_side():
    statuses, text = _run(robot=(0.0, 0.0, 0.0), blocks={'red': mc.block_spawn('red'),
                                                         'blue': (0.16, 0.0, 0.025)})
    assert statuses == ['pass', 'pass', 'fail']
    assert '0.60' in text                       # hypot(0.40 - 0.16, 0.55 - 0.0)


def test_blue_still_held_is_not_on_the_pad():
    statuses, _text = _run(blocks={'red': mc.block_spawn('red'), 'blue': (0.40, 0.55, 0.22)})
    assert statuses[2] == 'fail'


def test_unreadable_block_pose_fails_level_three_plainly():
    statuses, text = _run(blocks={'red': None, 'blue': None})
    assert statuses[2] == 'fail' and 'Gazebo' in text


def test_hints_name_no_key():
    cases = [dict(in_room_b=False, lifted=[], parked_tag=None),
             dict(parked_tag=2, lifted=[]), dict(parked_tag=2, lifted=['red']),
             dict(blocks={'red': None, 'blue': (0.16, 0.0, 0.025)})]
    for over in cases:
        text = _run(**over)[1]
        hints = [line for line in text.splitlines() if 'คำใบ้' in line]
        assert not any(key in line for key in FORBIDDEN for line in hints)
