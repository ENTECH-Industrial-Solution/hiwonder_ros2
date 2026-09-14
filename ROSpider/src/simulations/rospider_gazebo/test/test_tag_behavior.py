from rospider_gazebo import tag_settings
from rospider_gazebo.tag_behavior import TagBehavior


def _behavior(**rows):
    """TagBehavior with default gains and the given tag<id>=action rows."""
    behaviors = {key: {'action': action, 'standoff': 0.4}
                 for key, action in rows.items()}
    return TagBehavior(dict(tag_settings.CONTROL_DEFAULTS), behaviors,
                       enabled=True)


def tvec(x, z):
    return [x, 0.0, z]


def test_disabled_publishes_nothing():
    b = _behavior(tag0='approach')
    b.enabled = False
    d = b.update(0.0, [(0, tvec(0.3, 1.0))])
    assert d.twist is None and d.place is None


def test_none_action_is_ignored():
    b = _behavior(tag0='none')
    d = b.update(0.0, [(0, tvec(0.3, 1.0))])
    assert d.twist is None
    # An unconfigured id is 'none' too.
    d = b.update(0.1, [(5, tvec(0.3, 1.0))])
    assert d.twist is None


def test_approach_turns_toward_and_drives_to_standoff():
    b = _behavior(tag0='approach')
    # Tag to the right (x > 0) and far: turn right (negative z) and go.
    d = b.update(0.0, [(0, tvec(0.10, 1.0))])
    linear, angular = d.twist
    assert linear > 0 and angular < 0
    # Gains: kp_yaw 1.5 * 0.10 = 0.15 < max 0.3; kp_dist 0.4 * 0.6 = 0.24
    # clamps at max_linear 0.05.
    assert abs(angular - (-0.15)) < 1e-9
    assert linear == 0.05
    # Tag to the left and too close: turn left and back up.
    d = b.update(0.1, [(0, tvec(-0.10, 0.3))])
    linear, angular = d.twist
    assert angular > 0 and linear < 0
    assert abs(linear - (-0.04)) < 1e-9


def test_deadbands_make_reached():
    b = _behavior(tag0='approach')
    d = b.update(0.0, [(0, tvec(0.01, 0.42))])
    assert d.twist == (0.0, 0.0)
    assert d.place is None
    assert 'reached' in d.status


def test_nearest_tag_wins_and_stop_beats_everything():
    b = _behavior(tag0='approach', tag1='approach', tag2='stop')
    d = b.update(0.0, [(0, tvec(0.0, 2.0)), (1, tvec(0.2, 1.0))])
    assert 'tag 1' in d.status
    d = b.update(0.1, [(0, tvec(0.0, 0.5)), (2, tvec(0.0, 3.0))])
    assert d.twist == (0.0, 0.0)
    assert 'tag 2' in d.status and 'stop' in d.status


def test_nearest_is_by_range_not_by_z():
    # A remembered tag behind the camera has a negative z; it must not win
    # over a visible tag half a metre ahead just because -2 < 0.5.
    b = _behavior(tag0='approach', tag1='approach')
    d = b.update(0.0, [(0, tvec(0.0, -2.0)), (1, tvec(0.0, 0.5))])
    assert 'tag 1' in d.status


def test_place_fires_once_then_holds():
    b = _behavior(tag0='place')
    d = b.update(0.0, [(0, tvec(0.0, 1.0))])
    assert d.place is None
    d = b.update(0.1, [(0, tvec(0.0, 0.4))])
    assert d.place == 0
    assert d.twist == (0.0, 0.0)
    # Still in view, even drifting out of the deadband: hold, no re-fire,
    # no driving while the arm is moving.
    d = b.update(0.2, [(0, tvec(0.1, 0.6))])
    assert d.place is None
    assert d.twist == (0.0, 0.0)
    # Out of view past the timeout resets; back in view can place again.
    d = b.update(0.3, [])
    assert d.twist == (0.0, 0.0)          # holding through the loss window
    d = b.update(1.0, [])
    assert d.twist == (0.0, 0.0)          # the one release zero
    d = b.update(1.1, [])
    assert d.twist is None
    d = b.update(1.2, [(0, tvec(0.0, 0.4))])
    assert d.place == 0


def test_place_does_not_refire_while_another_tag_steals_the_target():
    b = _behavior(tag0='place', tag1='approach')
    # tag0 reaches its place standoff and fires.
    d = b.update(0.0, [(0, tvec(0.0, 0.4))])
    assert d.place == 0
    # tag1 becomes nearer and steals the scalar target/phase -- tag0 never
    # left view, so it must not forget it is already placed.
    d = b.update(0.1, [(0, tvec(0.0, 0.4)), (1, tvec(0.0, 0.2))])
    assert d.place is None
    assert 'tag 1' in d.status
    # tag1 is gone again, tag0 alone at the same spot: still placed, no
    # re-fire, and it holds rather than driving.
    d = b.update(0.2, [(0, tvec(0.0, 0.4))])
    assert d.place is None
    assert d.twist == (0.0, 0.0)
    # tag0 actually leaves view and stays gone past lost_timeout.
    d = b.update(0.3, [])
    d = b.update(1.0, [])
    # Back in view: placed is forgotten, so it can fire again.
    d = b.update(1.1, [(0, tvec(0.0, 0.4))])
    assert d.place == 0


def test_lost_tag_releases_cmd_vel_after_timeout():
    b = _behavior(tag0='approach')
    b.update(0.0, [(0, tvec(0.0, 1.0))])
    d = b.update(0.2, [])
    assert d.twist == (0.0, 0.0)          # within lost_timeout: hold
    d = b.update(0.6, [])
    assert d.twist == (0.0, 0.0)          # past it: zero exactly once
    d = b.update(0.7, [])
    assert d.twist is None                # then leave cmd_vel alone
    d = b.update(0.8, [])
    assert d.twist is None


def test_disabling_mid_drive_stops_once():
    b = _behavior(tag0='approach')
    b.update(0.0, [(0, tvec(0.0, 1.0))])
    b.enabled = False
    d = b.update(0.1, [(0, tvec(0.0, 1.0))])
    assert d.twist == (0.0, 0.0)
    d = b.update(0.2, [(0, tvec(0.0, 1.0))])
    assert d.twist is None
    # Re-enabling starts from idle: the first frame tracks again.
    b.enabled = True
    d = b.update(0.3, [(0, tvec(0.0, 1.0))])
    assert d.twist[0] > 0


def test_configure_replaces_rows_live():
    b = _behavior(tag0='none')
    assert b.update(0.0, [(0, tvec(0.0, 1.0))]).twist is None
    b.configure(behaviors={'tag0': {'action': 'approach', 'standoff': 0.4}})
    assert b.update(0.1, [(0, tvec(0.0, 1.0))]).twist[0] > 0
    b.configure(control=dict(tag_settings.CONTROL_DEFAULTS, max_linear=0.02))
    assert b.update(0.2, [(0, tvec(0.0, 1.0))]).twist[0] == 0.02


def test_tag_behind_the_camera_turns_in_place():
    # A P law on x alone would BACK the robot through a station that is
    # behind it (z < 0 makes the range error negative). It must turn first.
    b = _behavior(tag0='approach')
    d = b.update(0.0, [(0, tvec(0.0, -1.0))])
    linear, angular = d.twist
    assert linear == 0.0
    assert abs(angular) == tag_settings.CONTROL_DEFAULTS['max_angular']
    assert 'turning' in d.status


def test_wide_bearing_turns_before_driving():
    b = _behavior(tag0='approach')
    # bearing atan2(0.5, 0.5) = 45 deg > turn_first_rad 0.35 (20 deg):
    # rotate right (tag is on the right, x > 0 => negative angular.z).
    d = b.update(0.0, [(0, tvec(0.5, 0.5))])
    assert d.twist == (0.0, -tag_settings.CONTROL_DEFAULTS['max_angular'])
    # Tag on the left, behind: rotate left.
    d = b.update(0.1, [(0, tvec(-0.5, -0.5))])
    assert d.twist == (0.0, tag_settings.CONTROL_DEFAULTS['max_angular'])
    # Inside the cone the P law takes over.
    d = b.update(0.2, [(0, tvec(0.1, 1.0))])
    assert d.twist[0] > 0.0
