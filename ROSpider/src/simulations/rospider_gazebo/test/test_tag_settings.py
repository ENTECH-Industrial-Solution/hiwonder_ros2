import pytest

from rospider_gazebo import tag_settings, tags


def test_defaults_carry_every_section():
    settings = tag_settings.defaults()
    assert settings['detector'] == tags.DETECTOR_DEFAULTS
    assert settings['max_reproj_error_px'] == 3.0
    assert settings['control'] == tag_settings.CONTROL_DEFAULTS
    assert settings['behaviors'] == {}
    # A copy, not the module constant: a caller mutating it must not
    # change the next caller's defaults.
    settings['detector']['adaptive_thresh_constant'] = 99
    assert tags.DETECTOR_DEFAULTS['adaptive_thresh_constant'] == 7.0


def test_tag_key_round_trips():
    assert tag_settings.tag_key(12) == 'tag12'
    assert tag_settings.tag_id('tag12') == 12
    for bad in ('12', 'tag', 'tag-1', 'tagx', 'station0'):
        with pytest.raises(ValueError):
            tag_settings.tag_id(bad)


def test_tag_key_rejects_negative_ids():
    # A negative id would format as 'tag-1', which tag_id() then rejects
    # (the pattern has no sign) -- reject it at the source instead, so the
    # tuner's "add id" gets one clear error rather than a row that bricks
    # every later merge().
    with pytest.raises(ValueError):
        tag_settings.tag_key(-1)


def test_tag_key_accepts_zero():
    # 0 is a valid tag id (the first station), not to be confused with the
    # falsy-looking edge that the negative check above must not also catch.
    assert tag_settings.tag_key(0) == 'tag0'


def test_from_flat_reads_dotted_parameter_names():
    # What automatically_declare_parameters_from_overrides makes of the
    # nested YAML: one flat parameter per leaf, dotted.
    settings = tag_settings.from_flat({
        'max_reproj_error_px': 2.5,
        'detector.adaptive_thresh_constant': 9.0,
        'detector.corner_refinement': 'subpix',
        'control.max_linear': 0.04,
        'behaviors.tag0.action': 'approach',
        'behaviors.tag0.standoff': 0.35,
        'behaviors.tag7.action': 'stop',
        # Not part of the settings: must be ignored, not rejected.
        'stations.station0': [0.9, 0.0, 3.14159],
        'use_sim_time': True,
        'tune': False,
    })
    assert settings['max_reproj_error_px'] == 2.5
    assert settings['detector']['adaptive_thresh_constant'] == 9.0
    assert settings['detector']['corner_refinement'] == 'subpix'
    assert settings['detector']['adaptive_thresh_win_size_min'] == 3
    assert settings['control']['max_linear'] == 0.04
    assert settings['control']['kp_yaw'] == tag_settings.CONTROL_DEFAULTS['kp_yaw']
    assert settings['behaviors'] == {
        'tag0': {'action': 'approach', 'standoff': 0.35},
        'tag7': {'action': 'stop', 'standoff': tag_settings.DEFAULT_STANDOFF},
    }


def test_from_flat_rejects_bad_values():
    with pytest.raises(KeyError):
        tag_settings.from_flat({'detector.nope': 1})
    with pytest.raises(ValueError):
        tag_settings.from_flat({'behaviors.tag0.action': 'dance'})
    with pytest.raises(ValueError):
        tag_settings.from_flat({'behaviors.zero.action': 'stop'})


def test_merge_overlays_key_by_key():
    baseline = tag_settings.defaults()
    baseline['behaviors']['tag0'] = {'action': 'approach', 'standoff': 0.4}
    merged = tag_settings.merge(baseline, {
        'detector': {'adaptive_thresh_win_size_max': 51},
        'control': {'kp_dist': 0.6},
        'behaviors': {'tag0': {'standoff': 0.3}, 'tag3': {'action': 'place'}},
    })
    assert merged['detector']['adaptive_thresh_win_size_max'] == 51
    assert merged['detector']['adaptive_thresh_win_size_min'] == 3
    assert merged['control']['kp_dist'] == 0.6
    assert merged['max_reproj_error_px'] == 3.0
    # A partial behaviour keeps the baseline's other field.
    assert merged['behaviors']['tag0'] == {'action': 'approach', 'standoff': 0.3}
    assert merged['behaviors']['tag3'] == {'action': 'place',
                                           'standoff': tag_settings.DEFAULT_STANDOFF}
    # And the baseline is untouched.
    assert baseline['detector']['adaptive_thresh_win_size_max'] == 23
    assert baseline['behaviors']['tag0']['standoff'] == 0.4


def test_merge_coerces_and_validates():
    baseline = tag_settings.defaults()
    merged = tag_settings.merge(baseline, {
        'detector': {'adaptive_thresh_win_size_min': '5'},
        'control': {'max_linear': '0.03'},
        'max_reproj_error_px': '4',
    })
    assert merged['detector']['adaptive_thresh_win_size_min'] == 5
    assert isinstance(merged['detector']['adaptive_thresh_win_size_min'], int)
    assert merged['control']['max_linear'] == 0.03
    assert merged['max_reproj_error_px'] == 4.0
    with pytest.raises(KeyError):
        tag_settings.merge(baseline, {'control': {'warp': 9}})
    with pytest.raises(KeyError):
        tag_settings.merge(baseline, {'behaviors_enabled': True})
    with pytest.raises(ValueError):
        tag_settings.merge(baseline, {'behaviors': {'tag0': {'action': 'x'}}})


def test_yaml_block_is_a_paste_ready_config():
    settings = tag_settings.defaults()
    settings['behaviors']['tag0'] = {'action': 'place', 'standoff': 0.3}
    text = tag_settings.yaml_block(settings)
    lines = text.splitlines()
    assert lines[0] == 'apriltag_detect:'
    assert lines[1] == '  ros__parameters:'
    assert '    max_reproj_error_px: 3.0' in lines
    assert '    detector:' in lines
    assert '      corner_refinement: none' in lines
    assert '    control:' in lines
    assert '      max_linear: 0.05' in lines
    assert '    behaviors:' in lines
    assert '      tag0: {action: place, standoff: 0.3}' in lines
    # It must parse back to the same settings.
    import yaml
    parsed = yaml.safe_load(text)['apriltag_detect']['ros__parameters']
    assert parsed['detector'] == settings['detector']
    assert parsed['control'] == settings['control']
    assert parsed['behaviors'] == settings['behaviors']
