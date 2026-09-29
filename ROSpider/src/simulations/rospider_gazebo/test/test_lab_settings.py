import pytest
import yaml

from rospider_gazebo import lab_settings


def test_defaults_are_a_fresh_copy():
    settings = lab_settings.defaults()
    assert settings['colors'] == ['red', 'green', 'blue']
    assert set(settings['red']) == {'min', 'max'}
    settings['colors'].append('pink')
    assert lab_settings.defaults()['colors'] == ['red', 'green', 'blue']


def test_merge_overlays_key_by_key():
    base = lab_settings.defaults()
    merged = lab_settings.merge(base, {'min_area_px': '500',
                                       'red': {'min': [1, 2, 3], 'max': [4, 5, 6]}})
    assert merged['min_area_px'] == 500
    assert merged['red'] == {'min': [1, 2, 3], 'max': [4, 5, 6]}
    assert merged['green'] == base['green']
    assert base['min_area_px'] == 300          # untouched


def test_merge_keeps_colours_the_override_adds_and_drops_the_rest():
    base = lab_settings.defaults()
    merged = lab_settings.merge(base, {
        'colors': ['red', 'pink'],
        'pink': {'min': [0, 140, 0], 'max': [255, 255, 255]}})
    assert merged['colors'] == ['red', 'pink']
    assert 'green' not in merged
    assert merged['pink']['min'] == [0, 140, 0]


def test_merge_validates():
    base = lab_settings.defaults()
    with pytest.raises(KeyError):
        lab_settings.merge(base, {'hue': 3})
    with pytest.raises(ValueError):
        lab_settings.merge(base, {'red': {'min': [0, 0], 'max': [255, 255, 255]}})
    with pytest.raises(ValueError):
        lab_settings.merge(base, {'red': {'min': [0, 0, 300], 'max': [255, 255, 255]}})
    with pytest.raises(ValueError):
        lab_settings.merge(base, {'colors': ['red', 'mauve']})   # no band for mauve


def test_hsv_shaped_file_is_recognised_and_refused():
    old = {'colors': ['red'], 'red': {'lower': [0, 120, 80], 'upper': [10, 255, 255]}}
    assert lab_settings.is_hsv_shaped(old)
    assert not lab_settings.is_hsv_shaped(lab_settings.defaults())
    with pytest.raises(ValueError, match='HSV'):
        lab_settings.merge(lab_settings.defaults(), old)


def test_band_and_yaml_round_trip():
    settings = lab_settings.defaults()
    settings['min_area_px'] = 123
    lo, hi = lab_settings.band(settings, 'blue')
    assert len(lo) == len(hi) == 3
    text = lab_settings.yaml_block(settings)
    assert text.splitlines()[:2] == ['color_detect:', '  ros__parameters:']
    parsed = yaml.safe_load(text)['color_detect']['ros__parameters']
    assert parsed['min_area_px'] == 123
    assert parsed['colors'] == ['red', 'green', 'blue']
    assert parsed['blue'] == {'min': lo, 'max': hi}
