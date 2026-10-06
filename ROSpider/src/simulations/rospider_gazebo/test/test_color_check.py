"""color_check against a frame recorded from worlds/color_challenge.sdf (test/data)."""
import copy
import json
import pathlib

import cv2
import numpy as np
import pytest
import yaml
from rospider_gazebo import color_check, lab_settings

PKG = pathlib.Path(__file__).resolve().parents[1]
FRAME = cv2.imread(str(PKG / 'test' / 'data' / 'color_challenge.png'))
ANSWER = lab_settings.defaults()
FORBIDDEN = ('min', 'max', 'kernel', 'area', 'red.', 'green.', 'blue.')


def _bands(red=None, green=None, blue=None):
    settings = copy.deepcopy(ANSWER)
    for name, band in (('red', red), ('green', green), ('blue', blue)):
        if band:
            settings[name] = band
    return settings


START = _bands(red={'min': [0, 135, 130], 'max': [255, 255, 255]},
               green={'min': [0, 0, 130], 'max': [255, 60, 255]},
               blue={'min': [80, 130, 0], 'max': [255, 255, 100]})


def _levels(settings, frame=FRAME):
    return color_check.evaluate(color_check.find_blobs(frame, settings))


def _statuses(levels):
    return [level.status for level in levels]


def test_answer_key_passes_all():
    levels = _levels(ANSWER)
    assert _statuses(levels) == ['pass'] * 3
    assert 'ผ่านครบทุกด่าน' in color_check.format_report(levels)


def test_starting_file_misses_green_level_one():
    levels = _levels(START)
    assert _statuses(levels) == ['fail', 'skip', 'skip']
    assert levels[0].numbers['missing'] == ['green']


def test_too_wide_red_catches_the_orange_decoy_level_two():
    settings = copy.deepcopy(START)
    settings['green'] = ANSWER['green']
    levels = _levels(settings)
    assert _statuses(levels) == ['pass', 'fail', 'skip']
    assert [color for color, _ in levels[1].numbers['extra']] == ['red']


def test_narrow_lightness_misses_the_blue_cube_in_shadow_level_three():
    settings = copy.deepcopy(START)
    settings['green'], settings['red'] = ANSWER['green'], ANSWER['red']
    levels = _levels(settings)
    assert _statuses(levels) == ['pass', 'pass', 'fail']
    assert 'เงา' in color_check.format_report(levels)


def test_black_frame_is_reported():
    black = np.zeros_like(FRAME)
    assert color_check.frame_problem(black)
    assert color_check.frame_problem(FRAME) is None


def test_hints_name_no_parameter():
    green_fixed = copy.deepcopy(START)
    green_fixed['green'] = ANSWER['green']
    both = copy.deepcopy(green_fixed)
    both['red'] = ANSWER['red']
    for settings in (START, green_fixed, both):
        report = color_check.format_report(_levels(settings))
        assert not any(word in report for word in FORBIDDEN), report


def test_settings_file_then_tuned_json(tmp_path):
    params = {'color_detect': {'ros__parameters': {
        'min_area_px': 300, 'kernel_px': 5, 'colors': ['red', 'green', 'blue'],
        'red': START['red'], 'green': START['green'], 'blue': START['blue']}}}
    path = tmp_path / 'challenge.yaml'
    path.write_text(yaml.safe_dump(params))
    assert color_check.load_settings(str(path), str(tmp_path / 'none.json'))['green'] == START['green']
    tuned = tmp_path / 'tuned.json'
    tuned.write_text(json.dumps({**START, 'green': ANSWER['green']}))
    assert color_check.load_settings(str(path), str(tuned))['green'] == ANSWER['green']


@pytest.mark.parametrize('name, expected', [('color_challenge.yaml', START),
                                            ('color_challenge_solved.yaml', ANSWER)])
def test_shipped_files_hold_the_measured_bands(name, expected, tmp_path):
    if not (PKG / 'config' / name).exists():
        pytest.skip('answer key not in the repo (instructors keep it)')
    settings = color_check.load_settings(str(PKG / 'config' / name), str(tmp_path / 'none.json'))
    for color in ('red', 'green', 'blue'):
        assert settings[color] == expected[color], color


# ------------------------------------------------------------ review fixes

def test_a_band_that_whitens_the_frame_does_not_pass():
    # reviewer's case: "everything brighter than L 100" -- one green blob over 82% of the frame
    # whose centroid happened to fall in the green box
    levels = _levels(_bands(green={'min': [100, 0, 0], 'max': [255, 255, 255]}))
    assert levels[0].status == 'fail'
    report = color_check.format_report(levels)
    assert 'กว้าง' in report and 'แคบ' not in report


def test_a_fully_open_band_is_called_too_wide_not_missing():
    levels = _levels(_bands(red={'min': [0, 0, 0], 'max': [255, 255, 255]}))
    assert levels[0].status == 'fail' and levels[0].numbers['wide'] == ['red']
    assert 'แคบ' not in color_check.format_report(levels)


def test_a_colour_added_in_the_window_is_ignored():
    settings = copy.deepcopy(ANSWER)
    settings['colors'] = ['red', 'green', 'blue', 'orange']
    settings['orange'] = {'min': [0, 0, 0], 'max': [255, 255, 255]}
    levels = _levels(settings)
    assert _statuses(levels) == ['pass'] * 3
    assert 'orange' in color_check.format_report(levels, color_check.ignored_colors(settings))


def test_a_colour_removed_from_the_list_is_named_as_missing_from_it():
    settings = copy.deepcopy(ANSWER)
    settings['colors'] = ['red', 'blue']
    del settings['green']
    levels = _levels(settings)
    assert levels[0].status == 'fail' and levels[0].numbers['absent'] == ['green']
    assert 'Color list' in color_check.format_report(levels)


def test_a_frame_from_elsewhere_is_refused_before_scoring():
    assert color_check.scene_problem(FRAME) is None
    moved = np.roll(FRAME, 40, axis=1)                    # the robot turned ~5 degrees
    assert color_check.scene_problem(moved)
    assert color_check.scene_problem(cv2.resize(FRAME, (320, 240)))


def test_live_settings_from_the_window_are_parsed_like_the_file():
    text = json.dumps({**ANSWER, 'green': START['green']})
    assert color_check.settings_from_json(text)['green'] == START['green']
