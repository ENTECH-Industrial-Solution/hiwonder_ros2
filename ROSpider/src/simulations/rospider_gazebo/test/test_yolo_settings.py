import pytest
import yaml

from rospider_gazebo import yolo_settings


def test_defaults():
    settings = yolo_settings.defaults()
    assert settings == {'conf': 0.5, 'classes': []}
    # A copy: mutating it must not change the next caller's defaults.
    settings['classes'].append('red')
    assert yolo_settings.defaults()['classes'] == []


def test_merge_overlays_and_coerces():
    baseline = {'conf': 0.5, 'classes': ['red', 'green']}
    merged = yolo_settings.merge(baseline, {'conf': '0.3'})
    assert merged == {'conf': 0.3, 'classes': ['red', 'green']}
    merged = yolo_settings.merge(baseline, {'classes': ['blue']})
    assert merged == {'conf': 0.5, 'classes': ['blue']}
    # Neither argument is modified.
    assert baseline == {'conf': 0.5, 'classes': ['red', 'green']}


def test_merge_validates():
    baseline = yolo_settings.defaults()
    with pytest.raises(KeyError):
        yolo_settings.merge(baseline, {'model_path': 'x.pt'})
    with pytest.raises(ValueError):
        yolo_settings.merge(baseline, {'conf': 1.5})
    with pytest.raises(ValueError):
        yolo_settings.merge(baseline, {'classes': 'red'})


def test_yaml_block_round_trips():
    text = yolo_settings.yaml_block({'conf': 0.35, 'classes': ['red', 'blue']})
    lines = text.splitlines()
    assert lines[0] == 'yolo_detect:'
    assert lines[1] == '  ros__parameters:'
    parsed = yaml.safe_load(text)['yolo_detect']['ros__parameters']
    assert parsed == {'conf': 0.35, 'classes': ['red', 'blue']}
    # An empty class list is the "publish everything" case, and an empty
    # YAML list cannot be declared as a ROS parameter, so it is commented.
    text = yolo_settings.yaml_block({'conf': 0.5, 'classes': []})
    assert '    # classes: []' in text.splitlines()
    assert yaml.safe_load(text)['yolo_detect']['ros__parameters'] == {'conf': 0.5}
