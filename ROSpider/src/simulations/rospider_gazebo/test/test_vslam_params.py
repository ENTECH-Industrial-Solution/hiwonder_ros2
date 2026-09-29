import pathlib

import pytest
import yaml
from rospider_gazebo import vslam_params
from rospider_gazebo.challenge_params import ChallengeConfigError

PKG = pathlib.Path(__file__).resolve().parents[1]
VSLAM = yaml.safe_load((PKG / 'config' / 'vslam.yaml').read_text())


def _rtab(merged):
    return merged['rtabmap']['ros__parameters']


def test_camera_view_becomes_arm_pose():
    assert vslam_params.split_camera_view({'camera_view': 'floor', 'a': 1}, 'f') == ('init', {'a': 1})
    assert vslam_params.split_camera_view({'camera_view': 'ahead'}, 'f') == ('horizontal', {})
    assert vslam_params.split_camera_view({}, 'f') == ('horizontal', {})


@pytest.mark.parametrize('value', ['down', '', None, True, 1])
def test_bad_camera_view_is_refused(value):
    with pytest.raises(ChallengeConfigError, match='camera_view'):
        vslam_params.split_camera_view({'camera_view': value}, 'f')


def test_rtabmap_keys_land_as_strings():
    merged = vslam_params.apply_rtabmap(VSLAM, {'Grid/RangeMax': 0.5, 'Vis/MinInliers': 5,
                                                'Grid/CellSize': '0.05'}, 'f')
    assert _rtab(merged)['Grid/RangeMax'] == '0.5'
    assert _rtab(merged)['Vis/MinInliers'] == '5'
    assert _rtab(merged)['Grid/CellSize'] == '0.05'
    assert _rtab(VSLAM)['Grid/RangeMax'] == '5.0'            # the base is not modified


@pytest.mark.parametrize('overrides, word', [
    ({'Grid/RangMax': '5.0'}, 'Grid/RangMax'),       # typo
    ({'Grid/RangeMax': None}, 'Grid/RangeMax'),      # blank
    ({'Grid/RangeMax': ''}, 'Grid/RangeMax'),
    ({'Grid/RangeMax': True}, 'Grid/RangeMax'),
    ({'map_always_update': 'yes'}, 'map_always_update'),   # non-string key keeps its type
])
def test_bad_values_are_refused(overrides, word):
    with pytest.raises(ChallengeConfigError, match=word):
        vslam_params.apply_rtabmap(VSLAM, overrides, 'f')


@pytest.mark.parametrize('name, pose', [('vslam_challenge.yaml', 'init'),
                                        ('vslam_challenge_solved.yaml', 'horizontal')])
def test_shipped_mapping_files_merge(name, pose):
    path, arm_pose = vslam_params.mapping_params_file(str(PKG / 'config' / 'vslam.yaml'),
                                                      str(PKG / 'config' / name))
    assert arm_pose == pose
    with open(path) as handle:
        assert 'camera_view' not in yaml.safe_load(handle)['rtabmap']['ros__parameters']


@pytest.mark.parametrize('name, pose', [('vslam_nav_challenge.yaml', 'horizontal'),
                                        ('vslam_nav_challenge_solved.yaml', 'horizontal')])
def test_shipped_nav_files_merge(name, pose):
    path, arm_pose = vslam_params.nav_params_file(str(PKG / 'config' / 'vslam_nav2_params.yaml'),
                                                  str(PKG / 'config' / name))
    with open(path) as handle:
        merged = yaml.safe_load(handle)
    assert arm_pose == pose
    plugin = merged['controller_server']['ros__parameters']['FollowPath']['plugin']
    assert 'RegulatedPurePursuit' in plugin


def test_require_map_missing(tmp_path):
    with pytest.raises(ChallengeConfigError, match='vslam_challenge'):
        vslam_params.require_map('nothere', str(tmp_path), str(tmp_path))


def test_require_map_found(tmp_path):
    (tmp_path / 'mine.db').write_bytes(b'')
    assert vslam_params.require_map('mine', str(tmp_path), '/') == str(tmp_path / 'mine.db')
