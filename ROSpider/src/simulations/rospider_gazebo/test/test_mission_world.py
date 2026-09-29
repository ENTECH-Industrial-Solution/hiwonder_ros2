"""worlds/mission_challenge.sdf is tools/make_mission_world.py's output for mission_check's constants."""
import importlib.util
import math
import pathlib
import xml.etree.ElementTree as ET

from rospider_gazebo import mission_check

PKG = pathlib.Path(__file__).resolve().parents[1]


def _generator():
    spec = importlib.util.spec_from_file_location('make_mission_world',
                                                  PKG / 'tools' / 'make_mission_world.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _models(text):
    world = ET.fromstring(text[text.index('<sdf'):]).find('world')
    return {m.get('name'): m for m in world.findall('model')}


def test_shipped_world_is_the_generators_output():
    assert (PKG / 'worlds' / 'mission_challenge.sdf').read_text() == _generator().build()


def test_stations_and_pad_sit_where_the_checker_expects():
    models = _models(_generator().build())
    for tag, (x, y, yaw) in mission_check.STATIONS.items():
        pose = [float(v) for v in models[f'tag_station_{tag}'].find('pose').text.split()]
        assert pose[0] == x and pose[1] == y and math.isclose(pose[5], yaw, abs_tol=1e-4)
    for tag in mission_check.STATIONS:
        x, y, yaw = mission_check.pick_pedestal(tag)
        pose = [float(v) for v in models[f'pick_pedestal_{tag}'].find('pose').text.split()]
        assert math.isclose(pose[0], x, abs_tol=1e-4) and math.isclose(pose[1], y, abs_tol=1e-4)
    pad = [float(v) for v in models['mission_pad'].find('pose').text.split()]
    assert (pad[0], pad[1]) == mission_check.PAD


def test_the_maze_room_is_unchanged():
    maze = _models((PKG / 'worlds' / 'slam_challenge.sdf').read_text())
    mission = _models(_generator().build())
    for name in maze:
        maze[name].tail = mission[name].tail = None     # the whitespace after it, not the model
        assert ET.tostring(maze[name]) == ET.tostring(mission[name])


def test_each_block_spawns_on_its_own_pick_pedestal_past_the_centre():
    for tag, color in mission_check.BLOCK_OF_TAG.items():
        px, py, _ = mission_check.pick_pedestal(tag)
        sx, sy, _ = mission_check.STATIONS[tag]
        bx, by, bz = mission_check.block_spawn(color)
        assert math.isclose(math.hypot(bx - px, by - py), mission_check.BLOCK_BEYOND)
        assert math.hypot(bx - sx, by - sy) < math.hypot(px - sx, py - sy)   # toward the station
        assert bz == mission_check.BLOCK_Z
