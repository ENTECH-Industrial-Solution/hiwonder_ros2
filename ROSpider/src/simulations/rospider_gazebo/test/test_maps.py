from rospider_gazebo import maps


def _touch(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('x')
    return str(path)


def test_world_by_name(tmp_path):
    worlds = str(tmp_path)
    assert maps.resolve_world('slam_challenge', worlds) == str(tmp_path / 'slam_challenge.sdf')
    assert maps.resolve_world('slam_challenge.sdf', worlds) == str(tmp_path / 'slam_challenge.sdf')


def test_world_path_is_kept(tmp_path):
    assert maps.resolve_world('/abs/room.sdf', str(tmp_path)) == '/abs/room.sdf'
    assert maps.resolve_world('~/room.sdf', str(tmp_path)).endswith('/room.sdf')
    assert not maps.resolve_world('~/room.sdf', str(tmp_path)).startswith('~')


def test_map_found_wherever_map_saver_was_run(tmp_path):
    """map_saver_cli -f maps/x writes relative to the shell's folder."""
    workspace, home = tmp_path / 'ws' / 'maps', tmp_path / 'home'
    workspace.mkdir(parents=True)
    saved = _touch(home / 'maps' / 'jirapat.yaml')
    assert maps.find_map('jirapat', '.yaml', str(workspace), str(home)) == saved
    # The workspace copy wins when both exist.
    in_workspace = _touch(workspace / 'jirapat.yaml')
    assert maps.find_map('jirapat', '.yaml', str(workspace), str(home)) == in_workspace
    assert maps.find_map('nope', '.yaml', str(workspace), str(home)) is None


def test_map_path_with_or_without_extension(tmp_path):
    saved = _touch(tmp_path / 'maps' / 'room1.yaml')
    for value in ('maps/room1', 'maps/room1.yaml', str(tmp_path / 'maps' / 'room1')):
        assert maps.find_map(value, '.yaml', '/nowhere', str(tmp_path)) == saved
