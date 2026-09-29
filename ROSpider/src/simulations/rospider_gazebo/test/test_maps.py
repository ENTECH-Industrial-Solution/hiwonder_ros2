import os
import pytest
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


def _no_export(*args, **kwargs):
    return None


def test_localization_copy_leaves_the_map_untouched(tmp_path):
    db = tmp_path / 'mine.db'
    db.write_bytes(b'original')
    copy = maps.localization_copy(str(db), run=_no_export)
    assert copy != str(db)
    with open(copy, 'rb') as handle:
        assert handle.read() == b'original'
    with open(copy, 'wb') as handle:
        handle.write(b'changed by rtabmap')
    assert db.read_bytes() == b'original'


def test_localization_copy_of_a_missing_map_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        maps.localization_copy(str(tmp_path / 'none.db'), run=_no_export)


def test_localization_copy_rebuilds_the_cached_grid(tmp_path):
    # measured: the 2D grid rtabmap caches in the database at the end of mapping was 60x49
    # cells (the start area) while the graph covered the room; localization loads that cache
    db = tmp_path / 'mine.db'
    db.write_bytes(b'x')
    calls = []
    copy = maps.localization_copy(str(db), run=lambda cmd, **kw: calls.append((cmd, kw)))
    assert calls[0][0] == ['rtabmap-export', '--map', '--save_in_db', copy]
    assert calls[0][1]['check'] is True


def test_localization_copy_reports_a_failed_rebuild(tmp_path):
    import subprocess
    db = tmp_path / 'mine.db'
    db.write_bytes(b'x')

    def fail(cmd, **kw):
        raise subprocess.CalledProcessError(1, cmd)
    with pytest.raises(RuntimeError, match='rtabmap-export'):
        maps.localization_copy(str(db), run=fail)


def test_failed_rebuild_message_is_thai_and_keeps_the_reason(tmp_path):
    import subprocess
    db = tmp_path / 'mine.db'
    db.write_bytes(b'x')

    def fail(cmd, **kw):
        raise subprocess.CalledProcessError(1, cmd, stderr=b'line1\nDatabase is corrupted\n')
    with pytest.raises(RuntimeError) as err:
        maps.localization_copy(str(db), run=fail)
    assert 'สร้างแผนที่' in str(err.value) and 'Database is corrupted' in str(err.value)


def test_older_localization_copies_are_removed(tmp_path):
    db = tmp_path / 'mine.db'
    db.write_bytes(b'x')
    first = maps.localization_copy(str(db), run=_no_export, temp_root=str(tmp_path))
    second = maps.localization_copy(str(db), run=_no_export, temp_root=str(tmp_path))
    assert os.path.exists(second) and not os.path.exists(first)
    assert db.exists()
