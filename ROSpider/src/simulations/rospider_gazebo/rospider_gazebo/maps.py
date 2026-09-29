"""Where the simulation launch files find worlds and save/load maps: maps go in <workspace>/maps,
next to src/ and install/. find_map and resolve_world take their directories as arguments and
need no ROS, so they are unit tested; the ament lookup is imported only where it is used."""
import os


def workspace_maps_dir(subdir=''):
    """<workspace>/maps[/subdir], found from this package's install prefix; created if missing."""
    from ament_index_python.packages import get_package_prefix

    path = get_package_prefix('rospider_gazebo')
    while os.path.basename(path) != 'install':
        parent = os.path.dirname(path)
        if parent == path:
            raise RuntimeError('rospider_gazebo is not installed inside a colcon workspace')
        path = parent
    maps = os.path.join(os.path.dirname(path), 'maps', subdir)
    os.makedirs(maps, exist_ok=True)
    return maps


def resolve_map(value, extension, subdir=''):
    """A map argument is a name kept in the workspace (room1 -> <workspace>/maps[/subdir]/room1<extension>)
    or, if it contains a '/' or starts with '~', a path to a file anywhere."""
    if os.sep in value or value.startswith('~'):
        return os.path.abspath(os.path.expanduser(value))
    return os.path.join(workspace_maps_dir(subdir), value if value.endswith(extension) else value + extension)


def find_map(value, extension, workspace_maps, cwd):
    """An existing map file for `value`, or None -- for loading only.

    map_saver_cli -f writes relative to the shell's folder, so a map saved as `maps/x` from the
    home folder lands in ~/maps, not the workspace's. A name is looked up in `workspace_maps`, then
    in <cwd>/maps and <cwd>; a path (with '/' or '~') is taken relative to `cwd`. The extension may
    be left off either way, as map_saver_cli -f takes it.
    """
    if os.sep in value or value.startswith('~'):
        candidates = [os.path.join(cwd, os.path.expanduser(value))]
    else:
        candidates = [os.path.join(workspace_maps, value),
                      os.path.join(cwd, 'maps', value), os.path.join(cwd, value)]
    for candidate in candidates:
        candidate = os.path.normpath(candidate)
        for path in (candidate, candidate + extension):
            if path.endswith(extension) and os.path.isfile(path):
                return path
    return None


def resolve_world(value, worlds_dir):
    """A world argument is a name in the package's worlds/ (slam_challenge -> .../slam_challenge.sdf)
    or, if it contains a '/' or starts with '~', a path to a file anywhere."""
    if os.sep in value or value.startswith('~'):
        return os.path.expanduser(value)
    return os.path.join(worlds_dir, value if value.endswith('.sdf') else value + '.sdf')


def error_tail(err, lines=3):
    """The last lines rtabmap-export printed, for an error message."""
    text = getattr(err, 'stderr', None) or getattr(err, 'output', None) or b''
    if isinstance(text, bytes):
        text = text.decode(errors='replace')
    return ' / '.join(line.strip() for line in text.strip().splitlines()[-lines:] if line.strip())


def localization_copy(db_path, run=None, temp_root=None):
    """A temp copy of an RTAB-Map database to localize on, with its 2D grid rebuilt.

    rtabmap in localization mode loads the 2D grid cached in the database, and writes its own
    grid back on shutdown with ghost walls from that run (camera poses lag the arm's TF); every
    later run then loads them -- measured: the answer key failed G2 with 'no path' in 3 of 4
    runs. The grid cached at the end of mapping can also be a scrap (60 x 49 cells, the start
    area, while the graph covered the room: every goal was 'outside bounds'). So the copy's
    grid is rebuilt from the graph with rtabmap-export --save_in_db (0.3 s), and the
    participant's file is never opened for writing. Copies of earlier launches (~135 MB each)
    are removed first: the exercise is edit, relaunch, again.
    """
    import glob
    import shutil
    import subprocess
    import tempfile

    if not os.path.isfile(db_path):
        raise FileNotFoundError(db_path)
    root = temp_root or tempfile.gettempdir()
    for old in glob.glob(os.path.join(root, 'vslam_localize_*')):
        shutil.rmtree(old, ignore_errors=True)
    work = tempfile.mkdtemp(prefix='vslam_localize_', dir=root)
    copy = os.path.join(work, os.path.basename(db_path))
    shutil.copyfile(db_path, copy)
    run = run or subprocess.run
    try:
        run(['rtabmap-export', '--map', '--save_in_db', copy], check=True, cwd=work,
            capture_output=True, timeout=120)
    except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as err:
        reason = error_tail(err) or str(err)
        raise RuntimeError(f'สร้างแผนที่ 2D จากไฟล์ {db_path} ไม่ได้ (rtabmap-export: {reason}) - '
                           'ไฟล์แผนที่อาจบันทึกไม่เสร็จ ลองทำแผนที่ใหม่แล้วปิด launch ให้เสร็จก่อน') from err
    return copy
