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
