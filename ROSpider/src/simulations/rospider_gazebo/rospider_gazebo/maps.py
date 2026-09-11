"""Where the simulation launch files save and load maps: <workspace>/maps, next to src/ and install/."""
import os

from ament_index_python.packages import get_package_prefix


def workspace_maps_dir(subdir=''):
    """<workspace>/maps[/subdir], found from this package's install prefix; created if missing."""
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
