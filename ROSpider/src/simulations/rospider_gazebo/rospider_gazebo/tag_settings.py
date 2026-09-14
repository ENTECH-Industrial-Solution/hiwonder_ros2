"""Settings for apriltag_detect: defaults, parsing, JSON overlay, YAML text.

The node reads config/apriltag.yaml as flat dotted ROS parameters, overlays
~/.ros/apriltag_tuned.json on top, and the tuner writes that JSON back. All
three shapes meet here so the precedence rule is one function and testable
without ROS -- the same split color_detect.py has, but nested, because these
settings have sections and per-tag rows rather than a flat list of colours.

Settings dict shape (also the JSON shape):

    {'detector': {<tags.DETECTOR_DEFAULTS keys>},
     'max_reproj_error_px': float,
     'control': {<CONTROL_DEFAULTS keys>},
     'behaviors': {'tag<id>': {'action': str, 'standoff': float}}}

`behaviors_enabled` is deliberately not a setting: it is never saved, so a
robot never starts walking on its own because of a file left behind.
"""

import re
from copy import deepcopy

from rospider_gazebo import tags

ACTIONS = ('none', 'approach', 'place', 'stop')

# Per-tag defaults. standoff is the camera-to-tag range (tvec z) in metres at
# which approach/place stop.
DEFAULT_STANDOFF = 0.4

CONTROL_DEFAULTS = {
    'max_linear': 0.05,      # m/s -- the same ceiling nav2_params.yaml uses
    'max_angular': 0.3,      # rad/s
    'kp_yaw': 1.5,           # rad/s per metre of lateral offset
    'kp_dist': 0.4,          # m/s per metre of range error
    'yaw_deadband': 0.02,    # m of lateral offset counted as centred
    'dist_deadband': 0.03,   # m of range error counted as arrived
    'lost_timeout': 0.5,     # s without the tag before releasing cmd_vel
    'place_height': 0.055,   # m above the pedestal top; = drop_slots z - 0.08
    'turn_first_rad': 0.35,  # bearing beyond which the robot turns in place
}

_TAG_KEY = re.compile(r'^tag(\d+)$')


def defaults():
    return {
        'detector': dict(tags.DETECTOR_DEFAULTS),
        'max_reproj_error_px': 3.0,
        'control': dict(CONTROL_DEFAULTS),
        'behaviors': {},
    }


def tag_key(tag_id):
    tag_id = int(tag_id)
    if tag_id < 0:
        # A negative id would still format as 'tag-1', which _TAG_KEY (no
        # sign in the pattern) then rejects everywhere downstream -- catch
        # it here, at the one place that turns an id into a key, so the
        # caller gets one clear error instead of a mismatched key working
        # its way into merge().
        raise ValueError(f'tag id must be non-negative, not {tag_id}')
    return f'tag{tag_id}'


def tag_id(key):
    match = _TAG_KEY.match(str(key))
    if match is None:
        raise ValueError(f'behaviour keys look like tag<id>, not {key!r}')
    return int(match.group(1))


def _behavior(value, base=None):
    """A complete {'action', 'standoff'} from a possibly partial dict."""
    base = base or {'action': 'none', 'standoff': DEFAULT_STANDOFF}
    action = str(value.get('action', base['action']))
    if action not in ACTIONS:
        raise ValueError(f'action must be one of {ACTIONS}, not {action!r}')
    return {'action': action,
            'standoff': float(value.get('standoff', base['standoff']))}


def _set_section(settings, section, key, value):
    """Assign one leaf, coerced to the type of its default.

    Unknown keys raise: a typo in the YAML or JSON would otherwise be
    accepted and do nothing, which in a tuner looks like a broken slider.
    """
    if key not in settings[section]:
        raise KeyError(f'unknown {section} setting {key!r}')
    settings[section][key] = type(settings[section][key])(value)


def from_flat(flat):
    """Settings from dotted ROS parameter names, over the defaults.

    Names outside the four sections (stations, use_sim_time, tune, ...) are
    the node's other parameters and are ignored here.
    """
    settings = defaults()
    rows = {}
    for name, value in flat.items():
        parts = name.split('.')
        if name == 'max_reproj_error_px':
            settings['max_reproj_error_px'] = float(value)
        elif parts[0] in ('detector', 'control') and len(parts) == 2:
            _set_section(settings, parts[0], parts[1], value)
        elif parts[0] == 'behaviors' and len(parts) == 3:
            tag_id(parts[1])       # validates the key
            rows.setdefault(parts[1], {})[parts[2]] = value
    for key, row in rows.items():
        settings['behaviors'][key] = _behavior(row)
    return settings


def merge(baseline, override):
    """Baseline with a tuned-JSON overlay laid over it, key by key.

    Neither argument is modified. Behaviours the override adds are kept and
    a partial behaviour keeps the baseline's other field, so a tag
    configured in the tuner survives a restart without the YAML changing.
    """
    merged = deepcopy(baseline)
    for name, value in override.items():
        if name == 'max_reproj_error_px':
            merged['max_reproj_error_px'] = float(value)
        elif name in ('detector', 'control'):
            for key, leaf in dict(value).items():
                _set_section(merged, name, key, leaf)
        elif name == 'behaviors':
            for key, row in dict(value).items():
                tag_id(key)
                merged['behaviors'][key] = _behavior(
                    row, merged['behaviors'].get(key))
        else:
            raise KeyError(f'unknown setting {name!r}')
    return merged


def yaml_block(settings):
    """The settings as a paste-ready config/apriltag.yaml body.

    The JSON is for iterating; this is how a value that survives tuning
    gets back into the file that is actually committed.
    """
    lines = ['apriltag_detect:', '  ros__parameters:',
             f'    max_reproj_error_px: {float(settings["max_reproj_error_px"])}',
             '    detector:']
    for key, value in settings['detector'].items():
        lines.append(f'      {key}: {value}')
    lines.append('    control:')
    for key, value in settings['control'].items():
        lines.append(f'      {key}: {value}')
    lines.append('    behaviors:')
    for key in sorted(settings['behaviors'], key=tag_id):
        row = settings['behaviors'][key]
        lines.append(f'      {key}: {{action: {row["action"]}, '
                     f'standoff: {float(row["standoff"])}}}')
    return '\n'.join(lines)
