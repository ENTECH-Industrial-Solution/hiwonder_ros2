"""color_detect's settings: LAB bands in the shape of Hiwonder's lab_config.yaml.

Same three shapes as tag_settings.py / yolo_settings.py: config/color_detect.yaml
is the baseline, ~/.ros/color_detect_tuned.json overlays it key by key, and
yaml_block() renders the live values back into a paste-ready block.

One band per colour, min/max over OpenCV's 8-bit L, A, B (0-255 each) --
exactly the numbers LAB_Tool shows on the real robot. A tuned file left over
from the HSV days (lower/upper keys) is refused, not silently reinterpreted.
"""

from copy import deepcopy

#: Names a new colour may be given. Narrow on purpose: the name becomes
#: ObjectInfo.class_name, a ROS topic payload and a YAML key.
NAME_CHARS = 'abcdefghijklmnopqrstuvwxyz0123456789_'

#: What a colour added in the window starts from: wide open, so its mask
#: shows everything and the sliders narrow it onto the target.
SEED = {'min': [0, 0, 0], 'max': [255, 255, 255]}

_SCALARS = ('min_area_px', 'kernel_px')


def defaults():
    return {
        'min_area_px': 300,
        'kernel_px': 5,
        'colors': ['red', 'green', 'blue'],
        'red': {'min': [0, 150, 130], 'max': [255, 255, 255]},
        'green': {'min': [0, 0, 130], 'max': [255, 110, 255]},
        'blue': {'min': [0, 130, 0], 'max': [255, 255, 100]},
    }


def is_hsv_shaped(settings):
    """True when any colour entry still carries HSV-era lower/upper keys."""
    return any(isinstance(value, dict) and ('lower' in value or 'upper' in value)
               for value in settings.values())


def _band(value, color):
    if not isinstance(value, dict) or set(value) != {'min', 'max'}:
        raise ValueError(f'{color}: expected {{min: [L, A, B], max: [L, A, B]}}, '
                         f'got {value!r}')
    out = {}
    for key in ('min', 'max'):
        triple = [int(v) for v in value[key]]
        if len(triple) != 3 or not all(0 <= v <= 255 for v in triple):
            raise ValueError(f'{color}.{key}: need three values in 0..255, '
                             f'got {value[key]!r}')
        out[key] = triple
    return out


def merge(baseline, override):
    """Baseline with the override laid over it. Neither argument changes.

    Colours the override lists are kept and colours it leaves out are
    dropped, so a colour created in the window survives a restart and a
    deleted one stays deleted. Every listed colour must have a band.
    """
    if is_hsv_shaped(override):
        raise ValueError('HSV-shaped settings (lower/upper); the detector now '
                         'uses LAB min/max -- delete the old tuned file')
    merged = deepcopy(baseline)
    for key, value in override.items():
        if key in _SCALARS:
            merged[key] = int(value)
        elif key == 'colors':
            merged['colors'] = [str(name) for name in value]
        elif isinstance(value, dict):
            merged[key] = _band(value, key)
        else:
            raise KeyError(f'unknown setting {key!r}')
    for color in merged['colors']:
        if color not in merged:
            raise ValueError(f'no LAB band for colour {color!r}')
    return {k: v for k, v in merged.items()
            if k in _SCALARS or k == 'colors' or k in merged['colors']}


def band(settings, color):
    return list(settings[color]['min']), list(settings[color]['max'])


def yaml_block(settings):
    names = ', '.join(f"'{name}'" for name in settings['colors'])
    lines = ['color_detect:', '  ros__parameters:',
             f'    min_area_px: {int(settings["min_area_px"])}',
             f'    kernel_px: {int(settings["kernel_px"])}',
             f'    colors: [{names}]']
    for color in settings['colors']:
        lines.append(f'    {color}:')
        lines.append(f'      min: {list(settings[color]["min"])}')
        lines.append(f'      max: {list(settings[color]["max"])}')
    return '\n'.join(lines)
