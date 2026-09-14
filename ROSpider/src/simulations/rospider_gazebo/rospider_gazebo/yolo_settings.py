"""The two settings yolo_detect's tuner changes: confidence and class filter.

Same three shapes as tag_settings.py -- config/yolo.yaml is the baseline,
~/.ros/yolo_detect_tuned.json overlays it key by key, and Print YAML renders
the live values back into a paste-ready block -- but flat, because there
are only two values worth touching in a workshop: how sure the model must
be, and which of its classes the picker is allowed to see.

`classes` empty means "publish every class the model knows"; that is also
why the YAML output comments the key out in that case (an empty list cannot
be declared as a ROS 2 parameter, see config/yolo.yaml).
"""

from copy import deepcopy


def defaults():
    return {'conf': 0.5, 'classes': []}


def _conf(value):
    conf = float(value)
    if not 0.0 <= conf <= 1.0:
        raise ValueError(f'conf must be within 0..1, not {conf}')
    return conf


def _classes(value):
    if isinstance(value, str) or not hasattr(value, '__iter__'):
        raise ValueError(f'classes must be a list of names, not {value!r}')
    return [str(name) for name in value]


def merge(baseline, override):
    """Baseline with the override laid over it. Neither argument changes."""
    merged = deepcopy(baseline)
    for key, value in override.items():
        if key == 'conf':
            merged['conf'] = _conf(value)
        elif key == 'classes':
            merged['classes'] = _classes(value)
        else:
            raise KeyError(f'unknown setting {key!r}')
    return merged


def yaml_block(settings):
    lines = ['yolo_detect:', '  ros__parameters:',
             f'    conf: {float(settings["conf"])}']
    classes = list(settings['classes'])
    if classes:
        names = ', '.join(f"'{name}'" for name in classes)
        lines.append(f'    classes: [{names}]')
    else:
        lines.append('    # classes: []')
    return '\n'.join(lines)
