# AprilTag Tuner GUI and Per-Tag Behaviours Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `tune:=true` opens a Tk window for `apriltag_detect` that tunes the aruco detector live, assigns `none/approach/place/stop` to each tag id, runs those behaviours on the simulated robot, and saves everything to a JSON that overrides `config/apriltag.yaml`.

**Architecture:** The maths stays out of ROS: `tags.py` gains a `DetectorParameters` builder, a new `tag_settings.py` owns defaults/merge/YAML rendering, and a new `tag_behavior.py` turns "tags seen this frame" into a `(linear, angular)` twist plus a one-shot `place` event. `scripts/apriltag_detect.py` wires those to `/controller/cmd_vel`, `/pick_and_place/place` and TF, and hosts the Tk window built from a small shared `tkview.py`. Tk runs on the main thread with `rclpy.spin` on a daemon thread, exactly as `color_detect.py` does with highgui.

**Tech Stack:** Python 3.12, ROS 2 Jazzy (rclpy, tf2_ros, interfaces/SetString), OpenCV 5 aruco, Tkinter 8.6, pytest.

**Spec:** `docs/superpowers/specs/2026-09-14-apriltag-tuner-and-behaviors-design.md`

## Global Constraints

- All paths below are relative to `ROSpider/src/simulations/rospider_gazebo/` unless they start with `docs/` or `ROSpider/`.
- Tests run as plain pytest from that directory: `python3 -m pytest test/<file> -q`. Also `colcon build --packages-select rospider_gazebo --symlink-install` from `ROSpider/` must stay green.
- `color_detect.py` is not modified.
- `behaviors_enabled` is never written to the tuned JSON; every launch starts disabled.
- Twist ceilings: `max_linear` 0.05 m/s, `max_angular` 0.3 rad/s by default.
- Tuned JSON default path: `~/.ros/apriltag_tuned.json`, parameter `tuned_path`.
- Optical-frame convention for sightings: x right, y down, z forward (metres).
- Comment density and tone match the surrounding files (explain *why*, not *what*).
- Commit after every task with a `feat(sim):` / `test(sim):` / `docs(sim):` message ending in `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`.

---

### Task 1: Detector parameters in `tags.py`

**Files:**
- Modify: `rospider_gazebo/tags.py` (after `generate_tag_image`, and `detect_tags`)
- Test: `test/test_apriltag.py`

**Interfaces:**
- Produces: `tags.DETECTOR_DEFAULTS: dict`, `tags.detector_parameters(values: dict | None) -> cv2.aruco.DetectorParameters`, `tags.detect_tags(gray, params=None)`.

- [ ] **Step 1: Write the failing tests**

Append to `test/test_apriltag.py`:

```python
def test_detector_parameters_defaults_match_stock_aruco():
    # The GUI shows these as the starting point, so they must be exactly
    # what detect_tags used before parameters existed at all.
    stock = cv2.aruco.DetectorParameters()
    built = tags.detector_parameters()
    assert built.adaptiveThreshWinSizeMin == stock.adaptiveThreshWinSizeMin
    assert built.adaptiveThreshWinSizeMax == stock.adaptiveThreshWinSizeMax
    assert built.adaptiveThreshWinSizeStep == stock.adaptiveThreshWinSizeStep
    assert built.adaptiveThreshConstant == stock.adaptiveThreshConstant
    assert built.minMarkerPerimeterRate == stock.minMarkerPerimeterRate
    assert built.polygonalApproxAccuracyRate == stock.polygonalApproxAccuracyRate
    assert built.cornerRefinementMethod == cv2.aruco.CORNER_REFINE_NONE


def test_detector_parameters_maps_every_key():
    built = tags.detector_parameters({
        'adaptive_thresh_win_size_min': 5,
        'adaptive_thresh_win_size_max': 41,
        'adaptive_thresh_win_size_step': 4,
        'adaptive_thresh_constant': 9.5,
        'min_marker_perimeter_rate': 0.05,
        'polygonal_approx_accuracy_rate': 0.08,
        'corner_refinement': 'subpix',
    })
    assert built.adaptiveThreshWinSizeMin == 5
    assert built.adaptiveThreshWinSizeMax == 41
    assert built.adaptiveThreshWinSizeStep == 4
    assert built.adaptiveThreshConstant == 9.5
    assert built.minMarkerPerimeterRate == 0.05
    assert built.polygonalApproxAccuracyRate == 0.08
    assert built.cornerRefinementMethod == cv2.aruco.CORNER_REFINE_SUBPIX


def test_detector_parameters_rounds_even_windows_up():
    # aruco's adaptive threshold needs odd windows; an even slider value
    # must not silently become something other than the next odd one.
    built = tags.detector_parameters({'adaptive_thresh_win_size_min': 4,
                                      'adaptive_thresh_win_size_max': 20})
    assert built.adaptiveThreshWinSizeMin == 5
    assert built.adaptiveThreshWinSizeMax == 21


def test_detector_parameters_rejects_unknown_keys():
    import pytest
    with pytest.raises(KeyError):
        tags.detector_parameters({'adaptive_thresh_constant': 7.0,
                                  'made_up': 1})
    with pytest.raises(ValueError):
        tags.detector_parameters({'corner_refinement': 'contour'})


def test_detect_tags_accepts_explicit_parameters():
    image = tags.generate_tag_image(3)
    params = tags.detector_parameters({'corner_refinement': 'subpix'})
    assert [tag_id for tag_id, _ in tags.detect_tags(image, params)] == [3]
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `python3 -m pytest test/test_apriltag.py -q -k detector`
Expected: FAIL with `AttributeError: module 'rospider_gazebo.tags' has no attribute 'detector_parameters'`

- [ ] **Step 3: Implement**

In `rospider_gazebo/tags.py`, after `QUIET_MODULES`/`BOARD_FACE` constants add:

```python
# The aruco DetectorParameters the tuner exposes, with OpenCV's stock values.
# Names are snake_case so they can be YAML keys and ROS parameters; the
# mapping to aruco's camelCase attributes is in detector_parameters().
DETECTOR_DEFAULTS = {
    'adaptive_thresh_win_size_min': 3,
    'adaptive_thresh_win_size_max': 23,
    'adaptive_thresh_win_size_step': 10,
    'adaptive_thresh_constant': 7.0,
    'min_marker_perimeter_rate': 0.03,
    'polygonal_approx_accuracy_rate': 0.03,
    'corner_refinement': 'none',
}

_CORNER_REFINEMENT = {
    'none': cv2.aruco.CORNER_REFINE_NONE,
    'subpix': cv2.aruco.CORNER_REFINE_SUBPIX,
}


def _odd_window(value):
    """aruco's adaptive threshold wants an odd window of at least 3.

    OpenCV bumps an even value itself, silently; doing it here means the
    number the GUI shows is the number the detector uses.
    """
    value = max(3, int(value))
    return value if value % 2 else value + 1


def detector_parameters(values=None):
    """A cv2.aruco.DetectorParameters from a DETECTOR_DEFAULTS-shaped dict.

    Missing keys take the stock value; unknown keys raise, because a typo in
    the YAML would otherwise be accepted and do nothing, which in a tuner
    looks like "the slider is broken".
    """
    values = dict(DETECTOR_DEFAULTS, **(values or {}))
    unknown = set(values) - set(DETECTOR_DEFAULTS)
    if unknown:
        raise KeyError(f'unknown detector parameter(s): {sorted(unknown)}')
    refinement = str(values['corner_refinement'])
    if refinement not in _CORNER_REFINEMENT:
        raise ValueError(
            f'corner_refinement must be one of '
            f'{sorted(_CORNER_REFINEMENT)}, not {refinement!r}')

    params = cv2.aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMin = _odd_window(
        values['adaptive_thresh_win_size_min'])
    params.adaptiveThreshWinSizeMax = _odd_window(
        values['adaptive_thresh_win_size_max'])
    params.adaptiveThreshWinSizeStep = max(
        1, int(values['adaptive_thresh_win_size_step']))
    params.adaptiveThreshConstant = float(values['adaptive_thresh_constant'])
    params.minMarkerPerimeterRate = float(values['min_marker_perimeter_rate'])
    params.polygonalApproxAccuracyRate = float(
        values['polygonal_approx_accuracy_rate'])
    params.cornerRefinementMethod = _CORNER_REFINEMENT[refinement]
    return params
```

Change `detect_tags`:

```python
def detect_tags(gray, params=None):
    """[(tag_id, corners)] for every tag36h11 in a grayscale image.

    corners is (4, 2) float32 in the same order as object_points().
    `params` is a cv2.aruco.DetectorParameters, see detector_parameters();
    None means OpenCV's stock values.
    """
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(_DICT_ID),
        params if params is not None else detector_parameters())
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return []
    return [(int(tag_id), corner.reshape(4, 2).astype(np.float32))
            for tag_id, corner in zip(ids.ravel(), corners)]
```

- [ ] **Step 4: Run the whole test file**

Run: `python3 -m pytest test/test_apriltag.py -q`
Expected: all pass (the 13 existing plus 5 new).

- [ ] **Step 5: Commit**

```bash
git add rospider_gazebo/tags.py test/test_apriltag.py
git commit -m "feat(sim): expose the aruco detector parameters from tags.py

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 2: `tag_settings.py` — defaults, flat-param parsing, JSON merge, YAML rendering

**Files:**
- Create: `rospider_gazebo/tag_settings.py`
- Create: `test/test_tag_settings.py`
- Modify: `CMakeLists.txt` (register the test)

**Interfaces:**
- Consumes: `tags.DETECTOR_DEFAULTS`.
- Produces:
  - `ACTIONS = ('none', 'approach', 'place', 'stop')`, `CONTROL_DEFAULTS: dict`, `DEFAULT_STANDOFF = 0.4`
  - `defaults() -> dict` with keys `detector`, `max_reproj_error_px`, `control`, `behaviors`
  - `tag_key(tag_id: int) -> str` (`'tag0'`), `tag_id(key: str) -> int` (raises `ValueError` on anything but `tag<digits>`)
  - `from_flat(flat: dict[str, object]) -> dict` — dotted ROS parameter names → settings dict over defaults
  - `merge(baseline: dict, override: dict) -> dict` — JSON overlay, validated
  - `yaml_block(settings: dict) -> str`

- [ ] **Step 1: Write the failing tests**

`test/test_tag_settings.py`:

```python
import pytest

from rospider_gazebo import tag_settings, tags


def test_defaults_carry_every_section():
    settings = tag_settings.defaults()
    assert settings['detector'] == tags.DETECTOR_DEFAULTS
    assert settings['max_reproj_error_px'] == 3.0
    assert settings['control'] == tag_settings.CONTROL_DEFAULTS
    assert settings['behaviors'] == {}
    # A copy, not the module constant: a caller mutating it must not
    # change the next caller's defaults.
    settings['detector']['adaptive_thresh_constant'] = 99
    assert tags.DETECTOR_DEFAULTS['adaptive_thresh_constant'] == 7.0


def test_tag_key_round_trips():
    assert tag_settings.tag_key(12) == 'tag12'
    assert tag_settings.tag_id('tag12') == 12
    for bad in ('12', 'tag', 'tag-1', 'tagx', 'station0'):
        with pytest.raises(ValueError):
            tag_settings.tag_id(bad)


def test_from_flat_reads_dotted_parameter_names():
    # What automatically_declare_parameters_from_overrides makes of the
    # nested YAML: one flat parameter per leaf, dotted.
    settings = tag_settings.from_flat({
        'max_reproj_error_px': 2.5,
        'detector.adaptive_thresh_constant': 9.0,
        'detector.corner_refinement': 'subpix',
        'control.max_linear': 0.04,
        'behaviors.tag0.action': 'approach',
        'behaviors.tag0.standoff': 0.35,
        'behaviors.tag7.action': 'stop',
        # Not part of the settings: must be ignored, not rejected.
        'stations.station0': [0.9, 0.0, 3.14159],
        'use_sim_time': True,
        'tune': False,
    })
    assert settings['max_reproj_error_px'] == 2.5
    assert settings['detector']['adaptive_thresh_constant'] == 9.0
    assert settings['detector']['corner_refinement'] == 'subpix'
    assert settings['detector']['adaptive_thresh_win_size_min'] == 3
    assert settings['control']['max_linear'] == 0.04
    assert settings['control']['kp_yaw'] == tag_settings.CONTROL_DEFAULTS['kp_yaw']
    assert settings['behaviors'] == {
        'tag0': {'action': 'approach', 'standoff': 0.35},
        'tag7': {'action': 'stop', 'standoff': tag_settings.DEFAULT_STANDOFF},
    }


def test_from_flat_rejects_bad_values():
    with pytest.raises(KeyError):
        tag_settings.from_flat({'detector.nope': 1})
    with pytest.raises(ValueError):
        tag_settings.from_flat({'behaviors.tag0.action': 'dance'})
    with pytest.raises(ValueError):
        tag_settings.from_flat({'behaviors.zero.action': 'stop'})


def test_merge_overlays_key_by_key():
    baseline = tag_settings.defaults()
    baseline['behaviors']['tag0'] = {'action': 'approach', 'standoff': 0.4}
    merged = tag_settings.merge(baseline, {
        'detector': {'adaptive_thresh_win_size_max': 51},
        'control': {'kp_dist': 0.6},
        'behaviors': {'tag0': {'standoff': 0.3}, 'tag3': {'action': 'place'}},
    })
    assert merged['detector']['adaptive_thresh_win_size_max'] == 51
    assert merged['detector']['adaptive_thresh_win_size_min'] == 3
    assert merged['control']['kp_dist'] == 0.6
    assert merged['max_reproj_error_px'] == 3.0
    # A partial behaviour keeps the baseline's other field.
    assert merged['behaviors']['tag0'] == {'action': 'approach', 'standoff': 0.3}
    assert merged['behaviors']['tag3'] == {'action': 'place',
                                           'standoff': tag_settings.DEFAULT_STANDOFF}
    # And the baseline is untouched.
    assert baseline['detector']['adaptive_thresh_win_size_max'] == 23
    assert baseline['behaviors']['tag0']['standoff'] == 0.4


def test_merge_coerces_and_validates():
    baseline = tag_settings.defaults()
    merged = tag_settings.merge(baseline, {
        'detector': {'adaptive_thresh_win_size_min': '5'},
        'control': {'max_linear': '0.03'},
        'max_reproj_error_px': '4',
    })
    assert merged['detector']['adaptive_thresh_win_size_min'] == 5
    assert isinstance(merged['detector']['adaptive_thresh_win_size_min'], int)
    assert merged['control']['max_linear'] == 0.03
    assert merged['max_reproj_error_px'] == 4.0
    with pytest.raises(KeyError):
        tag_settings.merge(baseline, {'control': {'warp': 9}})
    with pytest.raises(KeyError):
        tag_settings.merge(baseline, {'behaviors_enabled': True})
    with pytest.raises(ValueError):
        tag_settings.merge(baseline, {'behaviors': {'tag0': {'action': 'x'}}})


def test_yaml_block_is_a_paste_ready_config():
    settings = tag_settings.defaults()
    settings['behaviors']['tag0'] = {'action': 'place', 'standoff': 0.3}
    text = tag_settings.yaml_block(settings)
    lines = text.splitlines()
    assert lines[0] == 'apriltag_detect:'
    assert lines[1] == '  ros__parameters:'
    assert '    max_reproj_error_px: 3.0' in lines
    assert '    detector:' in lines
    assert '      corner_refinement: none' in lines
    assert '    control:' in lines
    assert '      max_linear: 0.05' in lines
    assert '    behaviors:' in lines
    assert '      tag0: {action: place, standoff: 0.3}' in lines
    # It must parse back to the same settings.
    import yaml
    parsed = yaml.safe_load(text)['apriltag_detect']['ros__parameters']
    assert parsed['detector'] == settings['detector']
    assert parsed['control'] == settings['control']
    assert parsed['behaviors'] == settings['behaviors']
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `python3 -m pytest test/test_tag_settings.py -q`
Expected: FAIL with `ImportError: cannot import name 'tag_settings'`

- [ ] **Step 3: Implement**

`rospider_gazebo/tag_settings.py`:

```python
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
    return f'tag{int(tag_id)}'


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
```

Register the test in `CMakeLists.txt` after `test_labelling`:

```cmake
  ament_add_pytest_test(test_tag_settings test/test_tag_settings.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
```

- [ ] **Step 4: Run the tests**

Run: `python3 -m pytest test/test_tag_settings.py -q`
Expected: 7 passed.

- [ ] **Step 5: Commit**

```bash
git add rospider_gazebo/tag_settings.py test/test_tag_settings.py CMakeLists.txt
git commit -m "feat(sim): settings model for the AprilTag tuner

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 3: `tag_behavior.py` — the behaviour state machine

**Files:**
- Create: `rospider_gazebo/tag_behavior.py`
- Create: `test/test_tag_behavior.py`
- Modify: `CMakeLists.txt` (register the test)

**Interfaces:**
- Consumes: `tag_settings.CONTROL_DEFAULTS`, `tag_settings.tag_id`.
- Produces:
  - `Decision(twist: tuple[float, float] | None, place: int | None, status: str)` dataclass
  - `TagBehavior(control: dict, behaviors: dict[str, dict], enabled=False)`
  - `.configure(control=None, behaviors=None)`, `.enabled: bool` (settable), `.action(tag_id) -> str`
  - `.update(now: float, sightings: list[tuple[int, array-like tvec]]) -> Decision`

- [ ] **Step 1: Write the failing tests**

`test/test_tag_behavior.py`:

```python
from rospider_gazebo import tag_settings
from rospider_gazebo.tag_behavior import TagBehavior


def _behavior(**rows):
    """TagBehavior with default gains and the given tag<id>=action rows."""
    behaviors = {key: {'action': action, 'standoff': 0.4}
                 for key, action in rows.items()}
    return TagBehavior(dict(tag_settings.CONTROL_DEFAULTS), behaviors,
                       enabled=True)


def tvec(x, z):
    return [x, 0.0, z]


def test_disabled_publishes_nothing():
    b = _behavior(tag0='approach')
    b.enabled = False
    d = b.update(0.0, [(0, tvec(0.3, 1.0))])
    assert d.twist is None and d.place is None


def test_none_action_is_ignored():
    b = _behavior(tag0='none')
    d = b.update(0.0, [(0, tvec(0.3, 1.0))])
    assert d.twist is None
    # An unconfigured id is 'none' too.
    d = b.update(0.1, [(5, tvec(0.3, 1.0))])
    assert d.twist is None


def test_approach_turns_toward_and_drives_to_standoff():
    b = _behavior(tag0='approach')
    # Tag to the right (x > 0) and far: turn right (negative z) and go.
    d = b.update(0.0, [(0, tvec(0.10, 1.0))])
    linear, angular = d.twist
    assert linear > 0 and angular < 0
    # Gains: kp_yaw 1.5 * 0.10 = 0.15 < max 0.3; kp_dist 0.4 * 0.6 = 0.24
    # clamps at max_linear 0.05.
    assert abs(angular - (-0.15)) < 1e-9
    assert linear == 0.05
    # Tag to the left and too close: turn left and back up.
    d = b.update(0.1, [(0, tvec(-0.10, 0.3))])
    linear, angular = d.twist
    assert angular > 0 and linear < 0
    assert abs(linear - (-0.04)) < 1e-9


def test_deadbands_make_reached():
    b = _behavior(tag0='approach')
    d = b.update(0.0, [(0, tvec(0.01, 0.42))])
    assert d.twist == (0.0, 0.0)
    assert d.place is None
    assert 'reached' in d.status


def test_nearest_tag_wins_and_stop_beats_everything():
    b = _behavior(tag0='approach', tag1='approach', tag2='stop')
    d = b.update(0.0, [(0, tvec(0.0, 2.0)), (1, tvec(0.2, 1.0))])
    assert 'tag 1' in d.status
    d = b.update(0.1, [(0, tvec(0.0, 0.5)), (2, tvec(0.0, 3.0))])
    assert d.twist == (0.0, 0.0)
    assert 'tag 2' in d.status and 'stop' in d.status


def test_place_fires_once_then_holds():
    b = _behavior(tag0='place')
    d = b.update(0.0, [(0, tvec(0.0, 1.0))])
    assert d.place is None
    d = b.update(0.1, [(0, tvec(0.0, 0.4))])
    assert d.place == 0
    assert d.twist == (0.0, 0.0)
    # Still in view, even drifting out of the deadband: hold, no re-fire,
    # no driving while the arm is moving.
    d = b.update(0.2, [(0, tvec(0.1, 0.6))])
    assert d.place is None
    assert d.twist == (0.0, 0.0)
    # Out of view past the timeout resets; back in view can place again.
    d = b.update(0.3, [])
    assert d.twist == (0.0, 0.0)          # holding through the loss window
    d = b.update(1.0, [])
    assert d.twist == (0.0, 0.0)          # the one release zero
    d = b.update(1.1, [])
    assert d.twist is None
    d = b.update(1.2, [(0, tvec(0.0, 0.4))])
    assert d.place == 0


def test_lost_tag_releases_cmd_vel_after_timeout():
    b = _behavior(tag0='approach')
    b.update(0.0, [(0, tvec(0.0, 1.0))])
    d = b.update(0.2, [])
    assert d.twist == (0.0, 0.0)          # within lost_timeout: hold
    d = b.update(0.6, [])
    assert d.twist == (0.0, 0.0)          # past it: zero exactly once
    d = b.update(0.7, [])
    assert d.twist is None                # then leave cmd_vel alone
    d = b.update(0.8, [])
    assert d.twist is None


def test_disabling_mid_drive_stops_once():
    b = _behavior(tag0='approach')
    b.update(0.0, [(0, tvec(0.0, 1.0))])
    b.enabled = False
    d = b.update(0.1, [(0, tvec(0.0, 1.0))])
    assert d.twist == (0.0, 0.0)
    d = b.update(0.2, [(0, tvec(0.0, 1.0))])
    assert d.twist is None
    # Re-enabling starts from idle: the first frame tracks again.
    b.enabled = True
    d = b.update(0.3, [(0, tvec(0.0, 1.0))])
    assert d.twist[0] > 0


def test_configure_replaces_rows_live():
    b = _behavior(tag0='none')
    assert b.update(0.0, [(0, tvec(0.0, 1.0))]).twist is None
    b.configure(behaviors={'tag0': {'action': 'approach', 'standoff': 0.4}})
    assert b.update(0.1, [(0, tvec(0.0, 1.0))]).twist[0] > 0
    b.configure(control=dict(tag_settings.CONTROL_DEFAULTS, max_linear=0.02))
    assert b.update(0.2, [(0, tvec(0.0, 1.0))]).twist[0] == 0.02
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `python3 -m pytest test/test_tag_behavior.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'rospider_gazebo.tag_behavior'`

- [ ] **Step 3: Implement**

`rospider_gazebo/tag_behavior.py`:

```python
"""What the robot does about the tags it can see, without ROS.

scripts/apriltag_detect.py calls update() once per camera frame with the
tags solved in that frame and publishes whatever comes back. Keeping the
rules here means the sign conventions, the deadbands and the release
sequence are pinned by test/test_tag_behavior.py instead of by driving the
simulated robot into a wall.

Sightings are (tag_id, tvec) with tvec in the camera's optical frame: x
right, y down, z forward, metres. So "the tag is to the right" is x > 0 and
the robot must turn right, which is a NEGATIVE angular.z in ROS.

Twist output is (linear_x, angular_z); None means "publish nothing", which
is how teleop or Nav2 gets /controller/cmd_vel back once no tag is in view.
"""

from dataclasses import dataclass

from rospider_gazebo import tag_settings


@dataclass
class Decision:
    twist: tuple | None     # (linear_x, angular_z), or None to stay silent
    place: int | None       # tag id whose station ~/place must target now
    status: str             # one line for the GUI and the log


def _clamp(value, limit):
    return max(-limit, min(limit, value))


class TagBehavior:

    def __init__(self, control, behaviors, enabled=False):
        self.enabled = enabled
        self.configure(control, behaviors)
        self._reset()

    def configure(self, control=None, behaviors=None):
        """Replace the gains and/or the per-tag rows; keeps the current
        target so a slider tweak mid-approach does not restart it."""
        if control is not None:
            self.control = dict(control)
        if behaviors is not None:
            self.behaviors = {tag_settings.tag_id(key): dict(row)
                              for key, row in behaviors.items()}

    def action(self, tag_id):
        row = self.behaviors.get(int(tag_id))
        return row['action'] if row else 'none'

    def _reset(self):
        self.target = None      # tag id being acted on
        self.phase = 'idle'     # idle | tracking | reached | placed
        self.last_seen = None   # `now` of the last frame the target was in
        self.released = True    # the post-loss zero twist has been sent

    def update(self, now, sightings):
        if not self.enabled:
            # Switched off mid-drive: one zero so the robot actually stops,
            # then silence. Everything else forgets the target so switching
            # back on starts clean.
            self._reset_keeping_release()
            if not self.released:
                self.released = True
                return Decision((0.0, 0.0), None, 'behaviours disabled')
            return Decision(None, None, 'behaviours disabled')

        candidates = [(tag_id, tvec) for tag_id, tvec in sightings
                      if self.action(tag_id) != 'none']
        stops = [c for c in candidates if self.action(c[0]) == 'stop']
        pool = stops or candidates
        if not pool:
            return self._lost(now)
        tag_id, tvec = min(pool, key=lambda c: float(c[1][2]))
        x, z = float(tvec[0]), float(tvec[2])

        if tag_id != self.target:
            self.target, self.phase = tag_id, 'tracking'
        self.last_seen = now
        self.released = False
        action = self.action(tag_id)

        if action == 'stop':
            self.phase = 'reached'
            return Decision((0.0, 0.0), None, f'tag {tag_id}: stop')
        if self.phase == 'placed':
            # The arm is (or was) moving: hold still, and do not call
            # ~/place again until the tag has left and come back.
            return Decision((0.0, 0.0), None,
                            f'tag {tag_id}: placed; hold until it leaves view')

        c = self.control
        standoff = float(self.behaviors[tag_id]['standoff'])
        range_error = z - standoff
        angular = (0.0 if abs(x) < c['yaw_deadband']
                   else _clamp(-c['kp_yaw'] * x, c['max_angular']))
        linear = (0.0 if abs(range_error) < c['dist_deadband']
                  else _clamp(c['kp_dist'] * range_error, c['max_linear']))

        if angular == 0.0 and linear == 0.0:
            arrived = self.phase != 'reached'
            self.phase = 'reached'
            if action == 'place' and arrived:
                self.phase = 'placed'
                return Decision((0.0, 0.0), tag_id,
                                f'tag {tag_id}: reached; placing')
            return Decision((0.0, 0.0), None,
                            f'tag {tag_id}: reached (z {z:.2f} m)')

        self.phase = 'tracking'
        return Decision((linear, angular), None,
                        f'tag {tag_id}: {action} z {z:.2f} x {x:+.2f}')

    def _reset_keeping_release(self):
        released = self.released
        self._reset()
        self.released = released

    def _lost(self, now):
        """No actionable tag this frame.

        Hold zero through lost_timeout (a tag flickering out for a frame
        must not hand cmd_vel to someone else), then publish zero exactly
        once more and go silent.
        """
        if self.target is not None and self.last_seen is not None \
                and now - self.last_seen < self.control['lost_timeout']:
            return Decision((0.0, 0.0), None,
                            f'tag {self.target} lost; holding')
        self._reset_keeping_release()
        if not self.released:
            self.released = True
            return Decision((0.0, 0.0), None, 'tag lost; cmd_vel released')
        return Decision(None, None, 'no tag')
```

Register in `CMakeLists.txt` after `test_tag_settings`:

```cmake
  ament_add_pytest_test(test_tag_behavior test/test_tag_behavior.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
```

- [ ] **Step 4: Run the tests**

Run: `python3 -m pytest test/test_tag_behavior.py -q`
Expected: 10 passed. If `test_place_fires_once_then_holds` fails at `d = b.update(1.0, [])`, check `_lost`: at `now=1.0`, `last_seen=0.2`, `1.0 - 0.2 = 0.8 >= 0.5` so it must fall through to the release branch.

- [ ] **Step 5: Commit**

```bash
git add rospider_gazebo/tag_behavior.py test/test_tag_behavior.py CMakeLists.txt
git commit -m "feat(sim): per-tag behaviour state machine for apriltag_detect

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 4: Wire settings, tuned JSON and behaviours into `apriltag_detect.py` (no GUI yet)

**Files:**
- Modify: `scripts/apriltag_detect.py`
- Modify: `config/apriltag.yaml`
- Modify: `launch/pick_place.launch.py:97-104` (the `tag_detector` node)
- Modify: `package.xml`

**Interfaces:**
- Consumes: Task 1–3 APIs.
- Produces (used by the GUI in Task 5): on the node —
  `self._lock: threading.Lock`, `self.settings: dict` (live), `self.baseline: dict`,
  `self.apply_settings(settings) -> None` (under the lock, validated),
  `self.save_tuned() -> str` (message), `self.revert() -> None`, `self.yaml_block() -> str`,
  `self.behavior: TagBehavior`, `self.latest_frame: np.ndarray | None` (BGR overlay),
  `self.seen_ids: set[int]`, `self.status: str`, `self.place_response: str`,
  `self.tune: bool`, `self.tuned_path: Path`.

- [ ] **Step 1: Update `config/apriltag.yaml`**

Replace the `max_reproj_error_px`, `publish_tf`, `image_topic`, `draw` block (keep the comments that are still true) with:

```yaml
    # A planar square has two poses that reproject almost identically; the
    # node keeps the lower-error one. Above this threshold neither is
    # trustworthy and the detection is dropped. On a synthetic 60-degree view
    # the two errors measure 1.1e-06 px and 3.27 px, so 3.0 sits between a
    # clean solve and a genuinely ambiguous one.
    max_reproj_error_px: 3.0

    # cv2.aruco.DetectorParameters, OpenCV's stock values. Live sliders in
    # the tuner (tune:=true); the names map onto aruco's camelCase fields in
    # rospider_gazebo/tags.py. Window sizes must be odd; even ones are
    # rounded up.
    detector:
      adaptive_thresh_win_size_min: 3
      adaptive_thresh_win_size_max: 23
      adaptive_thresh_win_size_step: 10
      adaptive_thresh_constant: 7.0
      min_marker_perimeter_rate: 0.03
      polygonal_approx_accuracy_rate: 0.03
      corner_refinement: none        # none | subpix

    publish_tf: true
    image_topic: /depth_cam/rgb/image_raw

    # The overlay costs a frame copy; turn it off for a headless run.
    draw: true

    # Open the Tk tuning window. pick_place.launch.py sets this from tune:=.
    tune: false
    # The tuner's Save writes here and the node overlays it on this file at
    # startup, key by key. Delete the file to get back to exactly this YAML.
    tuned_path: ~/.ros/apriltag_tuned.json

    # ---- behaviours: what the robot does about a tag it can see ----------
    # Off unless switched on in the tuner (or here). Never saved to the
    # tuned JSON, so a file left behind cannot make the robot walk on its
    # own during some other demo.
    behaviors_enabled: false

    control:
      max_linear: 0.05        # m/s, the same ceiling nav2_params.yaml uses
      max_angular: 0.3        # rad/s
      kp_yaw: 1.5             # rad/s per metre the tag is off-centre
      kp_dist: 0.4            # m/s per metre of range error
      yaw_deadband: 0.02      # m of lateral offset counted as centred
      dist_deadband: 0.03     # m of range error counted as arrived
      lost_timeout: 0.5       # s without the tag before cmd_vel is released
      place_height: 0.055     # m above the pedestal top, = drop_slots z - 0.08

    # One row per tag id. action: none | approach | place | stop.
    # standoff is the camera-to-tag range in metres where approach/place
    # stop. approach and place need the camera level: arm_pose:=horizontal.
    # place then calls /pick_and_place/place with the station's pedestal
    # top, so ~/pick something first.
    behaviors:
      tag0: {action: none, standoff: 0.4}
```

Keep the `stations:` block and its comment exactly as they are.

- [ ] **Step 2: Rewrite `scripts/apriltag_detect.py`**

Replace the file's imports and class with the following (the module docstring keeps its first two paragraphs; add a third):

```python
#!/usr/bin/env python3
"""AprilTag detector for the simulation.

Publishes interfaces/ApriltagsInfo on ~/apriltag_info -- the topic and message
upstream's example/opencv_example/apriltag_recognition.py uses -- and
broadcasts each tag's pose as the TF frame tag_<id>. The TF pose is the useful
output; the message exists so upstream code has something familiar to read.

Two deliberate differences from upstream's node:

  * Intrinsics come from /depth_cam/rgb/camera_info. Upstream hardcodes a real
    camera's matrix, which is simply wrong for the simulated one.
  * The ApriltagInfo fields carry honest values. Upstream builds its object
    points from unit corners rather than metres, so the tvec it writes into
    `d` is in units of half a tag width, and `x, y` hold the pixel position of
    the text label it draws, not the tag centre. Here `x, y` is the tag centre
    in pixels, `w` its pixel width, and `d` the distance in millimetres.
    Nothing in this workspace reads upstream's version, so nothing breaks.

The node can also act on tags (rospider_gazebo/tag_behavior.py): approach a
tag and stop at a standoff, place the held cube on its station, or stop.
Off by default; switched on and configured live in the Tk tuner that
tune:=true opens, and saved to a JSON that overrides config/apriltag.yaml
the way color_detect's tuned HSV file does.

The maths lives in rospider_gazebo/tags.py so it can be tested without a
simulator; this file is only the ROS plumbing.
"""

import json
import os
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, Twist
from interfaces.msg import ApriltagInfo, ApriltagsInfo
from interfaces.srv import SetString
from rclpy.node import Node
from rospider_gazebo import tag_settings, tags
from rospider_gazebo.ros_image import to_image_msg
from rospider_gazebo.tag_behavior import TagBehavior
from sensor_msgs.msg import CameraInfo, Image


class AprilTagNode(Node):

    def __init__(self):
        super().__init__('apriltag_detect',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.bridge = CvBridge()

        family = str(self._param('family', tags.FAMILY))
        if family != tags.FAMILY:
            # Loudly, at startup. A wrong family otherwise runs forever and
            # detects nothing, which looks identical to a camera problem.
            raise ValueError(
                f'family {family!r} is not supported; this node only handles '
                f'{tags.FAMILY}')

        self.tag_size = float(self._param('tag_size', tags.TAG_SIZE))
        self.publish_tf = bool(self._param('publish_tf', True))
        self.draw = bool(self._param('draw', True))
        self.tune = bool(self._param('tune', False))
        self.tuned_path = Path(os.path.expanduser(
            str(self._param('tuned_path', '~/.ros/apriltag_tuned.json'))))
        image_topic = str(self._param('image_topic',
                                      '/depth_cam/rgb/image_raw'))

        # Shared with the tuner thread: it reads the last overlay frame and
        # the status, and writes settings through apply_settings().
        self._lock = threading.Lock()
        self.latest_frame = None
        self.seen_ids = set()
        self.status = 'no tag'
        self.place_response = ''

        self.baseline = tag_settings.from_flat(self._flat_params())
        self.behavior = TagBehavior(
            self.baseline['control'], self.baseline['behaviors'],
            enabled=bool(self._param('behaviors_enabled', False)))
        self.apply_settings(self.baseline)
        self._load_tuned()

        self.camera_matrix = None
        self.dist_coeffs = None

        self.tags_pub = self.create_publisher(ApriltagsInfo,
                                              '~/apriltag_info', 1)
        self.image_pub = self.create_publisher(Image, '~/image_result', 1)
        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.place_client = self.create_client(SetString,
                                               '/pick_and_place/place')
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, image_topic, self.image_callback, 1)
        self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info',
                                 self.info_callback, 1)
        self.get_logger().info(
            f'watching for {tags.FAMILY} tags of {self.tag_size} m on '
            f'{image_topic}; behaviours '
            f'{"ENABLED" if self.behavior.enabled else "disabled"}')

    def _param(self, name, default):
        """Read a parameter, falling back when the YAML does not carry it.

        Parameters are declared from overrides, so anything absent from
        config/apriltag.yaml comes back as None rather than raising. Same
        helper, same reason, as color_detect.py.
        """
        value = self.get_parameter(name).value
        return default if value is None else value

    def _flat_params(self):
        """Every declared parameter as {dotted_name: value}: the nested YAML
        arrives as one flat parameter per leaf, and an empty prefix matches
        them all."""
        return {name: param.value for name, param
                in self.get_parameters_by_prefix('').items()}

    # ------------------------------------------------------------- settings

    def apply_settings(self, settings):
        """Adopt a settings dict as the live detector and behaviour config.

        Validates by building the aruco parameters first, so a bad value
        from the GUI or the JSON is rejected before anything changes.
        """
        params = tags.detector_parameters(settings['detector'])
        with self._lock:
            self.settings = settings
            self.detector_params = params
            self.max_reproj_error = float(settings['max_reproj_error_px'])
            self.behavior.configure(settings['control'],
                                    settings['behaviors'])

    def _load_tuned(self):
        """Overlay the tuned JSON on the YAML baseline, if the file exists.

        Precedence is deliberately one-way and logged: the YAML is the
        committed truth, the JSON is a tuning override. Saying which one is
        live keeps a forgotten JSON from silently shadowing the YAML.
        """
        if not self.tuned_path.is_file():
            self.get_logger().info(
                f'settings from config/apriltag.yaml '
                f'(no tuned file at {self.tuned_path})')
            return
        try:
            with self.tuned_path.open() as handle:
                override = json.load(handle)
            self.apply_settings(tag_settings.merge(self.baseline, override))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.get_logger().error(
                f'ignoring unreadable tuned file {self.tuned_path}: {exc}; '
                'using config/apriltag.yaml')
            return
        self.get_logger().warn(
            f'settings OVERRIDDEN by {self.tuned_path} '
            '(delete it to go back to config/apriltag.yaml)')

    def save_tuned(self):
        """Write the live settings to the tuned JSON; returns a message."""
        with self._lock:
            settings = json.loads(json.dumps(self.settings))
        try:
            self.tuned_path.parent.mkdir(parents=True, exist_ok=True)
            with self.tuned_path.open('w') as handle:
                json.dump(settings, handle, indent=2)
                handle.write('\n')
        except OSError as exc:
            message = f'could not save {self.tuned_path}: {exc}'
            self.get_logger().error(message)
            return message
        message = f'saved {self.tuned_path}'
        self.get_logger().info(message)
        return message

    def revert(self):
        self.apply_settings(json.loads(json.dumps(self.baseline)))
        self.get_logger().info('reverted to the config/apriltag.yaml baseline')

    def yaml_block(self):
        with self._lock:
            return tag_settings.yaml_block(self.settings)

    # ------------------------------------------------------------ detection

    def info_callback(self, msg):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        if self.dist_coeffs.size == 0:
            self.dist_coeffs = np.zeros(5)

    def image_callback(self, msg):
        try:
            self._process_image(msg)
        except Exception:
            self.get_logger().error(
                'image_callback failed on this frame; skipping it',
                exc_info=True)

    def _process_image(self, msg):
        if self.camera_matrix is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        with self._lock:
            params = self.detector_params
            max_error = self.max_reproj_error

        info = ApriltagsInfo()
        transforms = []
        poses = {}          # tag id -> (rvec, tvec), for the behaviour
        for tag_id, corners in tags.detect_tags(gray, params):
            solved = tags.solve_tag_pose(
                corners, self.camera_matrix, self.dist_coeffs, self.tag_size)
            if solved is None:
                continue
            rvec, tvec, error = solved
            if error > max_error:
                # The two IPPE_SQUARE solutions are both poor; whichever won
                # the argmin is not trustworthy. Drop it rather than publish a
                # frame that may be the mirror pose.
                continue

            poses[tag_id] = (rvec, tvec)
            info.data.append(self._tag_info(tag_id, corners, tvec))
            transforms.append(self._transform(tag_id, rvec, tvec, msg.header))
            if self.draw:
                self._draw(frame, corners, tag_id, rvec, tvec)

        if self.publish_tf and transforms:
            self.broadcaster.sendTransform(transforms)
        # Published even when empty, so a consumer can tell "looking, saw
        # nothing" from "the node is dead".
        self.tags_pub.publish(info)
        if self.draw:
            self.image_pub.publish(to_image_msg(frame, msg.header))

        self._act(poses, msg.header)
        with self._lock:
            self.seen_ids.update(poses)
            self.latest_frame = frame

    # ----------------------------------------------------------- behaviours

    def _act(self, poses, header):
        """Run the behaviour on this frame's tags and publish what it says.

        Wall-clock time, not the image stamp: lost_timeout is about how long
        the robot holds still for a flickering detection, which is a
        real-time question, and a paused simulation must not freeze it.
        """
        sightings = [(tag_id, tvec.ravel()) for tag_id, (_, tvec)
                     in poses.items()]
        with self._lock:
            decision = self.behavior.update(time.monotonic(), sightings)
            self.status = decision.status
        if decision.twist is not None:
            twist = Twist()
            twist.linear.x, twist.angular.z = (float(v)
                                               for v in decision.twist)
            self.cmd_pub.publish(twist)
        if decision.place is not None:
            self.get_logger().info(decision.status)
            self._request_place(decision.place, *poses[decision.place],
                                header)

    def _request_place(self, tag_id, rvec, tvec, header):
        """Ask pick_and_place to put the held cube on this tag's pedestal.

        The pedestal top is known in the tag frame (tags.tag_to_pedestal_top)
        and the tag pose is known in the camera frame, so the point goes tag
        -> camera -> base_footprint, the frame ~/place reads. No retry: a
        "not reachable" answer means the standoff is too long, and that is a
        number for the user to change in the tuner.
        """
        rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
        point_cam = rotation @ tags.tag_to_pedestal_top() + tvec.ravel()
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_footprint', header.frame_id, rclpy.time.Time())
        except tf2_ros.TransformException as exc:
            self._note_place(f'no TF base_footprint <- {header.frame_id}: '
                             f'{exc}')
            return
        q = tf.transform.rotation
        t = tf.transform.translation
        rot_tf = _quaternion_matrix(q.x, q.y, q.z, q.w)
        point = rot_tf @ point_cam + np.array([t.x, t.y, t.z])
        with self._lock:
            point[2] += float(self.settings['control']['place_height'])

        if not self.place_client.service_is_ready():
            self._note_place('/pick_and_place/place is not available; is '
                             'pick_place.launch.py running?')
            return
        request = SetString.Request()
        request.data = f'{point[0]:.3f} {point[1]:.3f} {point[2]:.3f}'
        self._note_place(f'tag {tag_id}: ~/place {request.data} ...')
        future = self.place_client.call_async(request)
        future.add_done_callback(
            lambda done: self._note_place(
                f'tag {tag_id}: ~/place -> '
                + (done.result().message if done.exception() is None
                   else f'failed: {done.exception()}')))

    def _note_place(self, message):
        self.get_logger().info(message)
        with self._lock:
            self.place_response = message
```

Add the module-level helper above the class:

```python
def _quaternion_matrix(x, y, z, w):
    """3x3 rotation from an (x, y, z, w) quaternion. Same four lines as
    pick_and_place.py; not worth a dependency on tf_transformations."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
```

Keep `_tag_info`, `_transform`, `_draw` and `main()` exactly as they are for now (Task 5 changes `main`).

- [ ] **Step 3: Pass `tune` from the launch file and declare Tk**

In `launch/pick_place.launch.py`, change `tag_detector` to:

```python
    tag_detector = Node(
        package='rospider_gazebo',
        executable='apriltag_detect.py',
        name='apriltag_detect',
        output='screen',
        parameters=[apriltag_config,
                    {'use_sim_time': True,
                     'tune': ParameterValue(
                         LaunchConfiguration('tune'), value_type=bool)}],
        condition=IfCondition(LaunchConfiguration('tags')),
    )
```

and the `tune` argument description to `'open the tuning window(s): HSV trackbars in color_detect, the Tk tuner in apriltag_detect'`.

In `package.xml` after `<exec_depend>python3-yaml</exec_depend>` add `<exec_depend>python3-tk</exec_depend>`.

- [ ] **Step 4: Build and smoke-test without a simulator**

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider && source /opt/ros/jazzy/setup.bash && source install/local_setup.bash
colcon build --packages-select rospider_gazebo --symlink-install 2>&1 | tail -3
source install/local_setup.bash
timeout 5 ros2 run rospider_gazebo apriltag_detect.py --ros-args --params-file src/simulations/rospider_gazebo/config/apriltag.yaml; echo "exit $?"
```

Expected: build `Summary: 1 package finished`; the node logs `settings from config/apriltag.yaml (no tuned file ...)` and `watching for tag36h11 tags ... behaviours disabled`, then exit 124 from `timeout`. No traceback.

Then check the JSON overlay path with a deliberately bad file:

```bash
mkdir -p ~/.ros && echo '{"control": {"warp": 1}}' > ~/.ros/apriltag_tuned.json
timeout 5 ros2 run rospider_gazebo apriltag_detect.py --ros-args --params-file src/simulations/rospider_gazebo/config/apriltag.yaml 2>&1 | grep -i tuned
rm ~/.ros/apriltag_tuned.json
```

Expected: `ignoring unreadable tuned file ... 'unknown control setting 'warp''; using config/apriltag.yaml`.

Run the pytest suite too: `cd src/simulations/rospider_gazebo && python3 -m pytest test -q` → all pass.

- [ ] **Step 5: Commit**

```bash
git add scripts/apriltag_detect.py config/apriltag.yaml launch/pick_place.launch.py package.xml
git commit -m "feat(sim): apriltag_detect runs per-tag behaviours and loads a tuned JSON

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 5: `tkview.py` and the Tk tuner window

**Files:**
- Create: `rospider_gazebo/tkview.py`
- Modify: `scripts/apriltag_detect.py` (add `run_tuner`, change `main`)

**Interfaces:**
- Produces (reused by the later YOLO GUI):
  - `tkview.photo_from_bgr(frame, max_width=640) -> tk.PhotoImage`
  - `tkview.LabeledScale(parent, text, variable, from_, to, resolution, command)` — a `ttk.Frame` with a `tk.Scale` and an `ttk.Entry` on the same variable; `command()` is called (no args) after any change
  - `tkview.mainloop_until_shutdown(root)` — runs `root.mainloop()` and destroys the window when `rclpy.ok()` turns false

- [ ] **Step 1: Write `rospider_gazebo/tkview.py`**

```python
"""Small Tk pieces shared by the tuning windows.

Tk, like OpenCV's highgui, must own the main thread; the node's rclpy.spin
runs on a daemon thread and the window polls shared state with after(). The
helpers here are the three things every tuner needs: a camera frame as a
PhotoImage, a slider you can also type into, and a mainloop that ends when
ROS does.
"""

import tkinter as tk
from tkinter import ttk

import cv2
import rclpy


def photo_from_bgr(frame, max_width=640):
    """A tk.PhotoImage of a BGR frame, no PIL needed.

    Goes through PPM: Tk reads binary PPM natively, and cv2.imencode writes
    the channels in file order (RGB) from a BGR array, so no swap is needed.
    The caller must keep a reference to the returned image or Tk drops it.
    """
    height, width = frame.shape[:2]
    if width > max_width:
        frame = cv2.resize(frame, (max_width, int(height * max_width / width)))
    ok, ppm = cv2.imencode('.ppm', frame)
    if not ok:
        raise RuntimeError('cv2.imencode(.ppm) failed')
    return tk.PhotoImage(data=ppm.tobytes())


class LabeledScale(ttk.Frame):
    """A slider and an entry bound to one Tk variable.

    The slider is for exploring; the entry is for typing the exact value a
    tuning session ends on. `command` fires after either changes the value.
    """

    def __init__(self, parent, text, variable, from_, to, resolution,
                 command):
        super().__init__(parent)
        self.variable = variable
        self.command = command
        self.columnconfigure(0, weight=1)
        self.scale = tk.Scale(self, label=text, variable=variable,
                              from_=from_, to=to, resolution=resolution,
                              orient='horizontal', showvalue=False,
                              command=lambda _v: command())
        self.scale.grid(row=0, column=0, sticky='ew')
        self.entry = ttk.Entry(self, textvariable=variable, width=8)
        self.entry.grid(row=0, column=1, padx=(4, 0), sticky='s')
        self.entry.bind('<Return>', self._typed)
        self.entry.bind('<FocusOut>', self._typed)

    def _typed(self, _event=None):
        try:
            self.variable.get()
        except (tk.TclError, ValueError):
            return          # half-typed number; leave the old value live
        self.command()


def mainloop_until_shutdown(root):
    """root.mainloop(), ended early when rclpy shuts down (Ctrl-C in the
    terminal, or ros2 launch tearing the process down)."""
    def poll():
        if not rclpy.ok():
            root.destroy()
            return
        root.after(200, poll)
    root.after(200, poll)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
```

- [ ] **Step 2: Add the tuner to `scripts/apriltag_detect.py`**

Add imports at the top (after `from pathlib import Path`):

```python
import tkinter as tk
from tkinter import ttk
```

and `from rospider_gazebo import tag_settings, tags, tkview`.

Add these module constants after the imports:

```python
TUNE_TITLE = 'apriltag_detect tune'

# (key, label, from, to, resolution) for the detector sliders; corner
# refinement is a pair of radio buttons instead.
DETECTOR_SLIDERS = (
    ('adaptive_thresh_win_size_min', 'thresh win min', 3, 51, 2),
    ('adaptive_thresh_win_size_max', 'thresh win max', 3, 101, 2),
    ('adaptive_thresh_win_size_step', 'thresh win step', 1, 50, 1),
    ('adaptive_thresh_constant', 'thresh constant', 0.0, 30.0, 0.5),
    ('min_marker_perimeter_rate', 'min perimeter rate', 0.005, 0.2, 0.005),
    ('polygonal_approx_accuracy_rate', 'polygon accuracy', 0.01, 0.2, 0.005),
)
CONTROL_FIELDS = tuple(tag_settings.CONTROL_DEFAULTS)
```

Add a `run_tuner` method to the class (after `_note_place`, before `_tag_info`):

```python
    # ---------------------------------------------------------------- tuner

    def run_tuner(self):
        """Tk window. Runs on the main thread; Tk requires that.

        Detection and behaviours keep running throughout, so a slider's
        effect is visible on the overlay and on /controller/cmd_vel at once.
        Closing the window leaves the node running with the last values.
        """
        root = tk.Tk()
        root.title(TUNE_TITLE)
        root.columnconfigure(0, weight=1)

        image_label = ttk.Label(root)
        image_label.grid(row=0, column=0, rowspan=2, padx=6, pady=6,
                         sticky='n')
        side = ttk.Frame(root)
        side.grid(row=0, column=1, padx=6, pady=6, sticky='n')

        with self._lock:
            settings = json.loads(json.dumps(self.settings))
            enabled = self.behavior.enabled

        # ---- detector
        detector_box = ttk.LabelFrame(side, text='Detector')
        detector_box.grid(row=0, column=0, sticky='ew')
        detector_vars = {}
        for key, label, lo, hi, step in DETECTOR_SLIDERS:
            var = tk.DoubleVar(value=float(settings['detector'][key]))
            detector_vars[key] = var
            tkview.LabeledScale(detector_box, label, var, lo, hi, step,
                                self._tuner_apply).pack(fill='x')
        refinement = tk.StringVar(value=settings['detector']['corner_refinement'])
        detector_vars['corner_refinement'] = refinement
        row = ttk.Frame(detector_box)
        row.pack(fill='x')
        ttk.Label(row, text='corner refinement').pack(side='left')
        for choice in ('none', 'subpix'):
            ttk.Radiobutton(row, text=choice, value=choice,
                            variable=refinement,
                            command=self._tuner_apply).pack(side='left')
        reproj = tk.DoubleVar(value=float(settings['max_reproj_error_px']))
        tkview.LabeledScale(detector_box, 'max reproj error px', reproj,
                            0.5, 10.0, 0.1, self._tuner_apply).pack(fill='x')

        # ---- tags
        tags_box = ttk.LabelFrame(side, text='Tags')
        tags_box.grid(row=1, column=0, sticky='ew', pady=(6, 0))
        ttk.Label(tags_box, text='id').grid(row=0, column=0)
        ttk.Label(tags_box, text='action').grid(row=0, column=1)
        ttk.Label(tags_box, text='standoff m').grid(row=0, column=2)
        tag_rows = {}       # tag id -> (action StringVar, standoff StringVar)

        def add_row(tag_id, row_settings):
            if tag_id in tag_rows:
                return
            action = tk.StringVar(value=row_settings['action'])
            standoff = tk.StringVar(value=str(row_settings['standoff']))
            tag_rows[tag_id] = (action, standoff)
            r = len(tag_rows)
            ttk.Label(tags_box, text=str(tag_id)).grid(row=r, column=0)
            box = ttk.Combobox(tags_box, textvariable=action, width=9,
                               values=tag_settings.ACTIONS, state='readonly')
            box.grid(row=r, column=1)
            box.bind('<<ComboboxSelected>>', lambda _e: self._tuner_apply())
            entry = ttk.Entry(tags_box, textvariable=standoff, width=7)
            entry.grid(row=r, column=2)
            entry.bind('<Return>', lambda _e: self._tuner_apply())
            entry.bind('<FocusOut>', lambda _e: self._tuner_apply())

        for key in sorted(settings['behaviors'], key=tag_settings.tag_id):
            add_row(tag_settings.tag_id(key), settings['behaviors'][key])

        add_box = ttk.Frame(side)
        add_box.grid(row=2, column=0, sticky='ew')
        new_id = tk.StringVar()
        ttk.Entry(add_box, textvariable=new_id, width=6).pack(side='left')

        def add_typed():
            try:
                tag_id = int(new_id.get())
            except ValueError:
                return
            add_row(tag_id, {'action': 'none',
                             'standoff': tag_settings.DEFAULT_STANDOFF})
            new_id.set('')
            self._tuner_apply()
        ttk.Button(add_box, text='add id', command=add_typed).pack(side='left')

        # ---- control
        control_box = ttk.LabelFrame(side, text='Control')
        control_box.grid(row=3, column=0, sticky='ew', pady=(6, 0))
        control_vars = {}
        for i, key in enumerate(CONTROL_FIELDS):
            ttk.Label(control_box, text=key).grid(row=i, column=0, sticky='w')
            var = tk.StringVar(value=str(settings['control'][key]))
            control_vars[key] = var
            entry = ttk.Entry(control_box, textvariable=var, width=8)
            entry.grid(row=i, column=1)
            entry.bind('<Return>', lambda _e: self._tuner_apply())
            entry.bind('<FocusOut>', lambda _e: self._tuner_apply())
        enabled_var = tk.BooleanVar(value=enabled)

        def toggle():
            with self._lock:
                self.behavior.enabled = enabled_var.get()
            self.get_logger().info(
                f'behaviours {"ENABLED" if enabled_var.get() else "disabled"}')
        ttk.Checkbutton(control_box, text='Enable behaviors',
                        variable=enabled_var, command=toggle).grid(
            row=len(CONTROL_FIELDS), column=0, columnspan=2, sticky='w')
        status = ttk.Label(side, text='', wraplength=320, justify='left')
        status.grid(row=4, column=0, sticky='ew', pady=(6, 0))

        # ---- buttons
        buttons = ttk.Frame(side)
        buttons.grid(row=5, column=0, sticky='ew', pady=(6, 0))
        message = tk.StringVar()

        def do_save():
            message.set(self.save_tuned())

        def do_revert():
            self.revert()
            with self._lock:
                base = json.loads(json.dumps(self.settings))
            for key, var in detector_vars.items():
                var.set(base['detector'][key])
            reproj.set(base['max_reproj_error_px'])
            for key, var in control_vars.items():
                var.set(str(base['control'][key]))
            for tag_id, (action, standoff) in tag_rows.items():
                row = base['behaviors'].get(
                    tag_settings.tag_key(tag_id),
                    {'action': 'none',
                     'standoff': tag_settings.DEFAULT_STANDOFF})
                action.set(row['action'])
                standoff.set(str(row['standoff']))
            message.set('reverted to config/apriltag.yaml')

        def do_yaml():
            self.get_logger().info('current settings as YAML:\n'
                                   + self.yaml_block())
            message.set('YAML printed to the terminal')
        ttk.Button(buttons, text='Save', command=do_save).pack(side='left')
        ttk.Button(buttons, text='Revert', command=do_revert).pack(side='left')
        ttk.Button(buttons, text='Print YAML', command=do_yaml).pack(side='left')
        ttk.Label(side, textvariable=message).grid(row=6, column=0, sticky='w')

        # The widgets are the source of truth for the GUI; this reads them
        # all back into one settings dict and applies it. Any invalid entry
        # is reported in the message line and the previous settings stay.
        def read_widgets():
            new = {
                'detector': {key: var.get() for key, var
                             in detector_vars.items()},
                'max_reproj_error_px': reproj.get(),
                'control': {key: float(var.get()) for key, var
                            in control_vars.items()},
                'behaviors': {
                    tag_settings.tag_key(tag_id): {
                        'action': action.get(),
                        'standoff': float(standoff.get())}
                    for tag_id, (action, standoff) in tag_rows.items()},
            }
            for key in ('adaptive_thresh_win_size_min',
                        'adaptive_thresh_win_size_max',
                        'adaptive_thresh_win_size_step'):
                new['detector'][key] = int(round(new['detector'][key]))
            return tag_settings.merge(tag_settings.defaults(), new)
        self._tuner_read = read_widgets
        self._tuner_message = message

        def refresh():
            with self._lock:
                frame = self.latest_frame
                text = self.status
                response = self.place_response
                seen = sorted(self.seen_ids)
            if frame is not None:
                photo = tkview.photo_from_bgr(frame)
                image_label.configure(image=photo)
                image_label.image = photo      # keep it alive
            status.configure(text=f'{text}\n{response}')
            for tag_id in seen:
                add_row(tag_id, {'action': 'none',
                                 'standoff': tag_settings.DEFAULT_STANDOFF})
            root.after(50, refresh)
        refresh()

        self.get_logger().info(
            f'tuning window open: Save writes {self.tuned_path}, Revert '
            'reloads config/apriltag.yaml, Print YAML logs a paste-ready '
            'block; behaviours run only while "Enable behaviors" is ticked')
        tkview.mainloop_until_shutdown(root)
        self.get_logger().info('tuning window closed; still detecting')

    def _tuner_apply(self):
        try:
            self.apply_settings(self._tuner_read())
        except (ValueError, KeyError, tk.TclError) as exc:
            self._tuner_message.set(f'not applied: {exc}')
            return
        self._tuner_message.set('')
```

Replace `main()`:

```python
def main():
    rclpy.init()
    node = AprilTagNode()
    try:
        if node.tune:
            # Tk must own the main thread, so spin moves to a worker.
            spinner = threading.Thread(target=rclpy.spin, args=(node,),
                                       daemon=True)
            spinner.start()
            node.run_tuner()
            spinner.join()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
```

- [ ] **Step 3: Build and open the window without a simulator**

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider && source install/local_setup.bash
colcon build --packages-select rospider_gazebo --symlink-install 2>&1 | tail -1
source install/local_setup.bash
ros2 run rospider_gazebo apriltag_detect.py --ros-args --params-file src/simulations/rospider_gazebo/config/apriltag.yaml -p tune:=true
```

Expected: a window titled `apriltag_detect tune` with the Detector sliders, a Tags table with row `0` (action `none`, standoff `0.4`), the Control entries, the checkbox and the three buttons; the image area is blank (no camera). Exercise:
1. Drag `thresh constant` → no error in the terminal.
2. Type `abc` in `kp_yaw`, press Enter → message line shows `not applied: could not convert string to float: 'abc'`; fix it to `1.5` → message clears.
3. Set tag 0 to `approach`, click **Save** → message `saved /home/.../.ros/apriltag_tuned.json`; `cat ~/.ros/apriltag_tuned.json` shows `"tag0": {"action": "approach", "standoff": 0.4}` and no `behaviors_enabled` key.
4. Click **Revert** → tag 0 shows `none` again. Click **Print YAML** → the terminal shows the `apriltag_detect:` block.
5. Close the window → terminal logs `tuning window closed; still detecting`; Ctrl-C exits cleanly.
6. Restart with the same command → terminal logs `settings OVERRIDDEN by ...apriltag_tuned.json` and tag 0 opens as `approach`. Then `rm ~/.ros/apriltag_tuned.json`.

- [ ] **Step 4: Commit**

```bash
git add rospider_gazebo/tkview.py scripts/apriltag_detect.py
git commit -m "feat(sim): Tk tuner window for apriltag_detect

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

### Task 6: Verify in the simulator

**Files:** none modified (fixes, if any, go in the file they belong to with their own commit).

- [ ] **Step 1: Launch with the tuner and a level camera**

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider && source install/local_setup.bash && export need_compile=True
ros2 launch rospider_gazebo pick_place.launch.py tune:=true arm_pose:=horizontal auto_start:=false
```

Expected: Gazebo, RViz, the `color_detect tune` OpenCV window and the `apriltag_detect tune` Tk window. The Tk window shows the camera with tag 0 outlined (station at x 0.9) and its axes drawn.

- [ ] **Step 2: Detector sliders**

Push `min perimeter rate` up to 0.2 → the outline disappears (the tag is too small a fraction of the image at 0.9 m); back to 0.03 → it returns. Switch corner refinement to `subpix` → outline stays. Confirm `ros2 topic hz /apriltag_detect/apriltag_info` stays ~15 Hz throughout.

- [ ] **Step 3: approach**

Set tag 0 to `approach`, standoff `0.4`, tick **Enable behaviors**. Expected: status line reads `tag 0: approach z 0.8x x ...`, the robot turns to centre the tag and walks forward, and stops with status `tag 0: reached (z 0.4x m)`. `ros2 topic echo /controller/cmd_vel --once` during the walk shows `linear.x` ≤ 0.05.

Cover the tag by driving the tag station away with `gz service` is not needed: instead untick **Enable behaviors** → status `behaviours disabled`, robot stops.

- [ ] **Step 4: place**

In another terminal: `ros2 service call /pick_and_place/pick interfaces/srv/SetString "{data: red}"` and wait for state `CARRY` in the launch log. Set tag 0 to `place`, standoff `0.3`, tick **Enable behaviors**. Expected: the robot approaches, then status shows `tag 0: ~/place 0.xxx 0.yyy 0.zzz ...` followed by either `~/place -> placing red` and the cube ends up on the station's pedestal, or `~/place -> [...] is not reachable; ...`. In the second case lower the standoff by 0.05, untick and re-tick **Enable behaviors** (this resets the phase), and repeat until it places. Note the working standoff in the docs (Task 7) as the suggested default for `place`.

- [ ] **Step 5: Save and restart**

Click **Save**, close everything, relaunch **without** `tune:=true`. Expected log: `settings OVERRIDDEN by ~/.ros/apriltag_tuned.json` and `behaviours disabled` — the robot does not move on its own. Delete the JSON afterwards.

- [ ] **Step 6: Take a screenshot of the tuner for the docs/PR** and, if any step above needed a code change, commit it as `fix(sim): ...` before moving on.

---

### Task 7: Documentation

**Files:**
- Modify: `ROSpider/SIMULATION.md` (section 8, after "### ดูผลลัพธ์" and before "### เพิ่มสถานีใหม่")
- Modify: `ROSpider/CLAUDE.md` (the simulation bullets, after the `rtabtag`/AprilTag-related items — add one bullet under "Simulation (PC, no hardware)")
- Modify: `docs/superpowers/specs/2026-09-14-apriltag-tuner-and-behaviors-design.md` (file table: add `rospider_gazebo/tag_settings.py` and `test/test_tag_settings.py`; `placed` phase holds `(0, 0)` while the tag is in view)

- [ ] **Step 1: SIMULATION.md**

Insert after the "### ดูผลลัพธ์" subsection of section 8:

````markdown
### จูน detector และกำหนดพฤติกรรมต่อป้าย (`tune:=true`)

```bash
ros2 launch rospider_gazebo pick_place.launch.py tune:=true arm_pose:=horizontal auto_start:=false
```

เปิดหน้าต่าง `apriltag_detect tune` (Tk) ซ้ายคือภาพกล้องพร้อมกรอบและแกนของป้าย ขวามีสามกลุ่ม:

| กลุ่ม | มีอะไร |
|---|---|
| **Detector** | slider ของ `cv2.aruco.DetectorParameters` (`thresh win min/max/step`, `thresh constant`, `min perimeter rate`, `polygon accuracy`), `corner refinement` none/subpix และ `max reproj error px` — เปลี่ยนแล้วมีผลกับเฟรมถัดไปทันที |
| **Tags** | แถวละหนึ่ง id: `action` (`none` / `approach` / `place` / `stop`) และ `standoff` = ระยะกล้อง→ป้าย (เมตร) ที่จะหยุด id จาก `behaviors:` ใน YAML บวก id ที่เพิ่งเห็นจะโผล่มาเอง หรือพิมพ์เลขแล้วกด **add id** |
| **Control** | `max_linear` `max_angular` `kp_yaw` `kp_dist` `yaw_deadband` `dist_deadband` `lost_timeout` `place_height` และ checkbox **Enable behaviors** |

ปุ่มล่าง: **Save** เซฟลง `~/.ros/apriltag_tuned.json` (เปลี่ยนที่เก็บด้วยพารามิเตอร์ `tuned_path`), **Revert** กลับไปค่าใน `config/apriltag.yaml`, **Print YAML** พิมพ์ค่าปัจจุบันเป็นบล็อก YAML ลง terminal เอาไปวางในไฟล์ได้เลย กติกาเดียวกับ `color_detect`: YAML เป็นค่าตั้งต้น JSON ทับทีละ key ลบ JSON แล้วกลับเป็น YAML ล้วน

**พฤติกรรมเริ่มแบบปิดเสมอ** — checkbox `Enable behaviors` ไม่ถูกเซฟลง JSON (มีแต่ `behaviors_enabled` ใน YAML) เพื่อไม่ให้ไฟล์ที่ลืมไว้ทำให้หุ่นเดินเองตอนรัน demo อื่น

พฤติกรรม:

- `approach` — หมุนให้ป้ายอยู่กลางภาพ (P-controller จาก x ของป้าย) แล้วเดินเข้าหาจนระยะ z เท่า `standoff` (P-controller อีกตัว, clamp ที่ `max_linear`/`max_angular`) ถึงแล้วหยุดนิ่ง ต้องใช้ `arm_pose:=horizontal` ไม่งั้นกล้องก้มมองพื้นและไม่เห็นป้าย
- `place` — เหมือน `approach` แต่พอถึงจะเรียก `/pick_and_place/place` หนึ่งครั้ง โดยคำนวณจุดบนแท่นของสถานีจากท่าป้าย (`tags.tag_to_pedestal_top()` → กล้อง → `base_footprint` แล้วบวก `place_height`) **ต้อง `~/pick` ก่อน** ให้หุ่นอยู่ในสถานะ `CARRY` ถ้า service ตอบ `not reachable` แปลว่า `standoff` ยาวไป ลดลงทีละ 0.05 แล้วปิด-เปิด checkbox ใหม่ (การปิด-เปิดรีเซ็ตสถานะให้เริ่มใหม่) — จะไม่ retry ให้เอง คำตอบล่าสุดของ service โชว์ในบรรทัด status
- `stop` — ส่ง twist ศูนย์ตลอดที่เห็นป้ายนี้ ชนะทุก action อื่น (ใช้เป็นป้ายห้ามเข้า)
- เห็นหลายป้าย → เลือกป้ายที่ใกล้สุดที่ action ไม่ใช่ `none`
- ป้ายหายไป → ยืนนิ่ง `lost_timeout` วินาที (กันป้ายกะพริบหลุดเฟรมเดียว) แล้วส่ง twist ศูนย์อีกหนึ่งครั้งจากนั้น**เลิกยุ่งกับ `/controller/cmd_vel`** ให้ teleop/Nav2 ใช้ต่อได้

ตรรกะทั้งหมดอยู่ใน `rospider_gazebo/tag_behavior.py` (ไม่มี ROS) มีเทสต์ `test/test_tag_behavior.py` คุมทิศทางการหมุน, deadband, การยิง `place` ครั้งเดียว และลำดับการปล่อย `cmd_vel`
````

- [ ] **Step 2: CLAUDE.md**

Under the "Simulation (PC, no hardware)" bullets, after the pick-and-place bullet, add:

```markdown
- **AprilTag behaviours (`apriltag_detect.py`).** `tune:=true` opens a Tk window (`rospider_gazebo/tkview.py` holds the shared Tk helpers). Per-tag actions (`none/approach/place/stop`), the P-controller gains and the aruco `DetectorParameters` live in `config/apriltag.yaml`, overlaid by `~/.ros/apriltag_tuned.json` with the same YAML→JSON precedence as `color_detect`. The rules are in `rospider_gazebo/tag_behavior.py` (no ROS, tested): nearest actionable tag wins, `stop` beats all, `place` fires `/pick_and_place/place` once per sighting, and a lost tag holds zero for `lost_timeout` then releases `/controller/cmd_vel` with one more zero. `behaviors_enabled` is never saved, so every launch starts with behaviours off.
```

- [ ] **Step 3: Update the spec's file table and the `placed` wording**, then run `python3 -m pytest test -q` one last time and commit:

```bash
git add ROSpider/SIMULATION.md ROSpider/CLAUDE.md docs/superpowers/specs/2026-09-14-apriltag-tuner-and-behaviors-design.md
git commit -m "docs(sim): document the AprilTag tuner and per-tag behaviours

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

---

## Addendum after Task 6 (simulator findings)

Task 6 showed two things the original design missed (spec updated: "Tag memory", the turn-first rule, and `scene:=false`):

1. The robot "pitching" while approaching was the pick pedestal: its near edge is at x 0.13 and the skid box front at 0.12, so any forward motion from the spawn pose rams it. `gazebo.launch.py` alone walks 0.64 m in 15 s at 0.05 m/s with zero pitch.
2. The camera is on the wrist (`link4`); in `CARRY` the held cube fills the frame, so `place` can never see its tag. The node must remember tag poses in a fixed frame and use them as virtual sightings.

Task 7 (docs) runs AFTER Tasks 8 and 9 and must also describe the memory, `turn_first_rad` and `scene:=false`.

### Task 8: Tag memory, turn-in-place, and `scene:=false`

**Files:**
- Modify: `rospider_gazebo/tag_settings.py` (`CONTROL_DEFAULTS`)
- Modify: `rospider_gazebo/tag_behavior.py` (`update`)
- Modify: `test/test_tag_behavior.py`
- Modify: `scripts/apriltag_detect.py` (`__init__`, `_process_image`, `_act`, `run_tuner` refresh)
- Modify: `config/apriltag.yaml` (`control`, new `memory_frame`)
- Modify: `launch/pick_place.launch.py` (`scene` argument)

**Interfaces:**
- Consumes: everything Tasks 1–5 built.
- Produces: `tag_settings.CONTROL_DEFAULTS['turn_first_rad'] == 0.35`; node parameter `memory_frame` (str, default `'odom'`, `''` disables); node attribute `self.memory: dict[int, tuple[np.ndarray, np.ndarray]]` (rotation 3x3, translation 3 of the tag in `memory_frame`); `self.virtual_ids: set[int]` (ids whose sighting this frame came from memory, for the GUI).

- [ ] **Step 1: Failing tests for the turn-first rule**

Append to `test/test_tag_behavior.py`:

```python
def test_tag_behind_the_camera_turns_in_place():
    # A P law on x alone would BACK the robot through a station that is
    # behind it (z < 0 makes the range error negative). It must turn first.
    b = _behavior(tag0='approach')
    d = b.update(0.0, [(0, tvec(0.0, -1.0))])
    linear, angular = d.twist
    assert linear == 0.0
    assert abs(angular) == tag_settings.CONTROL_DEFAULTS['max_angular']
    assert 'turning' in d.status


def test_wide_bearing_turns_before_driving():
    b = _behavior(tag0='approach')
    # bearing atan2(0.5, 0.5) = 45 deg > turn_first_rad 0.35 (20 deg):
    # rotate right (tag is on the right, x > 0 => negative angular.z).
    d = b.update(0.0, [(0, tvec(0.5, 0.5))])
    assert d.twist == (0.0, -tag_settings.CONTROL_DEFAULTS['max_angular'])
    # Tag on the left, behind: rotate left.
    d = b.update(0.1, [(0, tvec(-0.5, -0.5))])
    assert d.twist == (0.0, tag_settings.CONTROL_DEFAULTS['max_angular'])
    # Inside the cone the P law takes over.
    d = b.update(0.2, [(0, tvec(0.1, 1.0))])
    assert d.twist[0] > 0.0
```

Run: `python3 -m pytest test/test_tag_behavior.py -q -k "turns"` → FAIL (`KeyError: 'turn_first_rad'` or a non-zero linear).

- [ ] **Step 2: Implement the rule**

`rospider_gazebo/tag_settings.py`, add to `CONTROL_DEFAULTS` after `place_height`:

```python
    'turn_first_rad': 0.35,  # bearing beyond which the robot turns in place
```

`rospider_gazebo/tag_behavior.py`: add `import math` at the top, and in `update()` replace the block from `c = self.control` down to the `linear = (...)` assignment with:

```python
        c = self.control
        standoff = float(self.behaviors[tag_id]['standoff'])
        bearing = math.atan2(x, z)
        if abs(bearing) > c['turn_first_rad']:
            # Far off-axis, or behind the camera (a remembered tag can be):
            # face it before driving. A P law on x alone would happily back
            # the robot through a station that is behind it.
            self.phase = 'tracking'
            angular = -math.copysign(c['max_angular'], bearing)
            return Decision((0.0, angular), None,
                            f'tag {tag_id}: turning toward it '
                            f'(bearing {math.degrees(bearing):+.0f} deg)')
        range_error = z - standoff
        angular = (0.0 if abs(x) < c['yaw_deadband']
                   else _clamp(-c['kp_yaw'] * x, c['max_angular']))
        linear = (0.0 if abs(range_error) < c['dist_deadband']
                  else _clamp(c['kp_dist'] * range_error, c['max_linear']))
```

Run: `python3 -m pytest test/test_tag_behavior.py test/test_tag_settings.py -q` → all pass (12 + 7).

- [ ] **Step 3: Memory in the node**

`config/apriltag.yaml`: in `control:` add `turn_first_rad: 0.35     # rad; beyond this bearing the robot turns in place first` after `place_height`, and after `behaviors_enabled: false` add:

```yaml
    # Where remembered tag poses live. The camera is on the wrist, so a
    # held cube hides the tag during CARRY; approach/place then steer by the
    # last pose seen, mapped through TF from this frame. '' disables memory.
    # odom is ground truth in the simulator; on a real robot use map.
    memory_frame: odom
```

`scripts/apriltag_detect.py`:

In `__init__`, after `self.place_response = ''`:

```python
        self.memory_frame = str(self._param('memory_frame', 'odom'))
        self.memory = {}          # tag id -> (R, t) of the tag in memory_frame
        self.virtual_ids = set()  # ids steered from memory this frame
```

Add two helpers after `_act`:

```python
    def _tf_matrix(self, target, source):
        """(R, t) of `source` expressed in `target`, latest, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(target, source,
                                                 rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        q = tf.transform.rotation
        t = tf.transform.translation
        return (labelling.rotation_matrix((q.x, q.y, q.z, q.w)),
                np.array([t.x, t.y, t.z]))

    def _remember(self, poses, header):
        """Store every tag seen this frame as a pose in memory_frame."""
        if not self.memory_frame or not poses:
            return
        mapped = self._tf_matrix(self.memory_frame, header.frame_id)
        if mapped is None:
            return
        rot_mc, t_mc = mapped
        with self._lock:
            for tag_id, (rvec, tvec) in poses.items():
                rot_ct, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
                self.memory[tag_id] = (rot_mc @ rot_ct,
                                       rot_mc @ tvec.ravel() + t_mc)

    def _recall(self, poses, header):
        """Virtual sightings: remembered approach/place tags not seen now.

        Mapped from memory_frame into the camera frame so the behaviour and
        _request_place see exactly what a live detection would give them.
        `stop` tags are never recalled: a remembered stop sign would freeze
        the robot forever.
        """
        with self._lock:
            enabled = self.behavior.enabled
            wanted = [tag_id for tag_id in self.memory
                      if tag_id not in poses
                      and self.behavior.action(tag_id) in ('approach', 'place')]
            memory = {tag_id: self.memory[tag_id] for tag_id in wanted}
        if not enabled or not memory:
            return {}
        mapped = self._tf_matrix(header.frame_id, self.memory_frame)
        if mapped is None:
            return {}
        rot_cm, t_cm = mapped
        recalled = {}
        for tag_id, (rot_mt, t_mt) in memory.items():
            rvec, _ = cv2.Rodrigues(rot_cm @ rot_mt)
            tvec = (rot_cm @ t_mt + t_cm).reshape(3, 1)
            recalled[tag_id] = (rvec, tvec)
        return recalled
```

In `_process_image`, replace `self._act(poses, msg.header)` with:

```python
        self._remember(poses, msg.header)
        recalled = self._recall(poses, msg.header)
        self._act({**recalled, **poses}, msg.header, set(recalled))
```

Change `_act`'s signature to `def _act(self, poses, header, virtual_ids=frozenset()):` and, inside the `with self._lock:` block after `self.status = decision.status`, add:

```python
            self.virtual_ids = set(virtual_ids)
            if self.behavior.target in virtual_ids:
                self.status += ' [remembered]'
```

(`self.status` is what the GUI shows; the appended marker is not part of `decision.status` used for the place log.)

In `run_tuner`'s `refresh()`, nothing changes: the status label already shows `self.status`.

- [ ] **Step 4: `scene:=false` in the launch file**

In `launch/pick_place.launch.py`, add `condition=IfCondition(LaunchConfiguration('scene')),` to the `Node(...)` inside the `spawns` list comprehension (same shape as `station_spawns` uses `tags`), and declare:

```python
        DeclareLaunchArgument(
            'scene', default_value='true',
            description='spawn the pick pedestal and cubes. false is the '
                        'tag-only demo: the spawn pose is 1 cm from the '
                        'pedestal, so approach can only walk when it is '
                        'absent'),
```

- [ ] **Step 5: Build, tests, smoke**

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider && source /opt/ros/jazzy/setup.bash && source install/local_setup.bash && export need_compile=True
colcon build --packages-select rospider_gazebo --symlink-install 2>&1 | tail -1
cd src/simulations/rospider_gazebo && python3 -m pytest test -q
```

Expected: build ok; 54 passed. Then `timeout 5 ros2 run rospider_gazebo apriltag_detect.py --ros-args --params-file config/apriltag.yaml` starts with no traceback.

- [ ] **Step 6: Commit**

```bash
git add rospider_gazebo/tag_settings.py rospider_gazebo/tag_behavior.py test/test_tag_behavior.py scripts/apriltag_detect.py config/apriltag.yaml launch/pick_place.launch.py
git commit -m "feat(sim): remember tag poses so place works with the camera hidden, and turn before driving

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>"
```

### Task 9: Live parameter changes, then re-verify approach and place in the simulator

Memory lives in the node process, so the instance that saw the tag before the pick is the one that must place. Without the GUI (a test driver cannot click it) there is no way to switch that instance's behaviours on later -- and the future combined Nav2 launch will need exactly that switch from another node. So this task first adds a parameter callback.

- [ ] **Step 0: live `ros2 param set`** — in `scripts/apriltag_detect.py`:

Add `from rcl_interfaces.msg import SetParametersResult` to the imports and `from copy import deepcopy`. In `__init__`, after the subscriptions are created, add `self.add_on_set_parameters_callback(self._on_parameters)`. Add the method after `revert`:

```python
    def _on_parameters(self, params):
        """Apply `ros2 param set` changes live.

        The tuner is one way to switch behaviours on and change a tag's
        action; this is the other, for a script or another node (the
        combined Nav2 launch will need it). Only the behaviour switch and
        the four settings sections are live; everything else is read once
        at startup. The GUI does not re-read these, so a slider touched
        afterwards re-applies whatever its widgets hold.
        """
        override = {}
        for param in params:
            name, value = param.name, param.value
            if name == 'behaviors_enabled':
                with self._lock:
                    self.behavior.enabled = bool(value)
                self.get_logger().info(
                    f'behaviours {"ENABLED" if value else "disabled"} '
                    '(parameter)')
            elif name == 'max_reproj_error_px':
                override[name] = value
            else:
                parts = name.split('.')
                if parts[0] in ('detector', 'control') and len(parts) == 2:
                    override.setdefault(parts[0], {})[parts[1]] = value
                elif parts[0] == 'behaviors' and len(parts) == 3:
                    override.setdefault('behaviors', {}).setdefault(
                        parts[1], {})[parts[2]] = value
        if override:
            try:
                with self._lock:
                    current = deepcopy(self.settings)
                self.apply_settings(tag_settings.merge(current, override))
            except (KeyError, ValueError, TypeError) as exc:
                return SetParametersResult(successful=False,
                                           reason=str(exc))
            self.get_logger().info(f'settings changed by parameter: '
                                   f'{override}')
        return SetParametersResult(successful=True)
```

Verify without a simulator: run the node (`ros2 run rospider_gazebo apriltag_detect.py --ros-args --params-file config/apriltag.yaml -p use_sim_time:=false`), then in another shell `ros2 param set /apriltag_detect behaviors_enabled true` → log `behaviours ENABLED (parameter)`; `ros2 param set /apriltag_detect behaviors.tag0.action place` → log `settings changed by parameter: {'behaviors': {'tag0': {'action': 'place'}}}`; `ros2 param set /apriltag_detect behaviors.tag0.action dance` → the command reports the failure reason. Commit as `feat(sim): switch tag behaviours and rows live with ros2 param set`.

Same environment rules as Task 6 (background launches, logs under the workspace dir, no XTest, kill everything at the end, delete `~/.ros/apriltag_tuned.json`). For every item below launch with `tags:=false` and spawn the station yourself, so there is exactly ONE `apriltag_detect` in the graph (the one you run with overrides).

- [ ] **C′ approach, tag-only scene**: `ros2 launch rospider_gazebo pick_place.launch.py scene:=false auto_start:=false arm_pose:=horizontal tags:=false`, spawn `tag_station_0` at `(0.9, 0, yaw 3.14159)`, run the detector with `-p behaviors_enabled:=true -p behaviors.tag0.action:=approach -p behaviors.tag0.standoff:=0.4 -p tune:=true`. Expected: the robot walks straight to the station and stops with `apriltag_info.d` ≈ 400 ± 30 mm and `odom` pitch ≈ 0; `cmd_vel` then (0,0). Record the final `d` and `odom.pose.position.x`.
- [ ] **Memory**: with the robot stopped at the station, cover the tag by spawning a box in front of it (`gz service -s /world/rospider_room/create ...` with a 0.3×0.3×0.5 box at (0.55, 0, 0.25)) or simply rotate the robot away with `ros2 topic pub --once /controller/cmd_vel ... angular.z 0.3` for 4 s so the tag leaves the frame. Expected: `apriltag_info.data` empty, but the detector keeps publishing `cmd_vel` and turns back to face the station (status in the log or GUI shows `[remembered]`), then holds. Record it.
- [ ] **E′ place, full scene**: relaunch with the scene: `pick_place.launch.py auto_start:=false arm_pose:=horizontal tags:=false`, spawn the station, and start the detector BEFORE the pick with behaviours off: `-p tune:=true -p behaviors_enabled:=false -p behaviors.tag0.action:=place -p behaviors.tag0.standoff:=0.3`. Sequence: confirm it sees tag 0 (`apriltag_info` has id 0; memory now holds it) → `ros2 service call /pick_and_place/pick interfaces/srv/SetString "{data: red}"` and wait for the launch log to show `CARRY` → drive the robot around the pedestal with `ros2 topic pub -r 10 /controller/cmd_vel ...` under `timeout` (e.g. back up `linear.x -0.05` for 6 s, strafe `linear.y 0.05` for 8 s, forward `linear.x 0.05` for 10 s) → `ros2 param set /apriltag_detect behaviors_enabled true`. Expected: it turns toward the remembered station (status `[remembered]`), drives to 0.3 m, logs `tag 0: ~/place x y z ...` then `~/place -> placing red`, and `gz model -m pick_cube_red -p` ends near the station pedestal (x ≈ 0.9, z ≈ 0.105). If `not reachable`, `ros2 param set /apriltag_detect behaviors_enabled false`, then `ros2 param set /apriltag_detect behaviors.tag0.standoff 0.25` (then 0.2), re-enable, and record the value that worked.
- [ ] Fix anything broken in its owning file with a `fix(sim):` commit; write `task-9-report.md` with commands, evidence and pass/fail per item.

### Task 10: Steer in a level frame at `base_footprint`, then re-run E′

Task 9 found that `place` declares "reached" almost immediately in `CARRY`: the wrist camera then points down and sideways (RPY about [-160°, 0°, -92°] vs `base_footprint`), so camera x/z have nothing to do with the robot's heading. Spec section "Tag memory" now requires every sighting to be expressed in a level frame at `base_footprint`.

**Files:**
- Modify: `scripts/apriltag_detect.py` (`_act`)
- Modify: `config/apriltag.yaml` (the `behaviors` comment: standoff is measured from `base_footprint`)

- [ ] **Step 1: map sightings before steering**

In `_act`, replace the `sightings = [...]` construction with:

```python
        # Steer in a level frame at the robot's base, not in the camera's.
        # The camera is on the wrist: in CARRY it points down and sideways,
        # and a controller working in camera x/z then "reaches" the standoff
        # somewhere meaningless. (-Y, -Z, X) of the base_footprint position
        # is the optical convention TagBehavior speaks (x right, z forward),
        # measured from the base along the heading.
        mapped = self._tf_matrix('base_footprint', header.frame_id)
        if mapped is None:
            if poses:
                self.get_logger().warn(
                    f'no TF base_footprint <- {header.frame_id}; not '
                    'steering this frame', throttle_duration_sec=5.0)
            return
        rot_bc, t_bc = mapped
        sightings = []
        for tag_id, (_, tvec) in poses.items():
            base = rot_bc @ tvec.ravel() + t_bc
            sightings.append((tag_id, np.array([-base[1], -base[2], base[0]])))
```

Everything after it (`self.behavior.update`, publishing, `_request_place` with the camera-frame pose) stays as it is.

In `config/apriltag.yaml`, change the `behaviors` comment's `standoff` sentence to: `standoff is the range from base_footprint to the tag, along the robot's heading, in metres, where approach/place stop -- so it does not depend on the arm pose.` Keep `tag0: {action: none, standoff: 0.4}`.

- [ ] **Step 2: build, pytest (54 passed), 5 s `ros2 run` smoke; commit** `fix(sim): steer tag behaviours from base_footprint, not from the wrist camera`.

- [ ] **Step 3: re-run Task 9's C′ and E′** exactly as written there (C′ expected numbers now refer to the base: the robot stops when the tag is 0.4 m ahead of `base_footprint`, so `apriltag_info.d` will read somewhat less than 400 mm since the camera sits ahead of the base -- record both). For E′ start at standoff 0.3; if `not reachable`, try 0.28 and 0.26 (the station pedestal centre is 0.12 m in front of the tag and the arm reaches about 0.16 m ahead of the base). Record the working value and put it in `config/apriltag.yaml`'s `behaviors` comment as the suggested value for `place`. Write `task-10-report.md`.

### Task 11: Simple view by default, Advanced behind a toggle

The window is for a 10-minute workshop segment: "the robot knows which tag it sees, how far it is, and what to do about it". The detector sliders and the nine control gains stay, but hidden.

**Files:**
- Modify: `scripts/apriltag_detect.py` (`run_tuner` layout only; no behaviour change)

- [ ] **Step 1: restructure the side column**

Inside `run_tuner`, keep every widget and callback as it is but change WHERE they are placed:

1. Create `advanced = ttk.Frame(side)` right after `side` is created. Build `detector_box` with parent `advanced` (not `side`) and `control_box` with parent `advanced`. Grid them inside `advanced` at rows 0 and 1 (`sticky='ew'`, `pady=(6, 0)`).
2. Move the `Enable behaviors` checkbox out of `control_box`: its parent becomes `side` and it is gridded at `side` row 2 (`sticky='w'`, `pady=(6, 0)`), directly under the add-id row. The status label goes to row 3, the buttons to row 4, the message label to row 5.
3. `tags_box` goes to `side` row 0, `add_box` to row 1.
4. Add the toggle after the message label:

```python
        # Workshop view: tags, the switch, and the buttons. The detector
        # sliders and the controller gains are one click away, not in the
        # way -- a ten-minute demo needs the first three, and a puzzled
        # afternoon needs the rest.
        advanced.grid(row=7, column=0, sticky='ew')
        advanced.grid_remove()
        advanced_shown = tk.BooleanVar(value=False)

        def toggle_advanced():
            if advanced_shown.get():
                advanced.grid_remove()
                advanced_button.configure(text='Advanced ▸')
            else:
                advanced.grid()
                advanced_button.configure(text='Advanced ▾')
            advanced_shown.set(not advanced_shown.get())
        advanced_button = ttk.Button(side, text='Advanced ▸',
                                     command=toggle_advanced)
        advanced_button.grid(row=6, column=0, sticky='w', pady=(6, 0))
```

`do_revert` and `read_widgets` keep working unchanged: they address the variables, not the frames.

- [ ] **Step 2: verify without a simulator**

Build, then `ros2 run rospider_gazebo apriltag_detect.py --ros-args --params-file config/apriltag.yaml -p tune:=true`. Expected on open: image area, Tags table with row 0, add-id, `Enable behaviors`, status line, Save/Revert/Print YAML, and a single `Advanced ▸` button — no sliders. Clicking it reveals Detector and Control below; clicking again hides them. Save still writes the same JSON; `python3 -m pytest test -q` still 54 passed.

- [ ] **Step 3: commit** `feat(sim): AprilTag tuner opens in a simple view, sliders under Advanced`.
