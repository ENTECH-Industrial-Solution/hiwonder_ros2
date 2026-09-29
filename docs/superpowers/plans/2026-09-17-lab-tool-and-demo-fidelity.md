# LAB Colour Tool and Demo-Window Fidelity Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `color_detect.py` threshold in LAB with a `LAB_Tool`-style Tk window, and make every ported demo window draw exactly what its Hiwonder original draws.

**Architecture:** `rospider_gazebo/lab_settings.py` (pure Python, tested) owns the LAB config shape and precedence; `scripts/color_detect.py` uses it for detection and its Tk window. `rospider_gazebo/vision_demo.py` loses `banner()` and draws FPS only on opt-in; each demo script's `process()` is reduced to the upstream draw calls, with simulation-only states logged instead of drawn.

**Tech Stack:** ROS 2 Jazzy, Python 3.12, OpenCV, tkinter, pytest (run from the package dir with `python3 -m pytest test/ -q -p no:cacheprovider`).

**Spec:** `docs/superpowers/specs/2026-09-17-lab-tool-and-demo-fidelity-design.md`

## Global Constraints

- All paths below are relative to `ROSpider/src/simulations/rospider_gazebo/` unless they start with `ROSpider/` or `docs/`.
- Before any ROS command: `cd ~/entech_hiwonder_ros2_ws/ROSpider && source /opt/ros/jazzy/setup.bash && source install/local_setup.bash && export need_compile=True`. Rebuild with `colcon build --packages-select rospider_gazebo --symlink-install` after adding a file (symlink install still needs new files registered).
- Upstream draws on RGB frames and converts before `imshow`; the sim draws on BGR. Copy upstream's colour tuples with the channels reversed: upstream `(255, 0, 0)` on RGB = red = sim `(0, 0, 255)`. Upstream files that already work in BGR (`kcf.py`, `hand_gesture.py`, `finger_trajectory.py`, `face_track.py`, `pose_control.py`, `color_detect_node.py`, the two `rgbd_example` depth demos) are copied verbatim.
- Nothing a node publishes changes: topics, message types and field meanings stay as they are.
- Simulation-only states (waiting for a topic, calibration, tracker lost, no selection) are reported with `self.get_logger().info(...)`, never drawn.
- Commit after each task. Commit messages: `feat(sim): ...` / `refactor(sim): ...` / `docs(sim): ...`, ending with `Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>`. The working tree carries uncommitted mini-game WIP in other files; only `git add` the files each task names.

---

### Task 1: `lab_settings.py`

**Files:**
- Create: `rospider_gazebo/lab_settings.py`
- Test: `test/test_lab_settings.py`
- Modify: `CMakeLists.txt` (add the test after `test_yolo_settings`)

**Interfaces:**
- Produces:
  - `defaults() -> dict`: `{'min_area_px': 300, 'kernel_px': 5, 'colors': ['red','green','blue'], 'red': {'min': [...], 'max': [...]}, ...}`
  - `merge(baseline, override) -> dict` (neither argument mutated; raises `KeyError` for an unknown key, `ValueError` for a bad band or an HSV-shaped colour)
  - `is_hsv_shaped(settings) -> bool`
  - `band(settings, color) -> (min list, max list)`
  - `yaml_block(settings) -> str`
  - `SEED = {'min': [0, 0, 0], 'max': [255, 255, 255]}`
  - `NAME_CHARS = 'abcdefghijklmnopqrstuvwxyz0123456789_'`

- [ ] **Step 1: Write the failing tests**

```python
# test/test_lab_settings.py
import pytest
import yaml

from rospider_gazebo import lab_settings


def test_defaults_are_a_fresh_copy():
    settings = lab_settings.defaults()
    assert settings['colors'] == ['red', 'green', 'blue']
    assert set(settings['red']) == {'min', 'max'}
    settings['colors'].append('pink')
    assert lab_settings.defaults()['colors'] == ['red', 'green', 'blue']


def test_merge_overlays_key_by_key():
    base = lab_settings.defaults()
    merged = lab_settings.merge(base, {'min_area_px': '500',
                                       'red': {'min': [1, 2, 3], 'max': [4, 5, 6]}})
    assert merged['min_area_px'] == 500
    assert merged['red'] == {'min': [1, 2, 3], 'max': [4, 5, 6]}
    assert merged['green'] == base['green']
    assert base['min_area_px'] == 300          # untouched


def test_merge_keeps_colours_the_override_adds_and_drops_the_rest():
    base = lab_settings.defaults()
    merged = lab_settings.merge(base, {
        'colors': ['red', 'pink'],
        'pink': {'min': [0, 140, 0], 'max': [255, 255, 255]}})
    assert merged['colors'] == ['red', 'pink']
    assert 'green' not in merged
    assert merged['pink']['min'] == [0, 140, 0]


def test_merge_validates():
    base = lab_settings.defaults()
    with pytest.raises(KeyError):
        lab_settings.merge(base, {'hue': 3})
    with pytest.raises(ValueError):
        lab_settings.merge(base, {'red': {'min': [0, 0], 'max': [255, 255, 255]}})
    with pytest.raises(ValueError):
        lab_settings.merge(base, {'red': {'min': [0, 0, 300], 'max': [255, 255, 255]}})
    with pytest.raises(ValueError):
        lab_settings.merge(base, {'colors': ['red', 'mauve']})   # no band for mauve


def test_hsv_shaped_file_is_recognised_and_refused():
    old = {'colors': ['red'], 'red': {'lower': [0, 120, 80], 'upper': [10, 255, 255]}}
    assert lab_settings.is_hsv_shaped(old)
    assert not lab_settings.is_hsv_shaped(lab_settings.defaults())
    with pytest.raises(ValueError, match='HSV'):
        lab_settings.merge(lab_settings.defaults(), old)


def test_band_and_yaml_round_trip():
    settings = lab_settings.defaults()
    settings['min_area_px'] = 123
    lo, hi = lab_settings.band(settings, 'blue')
    assert len(lo) == len(hi) == 3
    text = lab_settings.yaml_block(settings)
    assert text.splitlines()[:2] == ['color_detect:', '  ros__parameters:']
    parsed = yaml.safe_load(text)['color_detect']['ros__parameters']
    assert parsed['min_area_px'] == 123
    assert parsed['colors'] == ['red', 'green', 'blue']
    assert parsed['blue'] == {'min': lo, 'max': hi}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd ~/entech_hiwonder_ros2_ws/ROSpider/src/simulations/rospider_gazebo && python3 -m pytest test/test_lab_settings.py -q -p no:cacheprovider`
Expected: FAIL — `ImportError: cannot import name 'lab_settings'`

- [ ] **Step 3: Write the module**

```python
# rospider_gazebo/lab_settings.py
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
```

- [ ] **Step 4: Register the test and run it**

In `CMakeLists.txt`, after the `test_yolo_settings` pair add:
```cmake
  ament_add_pytest_test(test_lab_settings test/test_lab_settings.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
```
Run: `python3 -m pytest test/test_lab_settings.py -q -p no:cacheprovider`
Expected: 6 passed

- [ ] **Step 5: Commit**

```bash
git add rospider_gazebo/lab_settings.py test/test_lab_settings.py CMakeLists.txt
git commit -m "feat(sim): lab_settings -- LAB colour bands in lab_config.yaml's shape"
```

---

### Task 2: `color_detect.py` — LAB detection and upstream's drawing

**Files:**
- Modify: `scripts/color_detect.py` (everything from `DRAW_BGR` down to `_mask_for`; `run_tuner` and the panel code are replaced in Task 3)
- Modify: `config/color_detect.yaml`

**Interfaces:**
- Consumes: `lab_settings.defaults/merge/band/is_hsv_shaped/yaml_block/SEED/NAME_CHARS`
- Produces (used by Task 3's window): `self.settings` (live dict, guarded by `self._lock`), `self.apply_settings(settings)`, `self.save_tuned() -> str`, `self.yaml_block() -> str`, `self._last_frame`, `self._last_masks[color]`.

- [ ] **Step 1: Replace the config file**

```yaml
# config/color_detect.yaml
# LAB bands for the simulated cubes, in the shape of the real robot's
# lab_config.yaml: one min/max triple per colour over OpenCV's 8-bit
# L, A, B (0-255). A/B sit at 128 for grey; red pushes A up, green pulls A
# down, blue pulls B down and yellow pushes B up. L is left wide open so
# a cube in shadow still matches.
color_detect:
  ros__parameters:
    # 300: each colour mask matches TWO real objects, not one. The world
    # (worlds/rospider_room.sdf) has permanent, static, visual-only decorative
    # cubes named red_cube/green_cube/blue_cube at x=0.55 whose colour is
    # byte-for-byte the graspable pick_cube_{red,green,blue}'s at x=0.235.
    # A fixed area threshold can't separate them across camera poses, so
    # selection between same-colour blobs is left to the consumer
    # (pick_and_place: largest box per colour, confirmed reachable by IK).
    min_area_px: 300
    kernel_px: 5
    colors: ['red', 'green', 'blue']
    # circle: the min-enclosing circle of the largest blob, as upstream's
    # color_position / color_recognition ask for; rect: its min-area box
    # plus a centre dot. Only the drawn image_result differs.
    detect_type: circle

    # tune:=true opens the LAB_Tool-style window (mask left, camera right,
    # L/A/B min-max sliders, Color list with Add/Delete/Save). Off by
    # default: it needs a display and moves rclpy.spin onto a worker thread.
    tune: false
    # Written by Save and read back at startup if it exists. It OVERRIDES the
    # bands in this file, and the node logs loudly when it does. Delete it
    # to go back to the values here; Save also logs a YAML block to move a
    # value you want to keep back into this file, which is what's committed.
    tuned_path: '~/.ros/color_detect_tuned.json'
    red:
      min: [0, 150, 130]
      max: [255, 255, 255]
    green:
      min: [0, 0, 130]
      max: [255, 110, 255]
    blue:
      min: [0, 130, 0]
      max: [255, 255, 100]
```

- [ ] **Step 2: Rewrite the node's settings and detection**

Replace the module docstring's second paragraph with "With tune:=true the node also opens a Tk window laid out like Hiwonder's LAB_Tool; see run_tuner()." Delete `DRAW_BGR`, `NAME_CHARS`, `PREVIEW_KEY`, `PREVIEW_SEED`, `draw_bgr`, `TUNE_WINDOW`, `BAND_BARS`, `BAND_MAX`. Add imports `from rospider_gazebo import lab_settings, tkview`, `import tkinter as tk`, `from tkinter import ttk`, `from tkinter import simpledialog` and delete `from copy import deepcopy`. Then:

```python
#: Hiwonder's overlay colours (driver/sdk/sdk/common.py range_rgb), BGR.
RANGE_RGB = {'red': (0, 50, 255), 'green': (50, 255, 0), 'blue': (255, 50, 0)}

TUNE_TITLE = 'LAB_Tool 1.0'
LAB_HELP = ('LAB is composed of one lightness channel and two color channels. '
            'And each color is represented by three values, including L, A '
            'and B\nL refers to lightness;  A refers to the components from '
            'green to red;  B refers to the components from blue to yellow')


def draw_bgr(color, settings):
    """Overlay colour for a class name: Hiwonder's fixed three, or the BGR
    of the band's own LAB midpoint for a colour added in the window."""
    if color in RANGE_RGB:
        return RANGE_RGB[color]
    lo, hi = lab_settings.band(settings, color)
    mid = np.array([[[(lo[i] + hi[i]) // 2 for i in range(3)]]], dtype=np.uint8)
    return tuple(int(v) for v in cv2.cvtColor(mid, cv2.COLOR_LAB2BGR)[0, 0])


class ColorDetectNode(Node):

    def __init__(self):
        super().__init__('color_detect',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.bridge = CvBridge()
        self.detect_type = str(self._param('detect_type', 'circle'))
        if self.detect_type not in ('circle', 'rect'):
            raise ValueError(f'detect_type must be circle or rect, '
                             f'not {self.detect_type!r}')

        self._lock = threading.Lock()
        self.baseline = self._settings_from_params()
        self.settings = self.baseline
        self._apply(self.baseline)

        self.tune = bool(self._param('tune', False))
        self.tuned_path = Path(os.path.expanduser(
            str(self._param('tuned_path', '~/.ros/color_detect_tuned.json'))))
        self._load_tuned()

        # Shared with the window: the last frame and per-colour masks.
        self._last_frame = None
        self._last_masks = {}

        self.objects_pub = self.create_publisher(
            ObjectsInfo, '/yolo/object_detect', 1)
        self.image_pub = self.create_publisher(Image, '~/image_result', 1)
        self.create_subscription(
            Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.get_logger().info(
            f'watching for {self.settings["colors"]} on /depth_cam/rgb/image_raw')

    # ------------------------------------------------------------- settings

    def _param(self, name, default):
        value = self.get_parameter(name).value
        return default if value is None else value

    def _settings_from_params(self):
        """The committed baseline, read out of config/color_detect.yaml."""
        found = {
            'min_area_px': self._param('min_area_px', 300),
            'kernel_px': self._param('kernel_px', 5),
            'colors': list(self._param('colors', ['red', 'green', 'blue'])),
        }
        for color in found['colors']:
            found[color] = {'min': list(self._param(f'{color}.min', [])),
                            'max': list(self._param(f'{color}.max', []))}
        base = lab_settings.defaults()
        for color in base['colors']:
            if color not in found['colors']:
                del base[color]
        base['colors'] = []
        return lab_settings.merge(base, found)

    def _apply(self, settings):
        """Adopt a settings dict as the live detection parameters."""
        with self._lock:
            self.settings = settings
            self.min_area = int(settings['min_area_px'])
            self.kernel_px = max(1, int(settings['kernel_px']) | 1)
            self.kernel = cv2.getStructuringElement(
                cv2.MORPH_RECT, (self.kernel_px, self.kernel_px))

    def apply_settings(self, settings):
        """Validate then adopt; raises ValueError/KeyError on bad input."""
        self._apply(lab_settings.merge(lab_settings.defaults(), settings))

    def _load_tuned(self):
        if not self.tuned_path.is_file():
            self.get_logger().info(
                f'LAB bands from config/color_detect.yaml '
                f'(no tuned file at {self.tuned_path})')
            return
        try:
            with self.tuned_path.open() as handle:
                tuned = json.load(handle)
            self._apply(lab_settings.merge(self.baseline, tuned))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.get_logger().error(
                f'ignoring tuned file {self.tuned_path}: {exc}; '
                'using config/color_detect.yaml')
            return
        self.get_logger().warn(
            f'LAB bands OVERRIDDEN by {self.tuned_path} '
            '(delete it to go back to config/color_detect.yaml)')

    def save_tuned(self):
        """Write the live settings to the tuned JSON; returns a status line."""
        with self._lock:
            settings = json.loads(json.dumps(self.settings))
        try:
            self.tuned_path.parent.mkdir(parents=True, exist_ok=True)
            with self.tuned_path.open('w') as handle:
                json.dump(settings, handle, indent=2)
                handle.write('\n')
        except OSError as exc:
            self.get_logger().error(f'could not save {self.tuned_path}: {exc}')
            return f'could not save: {exc}'
        self.get_logger().info(f'saved {self.tuned_path}; as YAML:\n'
                               + lab_settings.yaml_block(settings))
        return f'saved {self.tuned_path}'

    def yaml_block(self):
        with self._lock:
            return lab_settings.yaml_block(self.settings)

    # ------------------------------------------------------------ detection

    def image_callback(self, msg):
        try:
            self._process_image(msg)
        except Exception:
            self.get_logger().error(
                'image_callback failed on this frame; skipping it',
                exc_info=True)

    def _process_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        result_image = frame.copy()
        height, width = frame.shape[:2]
        # Upstream: BGR -> LAB, then a 3x3 Gaussian blur before thresholding.
        img_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3)

        with self._lock:
            settings = self.settings
            min_area = self.min_area
            kernel = self.kernel

        masks = {}
        result = ObjectsInfo()
        biggest = None          # (area, colour, contour) of the largest blob
        for color in settings['colors']:
            lo, hi = lab_settings.band(settings, color)
            mask = cv2.inRange(img_blur, np.array(lo, dtype=np.uint8),
                               np.array(hi, dtype=np.uint8))
            mask = cv2.dilate(cv2.erode(mask, kernel), kernel)
            masks[color] = mask
            contours = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]
            survivors = 0
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < min_area:
                    continue
                survivors += 1
                x, y, w, h = cv2.boundingRect(contour)
                info = ObjectInfo()
                info.class_name = color
                info.box = [int(x), int(y), int(x + w), int(y + h)]
                info.score = 1.0
                info.width = int(width)
                info.height = int(height)
                info.angle = int(cv2.minAreaRect(contour)[2])
                result.objects.append(info)
                if biggest is None or area > biggest[0]:
                    biggest = (area, color, contour)
            if survivors > 1:
                self.get_logger().warn(
                    f'{survivors} {color} blobs above min_area_px '
                    '(expected: the graspable cube plus its same-colour '
                    'decorative twin further away)',
                    throttle_duration_sec=5.0)

        # Upstream draws one shape: the largest blob among the target
        # colours, as a circle or a rotated box with a centre dot.
        if biggest is not None:
            _area, color, contour = biggest
            overlay = draw_bgr(color, settings)
            if self.detect_type == 'circle':
                (cx, cy), radius = cv2.minEnclosingCircle(contour)
                cv2.circle(result_image, (int(cx), int(cy)), int(radius),
                           overlay, 2)
            else:
                box = np.intp(cv2.boxPoints(cv2.minAreaRect(contour)))
                cv2.drawContours(result_image, [box], -1, overlay, 2)
                cx = int((box[0, 0] + box[2, 0]) / 2)
                cy = int((box[0, 1] + box[2, 1]) / 2)
                cv2.circle(result_image, (cx, cy), 5, overlay, -1)

        self.objects_pub.publish(result)
        self.image_pub.publish(to_image_msg(result_image, msg.header))

        if self.tune:
            with self._lock:
                self._last_frame = frame
                self._last_masks = masks
```

Note the `masks[color]` for the window comes from the **raw** frame's threshold (before drawing), and `_last_frame` is the undrawn camera frame — LAB_Tool's right pane is the raw feed.

- [ ] **Step 3: Temporarily stub `run_tuner`**

Replace the whole old `run_tuner` and every helper below it (`_warn_if_qt_has_no_fonts`, the trackbar sync helpers, the panel builder — everything between `# ---- tuner` and `def main()`) with:

```python
    # ---------------------------------------------------------------- tuner

    def run_tuner(self):
        raise NotImplementedError('Task 3')
```

`main()` stays as it is.

- [ ] **Step 4: Build and check the node in the simulator**

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider && colcon build --packages-select rospider_gazebo --symlink-install && source install/local_setup.bash
ros2 launch rospider_gazebo pick_place.launch.py
```
In a second terminal: `ros2 topic echo /yolo/object_detect --once` shows three objects (`red`, `green`, `blue`) and `ros2 run rqt_image_view rqt_image_view /color_detect/image_result` shows one circle in `range_rgb` colour round the largest cube, no text. If a colour is missing, adjust its band in `config/color_detect.yaml` (Task 3's window makes this easier; a first guess is enough here). Then `ros2 service call /pick_and_place/start std_srvs/srv/Trigger` and confirm all three cubes are placed (`/pick_and_place/result`).

- [ ] **Step 5: Commit**

```bash
git add scripts/color_detect.py config/color_detect.yaml
git commit -m "feat(sim): color_detect thresholds in LAB and draws upstream's circle/box"
```

---

### Task 3: The LAB_Tool window

**Files:**
- Modify: `scripts/color_detect.py` (`run_tuner`)

- [ ] **Step 1: Write the window**

Replace the stub from Task 2 Step 3:

```python
    # ---------------------------------------------------------------- tuner

    def run_tuner(self):
        """Tk window laid out like Hiwonder's LAB_Tool 1.0. Main thread only.

        Top: the selected colour's mask (left) and the camera (right).
        Middle: L, A, B rows, each a min slider and a max slider, 0-255.
        Right: the Color list, Add / Delete / Save, and Quit. Every slider
        change applies to the running detector at once, so the mask,
        /color_detect/image_result and /yolo/object_detect all follow.
        Closing the window leaves the node detecting with the last values.
        """
        root = tk.Tk()
        root.title(TUNE_TITLE)

        panes = ttk.Frame(root)
        panes.grid(row=0, column=0, columnspan=2, padx=6, pady=6)
        mask_label = ttk.Label(panes)
        mask_label.grid(row=0, column=0, padx=(0, 4))
        frame_label = ttk.Label(panes)
        frame_label.grid(row=0, column=1)

        with self._lock:
            settings = json.loads(json.dumps(self.settings))
        colors = list(settings['colors'])
        selected = tk.StringVar(value=colors[0] if colors else '')
        message = tk.StringVar()
        loading = [False]           # True while the sliders are being set

        # --- L / A / B rows --------------------------------------------
        rows = ttk.Frame(root)
        rows.grid(row=1, column=0, padx=6, pady=(0, 6), sticky='nw')
        rows.columnconfigure(1, weight=1)
        rows.columnconfigure(2, weight=1)
        variables = {}              # (channel, 'min'|'max') -> IntVar
        for r, channel in enumerate('LAB'):
            ttk.Label(rows, text=channel, width=2).grid(row=r, column=0)
            for c, edge in enumerate(('min', 'max')):
                var = tk.IntVar(value=0)
                variables[(channel, edge)] = var
                tkview.LabeledScale(rows, f'{channel} {edge}', var, 0, 255, 1,
                                    lambda: apply()).grid(
                    row=r, column=1 + c, sticky='ew', padx=4)
        ttk.Label(rows, text=LAB_HELP, justify='left', wraplength=620).grid(
            row=3, column=0, columnspan=3, sticky='w', pady=(6, 0))

        # --- Color list and buttons ------------------------------------
        side = ttk.Frame(root)
        side.grid(row=1, column=1, padx=6, pady=(0, 6), sticky='n')
        ttk.Label(side, text='Color list').grid(row=0, column=0)
        chooser = ttk.Combobox(side, textvariable=selected, values=colors,
                               state='readonly', width=12)
        chooser.grid(row=1, column=0, pady=(0, 6))
        ttk.Button(side, text='Add', command=lambda: do_add()).grid(
            row=2, column=0, sticky='ew')
        ttk.Button(side, text='Delete', command=lambda: do_delete()).grid(
            row=3, column=0, sticky='ew')
        ttk.Button(side, text='Save',
                   command=lambda: message.set(self.save_tuned())).grid(
            row=4, column=0, sticky='ew')
        ttk.Button(side, text='Quit', command=root.destroy).grid(
            row=5, column=0, sticky='ew', pady=(12, 0))
        ttk.Label(side, textvariable=message, wraplength=160,
                  justify='left').grid(row=6, column=0, sticky='w', pady=(6, 0))

        def load_sliders():
            """Put the selected colour's band on the sliders."""
            color = selected.get()
            if not color:
                return
            lo, hi = lab_settings.band(settings, color)
            loading[0] = True
            for i, channel in enumerate('LAB'):
                variables[(channel, 'min')].set(lo[i])
                variables[(channel, 'max')].set(hi[i])
            loading[0] = False

        def apply():
            """Sliders -> settings -> live detector."""
            if loading[0] or not selected.get():
                return
            try:
                lo = [variables[(ch, 'min')].get() for ch in 'LAB']
                hi = [variables[(ch, 'max')].get() for ch in 'LAB']
                settings[selected.get()] = {'min': lo, 'max': hi}
                self.apply_settings(settings)
            except (ValueError, KeyError, tk.TclError) as exc:
                message.set(f'not applied: {exc}')
                return
            message.set('')

        def do_add():
            name = simpledialog.askstring('Add color', 'name (a-z, 0-9, _):',
                                          parent=root)
            if not name:
                return
            name = name.strip().lower()
            if not name or any(ch not in lab_settings.NAME_CHARS for ch in name):
                message.set(f'bad name {name!r}')
                return
            if name not in settings['colors']:
                settings['colors'].append(name)
                settings[name] = json.loads(json.dumps(lab_settings.SEED))
            chooser['values'] = list(settings['colors'])
            selected.set(name)
            load_sliders()
            apply()

        def do_delete():
            name = selected.get()
            if not name:
                return
            settings['colors'].remove(name)
            del settings[name]
            chooser['values'] = list(settings['colors'])
            selected.set(settings['colors'][0] if settings['colors'] else '')
            load_sliders()
            try:
                self.apply_settings(settings)
                message.set(f'deleted {name}')
            except (ValueError, KeyError) as exc:
                message.set(f'not applied: {exc}')

        chooser.bind('<<ComboboxSelected>>', lambda _e: load_sliders())
        load_sliders()

        def refresh():
            with self._lock:
                frame = self._last_frame
                mask = self._last_masks.get(selected.get())
            if frame is not None:
                photo = tkview.photo_from_bgr(frame, max_width=480)
                frame_label.configure(image=photo)
                frame_label.image = photo
                if mask is None:
                    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
                shown = tkview.photo_from_bgr(
                    cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), max_width=480)
                mask_label.configure(image=shown)
                mask_label.image = shown
            root.after(50, refresh)
        refresh()

        self.get_logger().info(
            f'LAB_Tool window open: Save writes {self.tuned_path} and logs '
            'a YAML block for config/color_detect.yaml; Quit closes the window')
        tkview.mainloop_until_shutdown(root)
        self.get_logger().info('LAB_Tool window closed; still detecting')
```

- [ ] **Step 2: Try it**

```bash
ros2 launch rospider_gazebo pick_place.launch.py tune:=true
```
Expect a window titled `LAB_Tool 1.0`: mask left / camera right, three slider rows, Color list with `red green blue`, Add / Delete / Save / Quit. Moving `A min` on `red` changes the left pane at once. `Add` → name `pink` → seeds a wide-open band; `Delete` removes it; `Save` writes `~/.ros/color_detect_tuned.json` and the terminal shows the YAML block. Restart without `tune` and confirm the `OVERRIDDEN by` warning; delete the JSON afterwards. Then write `{"red": {"lower": [0,0,0], "upper": [1,1,1]}}` into the JSON, start once more and confirm the `ignoring tuned file ... HSV-shaped` error; delete the JSON.

- [ ] **Step 3: Commit**

```bash
git add scripts/color_detect.py
git commit -m "feat(sim): LAB_Tool-style tuning window for color_detect"
```

---

### Task 4: Tune the three YAMLs in the simulator

**Files:**
- Modify: `config/color_detect.yaml`, `config/color_detect_arena.yaml`, `config/color_detect_arena_solved.yaml`
- Modify: `launch/pick_place.launch.py:157`, `launch/mini_game.launch.py:43,421` (wording: `HSV` → `LAB`)

- [ ] **Step 1: RGB cubes**

With `pick_place.launch.py tune:=true`, for each of red/green/blue narrow the band until the mask shows the two cubes of that colour and nothing else (floor, walls, other cubes all black). Save, copy the logged YAML values into `config/color_detect.yaml`, delete `~/.ros/color_detect_tuned.json`.

- [ ] **Step 2: Pastel cubes**

```bash
ros2 launch rospider_gazebo mini_game.launch.py detector:=color tune:=true map:=arena color_config:=color_detect_arena_solved.yaml
```
Find bands for `pink`, `yellow`, `sky` that show the cube and not the light-grey floor (grey sits at A ≈ B ≈ 128; the pastels are a few tens away). Write them into `color_detect_arena_solved.yaml` with `min`/`max` keys. Then write `color_detect_arena.yaml` with the same L range but chroma floors the pastels never reach — e.g. `pink.min[1]` (A) 40 higher than the solved value, `yellow.min[2]` (B) 40 higher, `sky.max[2]` (B) 40 lower — and check with the window that the starter file detects nothing. Rewrite both files' header comments to say which slider a team must move (the A or B floor) instead of "lower[1] (S)".

- [ ] **Step 3: Prove the consumers still work**

- `pick_place.launch.py` → `/pick_and_place/start`: three cubes placed.
- `mini_game.launch.py detector:=color map:=arena color_config:=color_detect_arena_solved.yaml` with `config/missions/basic.yaml`: at least the first cube is picked and delivered (`/mission/summary`).

- [ ] **Step 4: Commit**

```bash
git add config/color_detect.yaml config/color_detect_arena.yaml config/color_detect_arena_solved.yaml launch/pick_place.launch.py launch/mini_game.launch.py
git commit -m "feat(sim): LAB bands for the room cubes and the arena pastels"
```

---

### Task 5: `vision_demo.py` — FPS opt-in, `banner()` removed

**Files:**
- Modify: `rospider_gazebo/vision_demo.py`

**Interfaces:**
- Produces: class attribute `VisionDemo.show_fps = False`; `run()` calls `self.fps.show_fps(result)` only when it is `True`. `banner` no longer exists (Tasks 6-10 remove every import of it).

- [ ] **Step 1: Edit**

Delete the `banner()` function. In `VisionDemo`, under `window = None`, add:
```python
    #: Draw upstream's FPS caption. Only the demos whose original calls
    #: fps.show_fps() turn this on.
    show_fps = False
```
In `run()`, change
```python
            self.fps.update()
            self.fps.show_fps(result)
```
to
```python
            self.fps.update()
            if self.show_fps:
                self.fps.show_fps(result)
```
Update the module docstring's three-bullet list: no change needed (it lists the plumbing differences, not the caption).

- [ ] **Step 2: Check nothing else imports `banner`**

Run: `grep -rn "banner" rospider_gazebo/ scripts/ | grep -v "^scripts/"`
Expected: no output (the `scripts/` hits are fixed in Tasks 6-10; the package must import cleanly until then, so leave those scripts alone in this task).

- [ ] **Step 3: Commit**

```bash
git add rospider_gazebo/vision_demo.py
git commit -m "refactor(sim): VisionDemo draws FPS only on opt-in, banner() removed"
```

---

### Task 6: Colour demos

**Files:**
- Modify: `scripts/color_position.py`, `scripts/color_recognition.py`

- [ ] **Step 1: `color_position.py`**

Upstream (`opencv_example/include/color_position.py:132-138`, window `'image'`): on the detector's frame, a filled 5 px `(0, 255, 255)` dot at the centre and `"({:0.1f}, {:0.1f})"` 16 px below it, `FONT_HERSHEY_SIMPLEX` 0.6 `(0, 255, 255)` thickness 2; the centre is logged. Nothing when there is no target.

- Change the import to `from rospider_gazebo.vision_demo import VisionDemo` and delete `STABLE_FRAMES`, `self.stable`, `self.last_name`.
- Add `window = 'image'` as a class attribute.
- `objects_callback` keeps the largest detection but logs it as upstream does:
```python
    def objects_callback(self, message):
        objects = [o for o in message.objects
                   if not self.wanted or o.class_name == self.wanted]
        best = largest(objects)
        if best is None:
            self.target = None
            return
        obj, (u, v, _area) = best
        self.target = (obj.class_name, u, v)
        self.get_logger().info(f'(x, y):{(u, v)}', throttle_duration_sec=0.5)
```
- `process`:
```python
    def process(self, frame):
        if self.target is None:
            return frame
        _name, u, v = self.target
        cv2.circle(frame, (int(u), int(v)), 5, DRAW, -1)
        cv2.putText(frame, '({:0.1f}, {:0.1f})'.format(u, v),
                    (int(u), int(v) + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    DRAW, 2)
        return frame
```

- [ ] **Step 2: `color_recognition.py`**

Upstream (`color_recognition_node.py:165-175`, window `'image'`) shows the detector's frame untouched and logs `color: <name>` when a move starts (the sim already logs that in `_act_loop`).

- Import only `VisionDemo`; add `window = 'image'`.
- `process`:
```python
    def process(self, frame):
        if not self.acting:
            if self.color in MOVES:
                self.count += 1
                if self.count > STABLE_FRAMES:
                    self.count = 0
                    self.target = self.color
                    self.acting = True
            else:
                self.count = 0
        return frame
```

- [ ] **Step 3: Check in the simulator**

`pick_place.launch.py` in one terminal, then `vision_demo.launch.py demo:=color_position` and `demo:=color_recognition`: window title `image`, the detector's circle plus (for position) the yellow dot and coordinates; no FPS, no captions. `flake8 scripts/color_position.py scripts/color_recognition.py` clean.

- [ ] **Step 4: Commit**

```bash
git add scripts/color_position.py scripts/color_recognition.py
git commit -m "refactor(sim): color_position / color_recognition draw only what upstream draws"
```

---

### Task 7: AprilTag demos and `apriltag_detect`'s frame

**Files:**
- Modify: `scripts/apriltag_detect.py` (`_draw`, plus the module constants), `scripts/apriltag_position.py`, `scripts/apriltag_track.py`, `scripts/ar_view.py`

- [ ] **Step 1: `apriltag_detect.py` — upstream's axes, disc, corner dots and label**

Upstream (`apriltag_recognition.py:18-37, 90-105`, RGB): four corner dots radius 2 `(0, 255, 255)`; an axis triad of length 1.5 half-tags with a filled disc of radius 0.3 half-tags, projected with solvePnP — disc `(255, 255, 0)` filled, x-axis `(255, 0, 0)`, y `(0, 255, 0)`, z `(0, 0, 255)`, all thickness 3; text `'id' + str(id)` centred horizontally under the tag (`text_size[1] + 25` below the centre), `FONT_HERSHEY_SIMPLEX` 0.6 `(255, 255, 0)` thickness 2. In BGR those become: dots `(255, 255, 0)`, disc `(0, 255, 255)`, x `(0, 0, 255)`, y `(0, 255, 0)`, z `(255, 0, 0)`, text `(0, 255, 255)`.

Add near the top of the file (after the imports):
```python
#: Upstream's AXIS (apriltag_recognition.py) in half-tag units: the origin,
#: three axis tips at 1.5, then a 360-point circle of radius 0.3 on the tag
#: plane. Scaled by half the tag size at draw time, since the sim's pose is
#: solved in metres.
_AXIS_HALF_TAG = np.append(
    np.float32([[0, 0, 0], [1.5, 0, 0], [0, 1.5, 0], [0, 0, 1.5]]),
    np.float32([[0.3 * math.cos(math.radians(i)),
                 0.3 * math.sin(math.radians(i)), 0] for i in range(360)]),
    axis=0)
```
(add `import math` if missing). Replace `_draw`:
```python
    def _draw(self, frame, corners, tag_id, rvec, tvec):
        """Upstream's overlay (apriltag_recognition.py), BGR colours."""
        for pt in corners:
            cv2.circle(frame, (int(pt[0]), int(pt[1])), 2, (255, 255, 0), -1)
        axis = _AXIS_HALF_TAG * (self.tag_size / 2.0)
        imgpts, _ = cv2.projectPoints(axis, rvec, tvec, self.camera_matrix,
                                      self.dist_coeffs)
        imgpts = np.int32(imgpts).reshape(-1, 2)
        cv2.drawContours(frame, [imgpts[4:]], -1, (0, 255, 255), -1)
        cv2.line(frame, tuple(imgpts[0]), tuple(imgpts[1]), (0, 0, 255), 3)
        cv2.line(frame, tuple(imgpts[0]), tuple(imgpts[2]), (0, 255, 0), 3)
        cv2.line(frame, tuple(imgpts[0]), tuple(imgpts[3]), (255, 0, 0), 3)
        centre = corners.mean(axis=0)
        text = 'id' + str(tag_id)
        size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
        cv2.putText(frame, text,
                    (int(centre[0] - size[0] / 2), int(centre[1] + size[1] + 25)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
```

- [ ] **Step 2: `apriltag_position.py`**

Upstream (`apriltag_position.py:44-59`, window `'image'`): shows the detector's frame and logs `ID: .., X: .., Y: .., W: ..` for the first tag every frame. Import only `VisionDemo`, delete `DRAW`, add `window = 'image'`, and:
```python
    def process(self, frame):
        if self.tags:
            first = self.tags[0]
            self.get_logger().info(
                f'ID: {first.id}, X: {first.x:.2f}, Y: {first.y:.2f}, '
                f'W: {first.w:.2f}')
        return frame
```

- [ ] **Step 3: `apriltag_track.py`**

Upstream (`apriltag_track.py:108-135`, window `'image'`): shows the detector's frame and draws nothing else. Import only `VisionDemo`, delete `DRAW`, add `window = 'image'`. In `process`, replace `return banner(frame, f'TAG ... LOST', ...)` with `return frame`, and delete the four drawing lines after `self.drive(linear, angular)` (circle, line, banner, putText) so it ends with `return frame`. Keep the control logic untouched.

- [ ] **Step 4: `ar_view.py`**

Upstream (`ar.py:55-59, 171-174`, RGB): corner dots `(0, 255, 255)` → BGR `(255, 255, 0)`; cube base filled `(0, 255, 0)` (green either way), pillars scalar `255` = RGB `(255, 0, 0)` red → BGR `(0, 0, 255)`, top outline `(0, 0, 255)` RGB blue → BGR `(255, 0, 0)`. Window `'result'`.

- Set `CUBE_EDGE = (0, 255, 0)`, add `CUBE_PILLAR = (0, 0, 255)`, `CUBE_TOP = (255, 0, 0)`; in `draw_cube` use `CUBE_PILLAR` for the lines.
- Corner dots: `(255, 255, 0)`.
- Import only `VisionDemo`; add `window = 'result'`.
- In `process`: replace the `WAITING FOR camera_info` banner with
```python
        if self.intrinsics is None:
            self.get_logger().info('waiting for camera_info',
                                   throttle_duration_sec=2.0)
            return frame
```
and delete the `if not found: banner(...)` lines.

- [ ] **Step 5: Check in the simulator**

`pick_place.launch.py scene:=false` (tags only) then `demo:=apriltag_position`, `apriltag_track`, `ar_view`: axes (red/green/blue), yellow disc, cyan corner dots, `id1` under the tag; AR cube green base / red pillars / blue top. `python3 -m pytest test/test_apriltag.py -q -p no:cacheprovider` still passes; flake8 clean on the four files.

- [ ] **Step 6: Commit**

```bash
git add scripts/apriltag_detect.py scripts/apriltag_position.py scripts/apriltag_track.py scripts/ar_view.py
git commit -m "refactor(sim): AprilTag windows draw upstream's axes, dots and labels"
```

---

### Task 8: KCF and the two floor-probing depth demos

**Files:**
- Modify: `scripts/kcf_track.py`, `scripts/prevent_falling.py`, `scripts/cross_bridge.py`

- [ ] **Step 1: `kcf_track.py`**

Upstream (`kcf.py:85-126`, BGR, window `'result'`): rectangle `(255, 255, 0)` thickness 2 round the tracked box; nothing else. `selectROI` on the same window. Import only `VisionDemo`, add `window = 'result'`, keep `DRAW = (255, 255, 0)`, and:
```python
    def process(self, frame):
        if self.select_next:
            self.select_next = False
            self._select(frame)
            return frame
        if self.tracker is None:
            return frame
        found, box = self.tracker.update(frame)
        if not found:
            self.stop()
            self.pid_yaw.clear()
            self.get_logger().info('target lost', throttle_duration_sec=2.0)
            return frame
        x, y, w, h = (int(v) for v in box)
        cv2.rectangle(frame, (x, y), (x + w, y + h), DRAW, 2)
        centre_x = x + w / 2.0
        width = frame.shape[1]
        self.pid_yaw.SetPoint = width / 2.0
        self.pid_yaw.update(centre_x)
        angular = set_range(self.pid_yaw.output, -10, 10) / 10.0 * self.turn_limit
        self.drive(0.0, angular)
        return frame
```
(keep the PID sign comment from the current file.)

- [ ] **Step 2: `prevent_falling.py`**

Upstream (`prevent_falling_node.py:110-139`, window `'depth_color_map'`): JET map, three black 10 px dots, and `str([left, center, right])` logged every frame; in debug it only counts frames. Import only `VisionDemo` (keep `depth_color_map`), add `window = 'depth_color_map'`, and:
```python
    def process(self, depth_mm):
        distances = [depth_probe.roi_distance(depth_mm, roi)
                     for roi in FALL_ROIS.values()]
        view = depth_color_map(depth_mm)
        for roi in FALL_ROIS.values():
            cv2.circle(view, depth_probe.roi_center(roi), 10, (0, 0, 0), -1)
        self.get_logger().info(str(distances))
        if self.debug:
            self._calibrate(distances)
            return view
        command = self.policy.update(
            distances, self.get_clock().now().nanoseconds * 1e-9)
        if command is not None:
            self.drive(*command)
        return view

    def _calibrate(self, distances):
        """Average the probes, then switch to walking with what was measured."""
        if all(d > 0 for d in distances):
            self.samples.append(sum(distances) / len(distances))
        if len(self.samples) >= CALIBRATION_FRAMES:
            self.policy.plane_distance = round(
                sum(self.samples) / len(self.samples), 3)
            self.debug = False
            self.samples = []
            self.get_logger().info(
                f'floor is {self.policy.plane_distance} m away -- pass '
                f'plane_distance:={self.policy.plane_distance} next time to '
                'skip this')
```

- [ ] **Step 3: `cross_bridge.py`**

Upstream (`cross_bridge_node.py:163-237`, window `'depth_color_map'`): JET map + five black dots; distances logged. Same treatment: import only `VisionDemo`, `window = 'depth_color_map'`, `process` draws the map and dots, logs `str([distances[n] for n in BRIDGE_ROIS])`, runs the policy/drive, returns `view`; `_calibrate(center_distance)` keeps its averaging and log line but draws nothing.

- [ ] **Step 4: Check**

`gazebo.launch.py` then `demo:=kcf_track` (press `s`, drag, Enter → cyan box only), `demo:=prevent_falling` and `demo:=cross_bridge` (JET map with black dots, distances streaming in the terminal, title `depth_color_map`). flake8 clean.

- [ ] **Step 5: Commit**

```bash
git add scripts/kcf_track.py scripts/prevent_falling.py scripts/cross_bridge.py
git commit -m "refactor(sim): kcf_track / prevent_falling / cross_bridge draw upstream's overlay only"
```

---

### Task 9: `object_volume` and `object_classification`

**Files:**
- Modify: `scripts/object_volume.py`, `scripts/object_classification.py`

- [ ] **Step 1: `object_volume.py`**

Upstream (`object_volume_measurement.py:273-326`, window `'Object Classification (ROI Mode)'`): the sim already matches the running-state drawing (type text 0.7 white, red rotated box, info lines, dimmed outside ROI, white/cyan ROI rectangles, depth|rgb side by side). Changes:
- `window = 'Object Classification (ROI Mode)'`; import only `VisionDemo`, `depth_color_map`.
- Waiting: `self.get_logger().info('waiting for depth and camera_info', throttle_duration_sec=2.0); return frame`.
- Calibration (upstream logs `Calibrating Ground: {stable_dist} mm` each frame and shows no overlay): `_calibrate(near)` appends the sample, logs `f'Calibrating Ground: {near} mm'`, finishes as now, draws nothing; `process` returns `self._side_by_side(view, frame)` in that branch.

- [ ] **Step 2: `object_classification.py`**

Upstream (`object_classification.py:380, 427-483`, window `'depth'`): depth|rgb side by side; every object gets a white 2 px bounding box on the depth map; the **target** (the nearest object whose shape is in `shapes`) gets its name in `FONT_HERSHEY_COMPLEX` 1.0 at `(x + w // 2, y + h // 2 - 10)` — black thickness 2 `LINE_AA`, then white thickness 1 — and a red `(0, 0, 255)` rotated box `LINE_AA`; the rgb half gets the ROI rectangle `(255, 255, 0)` thickness 1. No ROI rectangle on the depth half, no dimming, no per-object text.

Rewrite `process`:
```python
    def process(self, frame):
        if self.depth_mm is None or self.intrinsics is None:
            self.get_logger().info('waiting for depth and camera_info',
                                   throttle_duration_sec=2.0)
            return frame
        depth_mm = self.depth_mm
        if depth_mm.shape[:2] != frame.shape[:2]:
            frame = cv2.resize(frame, (depth_mm.shape[1], depth_mm.shape[0]))
        near = shape_detect.nearest_distance(depth_mm, self.roi,
                                             self.plane_distance)
        view = depth_color_map(depth_mm, DEPTH_CEILING)
        if self.debug:
            self._calibrate(near)
            return np.concatenate([view, frame], axis=1)

        objects = shape_detect.recognise(depth_mm, frame, self.intrinsics,
                                         self.plane_distance, near, self.roi)
        for obj in objects:
            x, y, w, h = obj.box
            cv2.rectangle(view, (x, y), (x + w, y + h), (255, 255, 255), 2)
        wanted = [o for o in objects if o.kind in self.shapes]
        if wanted:
            target = min(wanted, key=lambda o: o.position[2])
            x, y, w, h = target.box
            label = target.kind
            cv2.putText(view, label, (x + w // 2, y + h // 2 - 10),
                        cv2.FONT_HERSHEY_COMPLEX, 1.0, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(view, label, (x + w // 2, y + h // 2 - 10),
                        cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 255, 255), 1)
            cv2.drawContours(view, [np.int32(cv2.boxPoints(target.rect))], -1,
                             (0, 0, 255), 2, cv2.LINE_AA)
            px, py, pz = target.position
            self.get_logger().info(
                f'{shape_detect.colour_name(target.bgr)} {target.name} '
                f'({px:+.3f}, {py:+.3f}, {pz:.3f}) m {target.angle:.0f} deg',
                throttle_duration_sec=1.0)
        cv2.rectangle(frame, (self.roi[2], self.roi[0]),
                      (self.roi[3], self.roi[1]), (255, 255, 0), 1)
        return np.concatenate([view, frame], axis=1)
```
`_calibrate(near)` appends, logs `f'Calibrating Ground: {near} mm'`, finishes as now, draws nothing. `window = 'depth'`; delete `DRAW_BGR`; import `numpy as np` if not already; import only `VisionDemo`, `depth_color_map`. Check `obj.kind`, `obj.rect`, `obj.position` exist on `shape_detect.recognise`'s result (they are used by the current file; `kind` is the shape without the index).

- [ ] **Step 3: Check**

`pick_place.launch.py` then `demo:=object_volume` and `demo:=object_classification`: titles as above, side-by-side views; classification shows white boxes on all cubes, the nearest one labelled in COMPLEX font with a red rotated box. `python3 -m pytest test/test_shape_detect.py -q -p no:cacheprovider` passes; flake8 clean.

- [ ] **Step 4: Commit**

```bash
git add scripts/object_volume.py scripts/object_classification.py
git commit -m "refactor(sim): depth demos draw upstream's overlay; calibration logs instead"
```

---

### Task 10: MediaPipe demos

**Files:**
- Modify: `scripts/hand_detect.py`, `scripts/hand_gesture.py`, `scripts/finger_trajectory.py`, `scripts/face_track.py`, `scripts/pose_control.py`

- [ ] **Step 1: `hand_detect.py`**

Upstream (`hand_detect.py:50-55`, window `'hand_detect'`): mirrored frame with landmarks drawn by `mediapipe_visual.draw_hand_landmarks_on_image` (landmarks + connections, plus the handedness word in `FONT_HERSHEY_PLAIN` size 1 `(255, 255, 0)` thickness 1 at the hand's top-left + 10 px margin). Import only `VisionDemo`; `window = 'hand_detect'`; `process` becomes:
```python
    def process(self, frame):
        results = self.detector.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        if not results.multi_hand_landmarks:
            return frame
        height, width = frame.shape[:2]
        for hand, handed in zip(results.multi_hand_landmarks,
                                results.multi_handedness):
            self.drawing.draw_landmarks(frame, hand,
                                        mp.solutions.hands.HAND_CONNECTIONS)
            xs = [lm.x * width for lm in hand.landmark]
            ys = [lm.y * height for lm in hand.landmark]
            cv2.putText(frame, handed.classification[0].label,
                        (int(min(xs)) + 10, int(min(ys)) - 10),
                        cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 0), 1, cv2.LINE_AA)
        return frame
```
Drop the `gestures` import if nothing else uses it.

- [ ] **Step 2: `hand_gesture.py`**

Upstream (`hand_gesture.py:229-256`, BGR, window `'result_image'`): mirrored; landmarks; `gesture.upper()` at `(10, 100)` `FONT_HERSHEY_SIMPLEX` 1.2 black thickness 5 then `(255, 255, 0)` thickness 2. Nothing while no hand. Import only `VisionDemo`; `window = 'result_image'`; in `process` replace `return banner(frame, 'RUNNING', ...)` and `return banner(frame, 'NO HAND', ...)` with `return frame`, replace `banner(frame, gesture.upper())` with
```python
        cv2.putText(frame, gesture.upper(), (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 5)
        cv2.putText(frame, gesture.upper(), (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 0), 2)
```
and delete the progress `cv2.rectangle(...)` block. Keep `import cv2`.

- [ ] **Step 3: `finger_trajectory.py`**

Upstream (`finger_trajectory.py:157-243`, windows `'image'` + `'track'`): mirrored; landmarks; the blue `(255, 0, 0)` 4 px trail while tracking; the `track` window with green contour, red approx polygon and the shape name at `(10, 40)` 1.2 `(255, 255, 0)` — `shape_detect.trajectory_shape` already renders that. Import only `VisionDemo`; `window = 'image'`; delete `_caption` and every `return self._caption(frame)` becomes `return frame`. Keep `self.get_logger().info(f'drawn shape: ...')` in `_finish`.

- [ ] **Step 4: `face_track.py`**

Upstream (`face_track.py:178-190`, BGR, not mirrored, window `'result'`): `show_faces` green 2 px boxes and red radius-2 keypoints, plus `show_fps`. Import only `VisionDemo`; `window = 'result'`; `show_fps = True`; in `process` replace `return banner(frame, 'NO FACE', ...)` with `return frame`, replace `return banner(frame, f'LOCKING ...')` with `return frame`, and delete the final `cv2.circle(frame, centre, 5, (0, 255, 255), -1)` and `return banner(frame, f'PAN ...')` in favour of `return frame`.

- [ ] **Step 5: `pose_control.py`**

Upstream (`pose_control.py:129-140, 217-224, 333-335`, BGR, window `'image'`): mirrored; pose landmarks via `draw_landmarks`; every landmark also a filled 5 px `(255, 0, 0)` dot, then landmark 11 `(255, 255, 0)`, 12 `(0, 255, 255)`, 14 `(0, 255, 0)`; shown resized to `display_size = (1280, 800)`. Import only `VisionDemo`; `window = 'image'`; add `DISPLAY_SIZE = (int(640 * 8 / 4), int(400 * 8 / 4))`; delete `_caption`; `process` becomes:
```python
    def process(self, frame):
        results = self.pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        if not results.pose_landmarks:
            return cv2.resize(frame, DISPLAY_SIZE)
        self.drawing.draw_landmarks(frame, results.pose_landmarks,
                                    mp.solutions.pose.POSE_CONNECTIONS)
        height, width = frame.shape[:2]
        landmarks = gestures.landmarks_to_pixels(
            (width, height), results.pose_landmarks.landmark)
        for cx, cy in landmarks:
            cv2.circle(frame, (int(cx), int(cy)), 5, (255, 0, 0), cv2.FILLED)
        for index, color in ((11, (255, 255, 0)), (12, (0, 255, 255)),
                             (14, (0, 255, 0))):
            cv2.circle(frame, tuple(int(v) for v in landmarks[index]), 5,
                       color, cv2.FILLED)
        <the existing state machine, unchanged, from `if self.state is State.NULL:` to its end>
        return cv2.resize(frame, DISPLAY_SIZE)
```
Every early `return self._caption(frame)` inside the state machine becomes `return cv2.resize(frame, DISPLAY_SIZE)`. State changes are already logged; if not, add `self.get_logger().info(f'state: {self.state.value}')` where `self.state` is assigned.

- [ ] **Step 6: Check**

`python3 -m pytest test/test_gestures.py -q -p no:cacheprovider` passes; flake8 clean on the five files; `grep -rn banner scripts/` returns nothing. If a webcam exists (`ls /dev/video*`): `vision_demo.launch.py demo:=hand_gesture source:=webcam use_sim_time:=false` shows landmarks + the yellow gesture word only; `demo:=pose_control source:=webcam use_sim_time:=false` opens a 1280×800 window. Otherwise note in the commit that MediaPipe demos were checked by import only (`python3 -c "import scripts.hand_detect"` is not importable as a package — instead run each with `show:=false use_sim_time:=false` for 5 s and confirm no traceback).

- [ ] **Step 7: Commit**

```bash
git add scripts/hand_detect.py scripts/hand_gesture.py scripts/finger_trajectory.py scripts/face_track.py scripts/pose_control.py
git commit -m "refactor(sim): MediaPipe demos draw upstream's overlay only"
```

---

### Task 11: Docs

**Files:**
- Modify: `ROSpider/SIMULATION.md` (§7 colour tuning at lines ~294-337, §10 arena notes at ~676, 703, 722, §11 demo table at ~800-860)
- Modify: `ROSpider/CLAUDE.md` (the `Ported demo windows` and colour-tuner bullets)

- [ ] **Step 1: SIMULATION.md §7**

Rewrite "จูนช่วงสี HSV ด้วย slider" as "จูนช่วงสี LAB ด้วยหน้าต่าง LAB_Tool": the window is the same layout as the real robot's LAB_Tool (mask left / camera right, L A B min-max, Color list, Add / Delete / Save, Quit); the config shape is `min`/`max` like `lab_config.yaml`; one band per colour (delete the two-band red paragraph); precedence rule unchanged; Save logs the YAML block (no `y` key); an old HSV-shaped JSON is refused with an error. Replace "HSV" with "LAB" at lines 222 and 294; note `detect_type: circle|rect` and that the frame shows one shape for the largest blob, as upstream.

- [ ] **Step 2: SIMULATION.md §10**

Lines ~676, 703, 722: the colour team tunes the A/B chroma floor (pink = A min, yellow = B min, sky = B max) instead of S; the starter bands sit 40 above/below the solved values.

- [ ] **Step 3: SIMULATION.md §11**

Add under "เรื่องที่ต่างจากหุ่นจริง": the windows now draw exactly what the originals draw (titles included); simulation-only states (waiting for a topic, calibration, target lost) go to the terminal, as the originals print. Update the table's `color_position`/`apriltag_position` rows if they mention captions.

- [ ] **Step 4: CLAUDE.md**

In the `Ported demo windows` bullet add: "Windows draw only what the upstream script draws (same primitives, BGR-translated colours, upstream titles); `VisionDemo.show_fps` opts a demo into the FPS caption; sim-only states are logged, never drawn." In the colour-detector mention (Mini game bullet, `color_detect` precedence sentence) say LAB / `lab_settings.py` instead of HSV.

- [ ] **Step 5: Commit**

```bash
git add ROSpider/SIMULATION.md CLAUDE.md
git commit -m "docs(sim): LAB_Tool window, LAB bands, demo windows match upstream"
```

---

### Task 12: Full verification pass

- [ ] **Step 1: Tests and lint**

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider && colcon build --packages-select rospider_gazebo --symlink-install && source install/local_setup.bash
cd src/simulations/rospider_gazebo && python3 -m pytest test/ -q -p no:cacheprovider && python3 -m flake8 scripts rospider_gazebo --max-line-length 99
```
Expected: all tests pass (166 + 6 new), flake8 clean.

- [ ] **Step 2: Every non-MediaPipe demo in Gazebo**

With `pick_place.launch.py` running, open each of `color_position`, `color_recognition`, `apriltag_position`, `apriltag_track`, `ar_view`, `kcf_track`, `prevent_falling`, `cross_bridge`, `object_volume`, `object_classification` in turn and compare with the wiki's screenshot for that section (https://wiki.hiwonder.com/projects/ROSpider/en/jetson-nano-version/docs/6_ROS+OpenCV_Course.html and the RGB-D course page). Record any mismatch and fix it in the demo's task before finishing.

- [ ] **Step 3: Report**

Summarise per window what was changed and what was verified, and list anything not run (MediaPipe without a webcam).
