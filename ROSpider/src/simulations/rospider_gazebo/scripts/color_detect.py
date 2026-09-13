#!/usr/bin/env python3
"""HSV cube detector for the simulation.

Publishes interfaces/ObjectsInfo on /yolo/object_detect, the same topic and
message competition/yolo_node.py uses on the real robot, so a YOLO node can
replace this one without the pick node changing.

With tune:=true the node also opens an OpenCV trackbar window for adjusting the
HSV bounds live. See run_tuner() for the key bindings and the two-file
precedence rule between config/color_detect.yaml and the tuned JSON.
"""

import json
import os
import threading
from copy import deepcopy
from pathlib import Path

# The pip-installed opencv-python bundles a Qt build with no fonts of its own
# ("QFontDatabase: Cannot find font directory .../cv2/qt/fonts"), and highgui
# then draws every trackbar label as a blank strip -- eight unlabelled sliders.
# Point Qt at the system fonts. This must happen before cv2 is imported, which
# is when Qt initialises, and it defers to the value if one is already set.
for _font_dir in ('/usr/share/fonts/truetype/dejavu',
                  '/usr/share/fonts/truetype/liberation'):
    if os.path.isdir(_font_dir):
        os.environ.setdefault('QT_QPA_FONTDIR', _font_dir)
        break

import cv2  # noqa: E402  (must follow the QT_QPA_FONTDIR default above)
import numpy as np
import rclpy
from cv_bridge import CvBridge
from interfaces.msg import ObjectInfo, ObjectsInfo
from rclpy.node import Node
from sensor_msgs.msg import Image

DRAW_BGR = {'red': (0, 0, 255), 'green': (0, 255, 0), 'blue': (255, 0, 0)}
# Names a new colour may be given in the tuner. Deliberately narrow: the name
# becomes ObjectInfo.class_name, a ROS topic's payload and a YAML key.
NAME_CHARS = 'abcdefghijklmnopqrstuvwxyz0123456789_'
# Mask slot for the not-yet-named colour being tuned. '+' is not in NAME_CHARS,
# so this can never collide with a real colour.
PREVIEW_KEY = '+preview'
# What a new colour starts from: wide open, so its preview mask shows
# everything and the hue can be narrowed down onto the target.
PREVIEW_SEED = [[0, 80, 60], [179, 255, 255]]


def draw_bgr(color, bands):
    """Overlay colour for a class name.

    The three built-in names keep their fixed colour. A colour added in the
    tuner gets one derived from the midpoint of its own HSV band, so its boxes
    look like whatever it matches without needing to be configured.
    """
    if color in DRAW_BGR:
        return DRAW_BGR[color]
    lower, upper = bands[0]
    # Hue from the band's midpoint, but saturation and value pinned to full:
    # this is an overlay that has to stand out on the frame, not a sample of
    # the matched pixels. Taking the band's own S/V midpoint gives a muted
    # colour that reads badly against the image.
    mid = np.array([[[(lower[0] + upper[0]) // 2, 255, 255]]], dtype=np.uint8)
    return tuple(int(v) for v in cv2.cvtColor(mid, cv2.COLOR_HSV2BGR)[0, 0])

TUNE_WINDOW = 'color_detect tune'
# Trackbar labels. Kept short because highgui draws them in a fixed-width gutter.
BAND_BARS = ('H lo', 'H hi', 'S lo', 'S hi', 'V lo', 'V hi')
BAND_MAX = (179, 179, 255, 255, 255, 255)


class ColorDetectNode(Node):

    def __init__(self):
        super().__init__('color_detect',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.bridge = CvBridge()

        self.colors = list(self._param('colors', ['red', 'green', 'blue']))
        self.baseline = self._settings_from_params()
        self._apply_settings(self.baseline)

        self.tune = bool(self._param('tune', False))
        self.tuned_path = Path(os.path.expanduser(
            str(self._param('tuned_path', '~/.ros/color_detect_tuned.json'))))
        self._load_tuned()

        # Shared with the tuner thread: the UI reads the last frame and masks,
        # and writes HSV bounds back into the same settings the callback uses.
        self._lock = threading.Lock()
        self._last_frame = None
        self._last_masks = {}
        self._preview = None      # candidate band while a colour is being added

        self.objects_pub = self.create_publisher(
            ObjectsInfo, '/yolo/object_detect', 1)
        self.image_pub = self.create_publisher(Image, '~/image_result', 1)
        self.create_subscription(
            Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.get_logger().info(
            f'watching for {sorted(self.ranges)} on /depth_cam/rgb/image_raw')

    # ------------------------------------------------------------- settings

    def _param(self, name, default):
        """Read a parameter, falling back when the YAML does not carry it.

        The node declares parameters from overrides, so anything absent from
        config/color_detect.yaml comes back with value None rather than raising.
        """
        value = self.get_parameter(name).value
        return default if value is None else value

    def _settings_from_params(self):
        """The committed baseline, read out of config/color_detect.yaml."""
        settings = {
            'min_area_px': int(self._param('min_area_px', 300)),
            'kernel_px': int(self._param('kernel_px', 5)),
            'colors': list(self.colors),
        }
        for color in self.colors:
            settings[color] = {
                'lower': [int(v) for v in self._param(f'{color}.lower', [])],
                'upper': [int(v) for v in self._param(f'{color}.upper', [])],
            }
        return settings

    @staticmethod
    def _merge_settings(baseline, override):
        """Baseline with an override laid over it, colour list included.

        Colours the override adds are kept and colours it drops are discarded,
        so a colour created in the tuner survives a restart. Raises if the
        override names a colour it carries no bounds for.
        """
        merged = {
            'min_area_px': int(override.get('min_area_px',
                                            baseline['min_area_px'])),
            'kernel_px': int(override.get('kernel_px',
                                          baseline['kernel_px'])),
            'colors': list(override.get('colors', baseline['colors'])),
        }
        for color in merged['colors']:
            bounds = override.get(color, baseline.get(color))
            if not bounds:
                raise KeyError(f'no HSV bounds for colour {color!r}')
            merged[color] = {'lower': [int(v) for v in bounds['lower']],
                             'upper': [int(v) for v in bounds['upper']]}
        return merged

    def _apply_settings(self, settings):
        """Adopt a settings dict as the live detection parameters."""
        self.colors = list(settings['colors'])
        self.min_area = int(settings['min_area_px'])
        self.kernel_px = max(1, int(settings['kernel_px']) | 1)
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (self.kernel_px, self.kernel_px))
        # ranges[color] is a list of [lower3, upper3] bands, OR-ed together.
        # Red wraps the hue origin and so carries two bands; the others one.
        self.ranges = {}
        for color in self.colors:
            flat_lower = settings[color]['lower']
            flat_upper = settings[color]['upper']
            self.ranges[color] = [
                [list(flat_lower[i:i + 3]), list(flat_upper[i:i + 3])]
                for i in range(0, len(flat_lower), 3)
            ]

    def _settings_dict(self):
        """Current live settings, in the same flat shape as the YAML."""
        settings = {'min_area_px': self.min_area, 'kernel_px': self.kernel_px,
                    'colors': list(self.colors)}
        for color, bands in self.ranges.items():
            settings[color] = {
                'lower': [v for band in bands for v in band[0]],
                'upper': [v for band in bands for v in band[1]],
            }
        return settings

    def _load_tuned(self):
        """Overlay the tuned JSON on the YAML baseline, if the file exists.

        Precedence is deliberately one-way and logged: the YAML is the
        committed truth, the JSON is a tuning override. Saying which one is
        live keeps a forgotten JSON from silently shadowing the YAML.
        """
        if not self.tuned_path.is_file():
            self.get_logger().info(
                f'HSV bounds from config/color_detect.yaml '
                f'(no tuned file at {self.tuned_path})')
            return
        try:
            with self.tuned_path.open() as handle:
                settings = json.load(handle)
            self._apply_settings(self._merge_settings(self.baseline, settings))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.get_logger().error(
                f'ignoring unreadable tuned file {self.tuned_path}: {exc}; '
                'using config/color_detect.yaml')
            return
        self.get_logger().warn(
            f'HSV bounds OVERRIDDEN by {self.tuned_path} '
            '(delete it to go back to config/color_detect.yaml)')

    def _save_tuned(self):
        try:
            self.tuned_path.parent.mkdir(parents=True, exist_ok=True)
            with self.tuned_path.open('w') as handle:
                json.dump(self._settings_dict(), handle, indent=2)
                handle.write('\n')
        except OSError as exc:
            self.get_logger().error(f'could not save {self.tuned_path}: {exc}')
            return
        self.get_logger().info(f'saved {self.tuned_path}')

    def _yaml_block(self):
        """The current settings as a paste-ready config/color_detect.yaml body.

        The JSON is for iterating; this is how a value that survives tuning
        gets back into the file that is actually committed.
        """
        settings = self._settings_dict()
        lines = ['color_detect:', '  ros__parameters:',
                 f'    min_area_px: {settings["min_area_px"]}',
                 f'    kernel_px: {settings["kernel_px"]}',
                 f'    colors: {self.colors!r}'.replace('"', "'")]
        for color in self.colors:
            lines.append(f'    {color}:')
            lines.append(f'      lower: {settings[color]["lower"]}')
            lines.append(f'      upper: {settings[color]["upper"]}')
        return '\n'.join(lines)

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
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        height, width = frame.shape[:2]

        with self._lock:
            ranges = deepcopy(self.ranges)
            preview = deepcopy(self._preview)
            min_area = self.min_area
            kernel = self.kernel

        masks = {}
        if preview is not None:
            # The candidate colour on the tuner's "+ new" slot. Masked so the
            # sliders show their effect before the colour exists, but never
            # published: it has no name yet, so no class_name to publish under.
            masks[PREVIEW_KEY] = self._mask_for(hsv, [preview], kernel)

        result = ObjectsInfo()
        for color, bands in ranges.items():
            overlay = draw_bgr(color, bands)
            mask = self._mask_for(hsv, bands, kernel)
            masks[color] = mask

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            survivors = 0
            for contour in contours:
                if cv2.contourArea(contour) < min_area:
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

                cv2.rectangle(frame, (x, y), (x + w, y + h), overlay, 2)
                cv2.putText(frame, color, (x, max(0, y - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, overlay, 1)

            if survivors > 1:
                # Expected: the world has a same-coloured decorative cube
                # behind each graspable one (see config/color_detect.yaml).
                # This node is intentionally 2D-only and does not pick a
                # winner; whoever consumes /yolo/object_detect must select
                # among same-colour boxes (e.g. largest-area + reachability).
                self.get_logger().warn(
                    f'{survivors} {color} blobs above min_area_px '
                    '(expected: the graspable cube plus its same-colour '
                    'decorative twin further away)',
                    throttle_duration_sec=5.0)

        self.objects_pub.publish(result)
        self.image_pub.publish(self._to_image_msg(frame, msg.header))

        if self.tune:
            with self._lock:
                self._last_frame = frame
                self._last_masks = masks

    @staticmethod
    def _mask_for(hsv, bands, kernel):
        """OR the bands together, then open and close."""
        mask = None
        for lower, upper in bands:
            matched = cv2.inRange(hsv,
                                  np.array(lower, dtype=np.uint8),
                                  np.array(upper, dtype=np.uint8))
            mask = matched if mask is None else cv2.bitwise_or(mask, matched)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    def _to_image_msg(self, frame, header):
        # Built by hand instead of cv_bridge.cv2_to_imgmsg: on this dev
        # machine a pip-installed opencv-python (5.0.0) shadows the apt
        # OpenCV that cv_bridge's C++ extension was compiled against, so
        # cv2_to_imgmsg raises KeyError: 16 while looking up the encoding's
        # numpy dtype from the wrong module's constants. imgmsg_to_cv2 (used
        # above, for the incoming image) is unaffected. A contiguous bgr8
        # frame needs no cv_bridge machinery to serialize, so build it here
        # instead of depending on the shadowed cv2 import resolving right.
        img_msg = Image()
        img_msg.header = header
        img_msg.height = frame.shape[0]
        img_msg.width = frame.shape[1]
        img_msg.encoding = 'bgr8'
        img_msg.is_bigendian = 0
        img_msg.step = frame.shape[1] * 3
        img_msg.data = np.ascontiguousarray(frame).tobytes()
        return img_msg

    # ---------------------------------------------------------------- tuner

    def run_tuner(self):
        """Trackbar UI. Runs on the main thread; highgui requires that.

        Detection keeps running and publishing throughout, so the effect of a
        slider is visible on /yolo/object_detect as well as in the window.

        Sliding "colour" one past the last colour lands on the "+ new" slot
        and starts naming a new colour; n does the same from anywhere. Either
        way, type the name, enter to confirm, esc to cancel.

          n  add a new colour (same as sliding onto the "+ new" slot)
          s  save the current bounds to the tuned JSON
          r  revert to the config/color_detect.yaml baseline
          y  print the current bounds as a paste-ready YAML block
          q  close the window; the node keeps detecting
        """
        self._warn_if_qt_has_no_fonts()
        cv2.namedWindow(TUNE_WINDOW, cv2.WINDOW_NORMAL)
        # Without an explicit size the window opens too small to show the
        # 1280-wide frame+mask panel, leaving only the trackbar gutter visible.
        cv2.resizeWindow(TUNE_WINDOW, 1280, 620)
        noop = (lambda _value: None)
        # One slot past the last colour is "+ new": sliding onto it starts
        # naming a new colour, so the selector doubles as the way to add one.
        cv2.createTrackbar('colour', TUNE_WINDOW, 0, len(self.colors), noop)
        cv2.createTrackbar('band', TUNE_WINDOW, 0, 1, noop)
        for name, limit in zip(BAND_BARS, BAND_MAX):
            cv2.createTrackbar(name, TUNE_WINDOW, 0, limit, noop)
        cv2.createTrackbar('min_area', TUNE_WINDOW, self.min_area, 30000, noop)
        cv2.createTrackbar('kernel', TUNE_WINDOW, self.kernel_px, 15, noop)

        selection = None     # (colour index, band index) the sliders last held
        naming = None        # the name being typed, or None when not naming
        index = 0            # last real colour the selector was on
        self.get_logger().info(
            'tuning window open: slide "colour" past the last colour (or '
            f'press n) to add one, s=save to {self.tuned_path}, '
            'r=revert to YAML, y=print YAML, q=close')

        while rclpy.ok():
            if cv2.getWindowProperty(TUNE_WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break

            # The "+ new" slot IS the naming mode: the selector's position is
            # the only state, so sliding off it cancels, exactly as sliding
            # onto it started. Anything else leaves the two able to disagree.
            position = cv2.getTrackbarPos('colour', TUNE_WINDOW)
            adding = position >= len(self.colors)
            if adding:
                if naming is None:
                    naming = ''
                    # Open the bounds right up so the preview mask starts by
                    # showing everything, then gets narrowed onto the target.
                    for name, value in zip(BAND_BARS,
                                           (PREVIEW_SEED[0][0],
                                            PREVIEW_SEED[1][0],
                                            PREVIEW_SEED[0][1],
                                            PREVIEW_SEED[1][1],
                                            PREVIEW_SEED[0][2],
                                            PREVIEW_SEED[1][2])):
                        cv2.setTrackbarPos(name, TUNE_WINDOW, value)
            else:
                if naming is not None:
                    self.get_logger().info('new colour cancelled')
                    naming = None
                    # The bounds sliders were driving the preview, not this
                    # colour; resync them from what it actually holds.
                    selection = None
                index = position

            color = self.colors[index]
            bands = self.ranges[color]
            band = min(cv2.getTrackbarPos('band', TUNE_WINDOW), len(bands) - 1)

            # min_area and kernel belong to the detector, not to a colour, so
            # they stay live even while a new colour is being named.
            self._globals_from_sliders()

            if adding:
                # The bounds sliders drive the preview mask instead of a
                # colour, which is what makes them visibly do something here.
                with self._lock:
                    self._preview = self._read_band_sliders()
            elif selection != (color, band):
                # Switched band: push its stored values out to the sliders
                # instead of writing the previous band's values into it.
                self._sliders_from_state(color, band)
                selection = (color, band)
                with self._lock:
                    self._preview = None
            else:
                self._band_from_sliders(color, band)

            self._show_tuner(color, band, naming)
            key = cv2.waitKey(30) & 0xFF

            if adding:
                if key in (13, 10):                       # enter
                    if self._add_color(naming, self._read_band_sliders()):
                        index = len(self.colors) - 1
                        # Grow the selector before moving it: setTrackbarPos
                        # clamps to the current maximum. The "+ new" slot
                        # shifts up with it.
                        cv2.setTrackbarMax('colour', TUNE_WINDOW,
                                           len(self.colors))
                        cv2.setTrackbarPos('colour', TUNE_WINDOW, index)
                        selection = None
                        naming = None
                    # A rejected name is kept in the buffer so it can be
                    # corrected rather than retyped from scratch.
                elif key == 27:                           # esc
                    # Just step off the slot; the top of the loop does the
                    # cancelling, so there is one path out, not two.
                    cv2.setTrackbarPos('colour', TUNE_WINDOW, index)
                elif key in (8, 127):                     # backspace
                    naming = naming[:-1]
                elif key < 128 and chr(key) in NAME_CHARS and len(naming) < 24:
                    naming += chr(key)
                continue

            if key == ord('n'):
                # Same thing the slider does, without having to drag.
                cv2.setTrackbarPos('colour', TUNE_WINDOW, len(self.colors))
            elif key == ord('s'):
                self._save_tuned()
            elif key == ord('r'):
                with self._lock:
                    self._apply_settings(deepcopy(self.baseline))
                cv2.setTrackbarPos('min_area', TUNE_WINDOW, self.min_area)
                cv2.setTrackbarPos('kernel', TUNE_WINDOW, self.kernel_px)
                # Reverting drops any colour added since startup, so the
                # selector has to shrink back with it.
                cv2.setTrackbarMax('colour', TUNE_WINDOW, len(self.colors))
                cv2.setTrackbarPos('colour', TUNE_WINDOW, 0)
                index = 0
                selection = None
                self.get_logger().info(
                    'reverted to the config/color_detect.yaml baseline')
            elif key == ord('y'):
                self.get_logger().info(
                    'current bounds as YAML:\n' + self._yaml_block())
            elif key == ord('q'):
                break

        with self._lock:
            # Stop masking a candidate nobody is looking at any more.
            self._preview = None
        cv2.destroyWindow(TUNE_WINDOW)
        self.get_logger().info('tuning window closed; still detecting')

    def _add_color(self, name, band):
        """Register a new detection class from the tuner.

        The name becomes ObjectInfo.class_name on /yolo/object_detect and a key
        in the YAML block 'y' prints, so it is restricted to the characters
        both of those can carry without quoting.

        Returns True when the colour was added.
        """
        name = name.strip()
        if not name:
            self.get_logger().warn('a new colour needs a name; nothing added')
            return False
        if name in self.colors:
            self.get_logger().warn(f'{name!r} already exists; nothing added')
            return False
        with self._lock:
            self.colors.append(name)
            # Keeps whatever the sliders were showing, so the mask you tuned
            # on the "+ new" slot is the mask the colour starts with. One band:
            # only reds wrap the hue origin and red already exists, so a
            # second band is left to the YAML.
            self.ranges[name] = [deepcopy(band)]
        self.get_logger().info(
            f'added colour {name!r}: it publishes as class_name {name!r} on '
            '/yolo/object_detect. Narrow the sliders, then press s to keep it '
            '(or y to print it for config/color_detect.yaml).')
        return True

    def _warn_if_qt_has_no_fonts(self):
        """Say how to fix blank trackbar labels, rather than leaving a puzzle.

        A stock pip opencv-python ships cv2/qt/ without a fonts/ directory. Its
        Qt then has no font at all and draws every trackbar label as an empty
        strip, which makes ten unlabelled sliders. QT_QPA_FONTDIR does not help
        this build; supplying one font file does. The readout drawn into the
        image panel stays correct either way, so this is a warning, not a
        failure.
        """
        fonts = Path(cv2.__file__).parent / 'qt' / 'fonts'
        if fonts.is_dir() and any(fonts.iterdir()):
            return
        self.get_logger().warn(
            'this OpenCV build has no Qt fonts, so the trackbar labels will '
            'be blank. The slider order is: colour, band, H lo, H hi, S lo, '
            'S hi, V lo, V hi, min_area, kernel -- and the readout along the '
            'bottom of the image always shows the real values. To label them, '
            f'run: mkdir -p {fonts} && cp '
            '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf ' + str(fonts))

    def _sliders_from_state(self, color, band):
        """Push stored values out to every slider, bounds and globals alike.

        min_area and kernel are included on purpose. They belong to the whole
        detector, not to a colour, and leaving them out let a value dragged
        while they were inert survive to be written back later -- which is how
        min_area once reached 30000 and silently suppressed every detection.
        """
        with self._lock:
            lower, upper = self.ranges[color][band]
            min_area, kernel_px = self.min_area, self.kernel_px
        for name, value in zip(BAND_BARS, (lower[0], upper[0], lower[1],
                                           upper[1], lower[2], upper[2])):
            cv2.setTrackbarPos(name, TUNE_WINDOW, int(value))
        cv2.setTrackbarPos('min_area', TUNE_WINDOW, int(min_area))
        cv2.setTrackbarPos('kernel', TUNE_WINDOW, int(kernel_px))

    def _read_band_sliders(self):
        """The six HSV sliders as a [lower, upper] pair."""
        values = [cv2.getTrackbarPos(name, TUNE_WINDOW) for name in BAND_BARS]
        return [[values[0], values[2], values[4]],
                [values[1], values[3], values[5]]]

    def _globals_from_sliders(self):
        """min_area and kernel, read every frame.

        These are detector-wide, so unlike the HSV bounds they stay live even
        on the "+ new" slot where no colour is selected.
        """
        with self._lock:
            self.min_area = cv2.getTrackbarPos('min_area', TUNE_WINDOW)
            kernel_px = max(1, cv2.getTrackbarPos('kernel', TUNE_WINDOW) | 1)
            if kernel_px != self.kernel_px:
                self.kernel_px = kernel_px
                self.kernel = cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE, (kernel_px, kernel_px))

    def _band_from_sliders(self, color, band):
        lower_upper = self._read_band_sliders()
        with self._lock:
            self.ranges[color][band] = lower_upper

    def _show_tuner(self, color, band, naming=None):
        adding = naming is not None
        with self._lock:
            frame = self._last_frame
            bands = deepcopy(self.ranges[color])
            # On the "+ new" slot no colour is selected, so the panel shows
            # the candidate's preview mask -- showing the last colour's mask
            # and labelling it as the new one would be a lie.
            mask = self._last_masks.get(PREVIEW_KEY if adding else color)
            lower, upper = (deepcopy(self._preview) if adding and self._preview
                            else bands[band])
            min_area, kernel_px = self.min_area, self.kernel_px
        if adding:
            label, label_bgr = '+ new colour', (255, 255, 255)
            prefix = f'new colour name: {naming}_  [enter=add esc=cancel]'
        else:
            label, label_bgr = f'{color} mask', draw_bgr(color, bands)
            prefix = f'{color} band {band + 1}/{len(bands)}'
        readout = (f'{prefix}  H {lower[0]}-{upper[0]}  '
                   f'S {lower[1]}-{upper[1]}  V {lower[2]}-{upper[2]}  '
                   f'area>{min_area}  k={kernel_px}')
        if frame is None or mask is None:
            # The camera is bridged lazily and renders only while something
            # subscribes, so the first frames after startup can be missing.
            waiting = np.zeros((240, 640, 3), dtype=np.uint8)
            cv2.putText(waiting, 'waiting for /depth_cam/rgb/image_raw',
                        (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (255, 255, 255), 1)
            cv2.imshow(TUNE_WINDOW, waiting)
            return
        panel = np.hstack((frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))
        cv2.putText(panel, label, (frame.shape[1] + 10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, label_bgr, 2)
        # The readout is drawn into the image rather than left to the trackbar
        # labels: highgui's labels depend on Qt finding fonts, and they render
        # blank on a stock pip opencv-python. This always works, and it also
        # shows the exact values the sliders are only approximating.
        cv2.rectangle(panel, (0, panel.shape[0] - 30),
                      (panel.shape[1], panel.shape[0]), (0, 0, 0), -1)
        cv2.putText(panel, readout, (10, panel.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.imshow(TUNE_WINDOW, panel)


def main():
    rclpy.init()
    node = ColorDetectNode()
    try:
        if node.tune:
            # highgui must own the main thread, so spin moves to a worker.
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


if __name__ == '__main__':
    main()
