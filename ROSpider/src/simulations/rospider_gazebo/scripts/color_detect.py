#!/usr/bin/env python3
"""LAB cube detector for the simulation.

Publishes interfaces/ObjectsInfo on /yolo/object_detect, the same topic and
message competition/yolo_node.py uses on the real robot, so a YOLO node can
replace this one without the pick node changing.

Thresholding is in LAB with one min/max band per colour, the shape of the
real robot's lab_config.yaml, and the drawn ~/image_result follows
example/opencv_example/include/color_detect_node.py: one circle (or rotated
box) round the largest blob in Hiwonder's range_rgb colours.

With tune:=true the node also opens a Tk window laid out like Hiwonder's
LAB_Tool 1.0; see run_tuner(). The two-file precedence rule between
config/color_detect.yaml and the tuned JSON is in rospider_gazebo/lab_settings.py.
"""

import json
import os
import threading
import tkinter as tk
from pathlib import Path
from tkinter import simpledialog, ttk

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from interfaces.msg import ObjectInfo, ObjectsInfo
from rclpy.node import Node
from rospider_gazebo import lab_settings, tkview
from rospider_gazebo.ros_image import to_image_msg
from sensor_msgs.msg import Image

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

        # Shared with the window: the last raw frame and per-colour masks.
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
        """Read a parameter, falling back when the YAML does not carry it.

        The node declares parameters from overrides, so anything absent from
        config/color_detect.yaml comes back with value None rather than raising.
        """
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
        """Overlay the tuned JSON on the YAML baseline, if the file exists.

        Precedence is deliberately one-way and logged: the YAML is the
        committed truth, the JSON is a tuning override. Saying which one is
        live keeps a forgotten JSON from silently shadowing the YAML.
        """
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
        """Write the live settings to the tuned JSON; returns a status line.

        The JSON is for iterating; the YAML block logged alongside is how a
        value that survives tuning gets back into the committed file.
        """
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
                frame_label.image = photo      # keep it alive
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


def main():
    rclpy.init()
    node = ColorDetectNode()
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


if __name__ == '__main__':
    main()
