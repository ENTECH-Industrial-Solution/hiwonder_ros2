#!/usr/bin/env python3
"""YOLO object detector for the simulation.

A drop-in replacement for color_detect.py: same message, same topic, same
image_result. pick_and_place.py needs no change to consume it -- its
_box_centroid_and_area already reads both the 4-number axis-aligned box and
the 8-number oriented box an OBB model produces.

The two detectors are alternatives, never both at once:

    ros2 launch rospider_gazebo pick_place.launch.py detector:=yolo

With tune:=true the node opens a Tk window (rospider_gazebo/tkview.py, the
same helpers apriltag_detect uses): the live detections, a confidence
slider and one checkbox per class the model knows. Save writes the two
values to ~/.ros/yolo_detect_tuned.json, which overrides config/yolo.yaml on
the next launch the way color_detect's and apriltag_detect's tuned files do.

ultralytics and torch are imported inside __init__, not at module scope, and
are not declared in package.xml. The colour path, colcon build and every test
must all work on a machine that has never installed them, so a missing torch
has to produce one clear sentence rather than an import traceback at launch.
"""

import json
import logging
import os
import queue
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from interfaces.msg import ObjectInfo, ObjectsInfo
from rclpy.node import Node
from rospider_gazebo import tkview, yolo_settings
from rospider_gazebo.ros_image import to_image_msg
from sensor_msgs.msg import Image

TUNE_TITLE = 'yolo_detect tune'


def _package_dir():
    """Where a relative model_path is resolved from.

    The package share directory, not this file's parent: in the install space
    scripts land in lib/rospider_gazebo while CMakeLists installs models/ to
    share/rospider_gazebo, so walking up from __file__ lands in the wrong
    tree.

    Note that --symlink-install links the files that existed at build time,
    not the directory. A model trained into a models/yolo/ that did not exist
    at the last build is NOT visible until `colcon build` is run again -- the
    node then reports the share path as missing, which is confusing until you
    know this.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('rospider_gazebo')
    except Exception:
        # Running the script straight out of the source tree, uninstalled.
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _resolve_model(model_path):
    """Absolute path to the weights, or the bare name for one ultralytics can
    fetch itself (yolo11n.pt and friends).

    A relative path is taken as relative to the package, so config/yolo.yaml
    can say `models/yolo/cubes.pt` and mean the file in this repository rather
    than something relative to whatever directory the launch happened from.
    """
    if os.path.isabs(model_path):
        return model_path
    candidate = os.path.join(_package_dir(), model_path)
    if os.path.exists(candidate):
        return candidate
    if os.sep in model_path:
        # It names a path in this package, and that path is not there.
        raise FileNotFoundError(
            f'{candidate} does not exist. The repository does not ship a '
            'trained model yet: capture a dataset and train one with\n'
            '  python3 tools/capture_dataset.py --samples 500 --out '
            '~/datasets/cubes\n'
            '  python3 tools/train_yolo.py --data ~/datasets/cubes '
            '--name cubes\n'
            'or point model_path at a stock model such as yolo11n.pt, which '
            'ultralytics downloads on first use.')
    # A bare name: let ultralytics fetch it.
    return model_path


def _load_yolo(model_path, task):
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError(
            'detector:=yolo needs ultralytics and torch, which this package '
            'deliberately does not declare as dependencies. Install them '
            'with:  pip install --user --break-system-packages -r '
            'requirements-yolo.txt  -- see that file for the torch index URL '
            'this GPU needs.') from exc
    logging.getLogger('ultralytics').setLevel(logging.WARNING)
    return YOLO(model_path, task=task)


class YoloDetectNode(Node):

    def __init__(self):
        super().__init__('yolo_detect',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.bridge = CvBridge()

        model_path = str(self._param('model_path', ''))
        if not model_path:
            raise ValueError('yolo_detect needs a model_path; see '
                             'config/yolo.yaml')
        self.task = str(self._param('task', 'detect'))
        self.device = str(self._param('device', ''))
        self.draw = bool(self._param('draw', True))
        self.tune = bool(self._param('tune', False))
        self.tuned_path = Path(os.path.expanduser(
            str(self._param('tuned_path', '~/.ros/yolo_detect_tuned.json'))))
        image_topic = str(self._param('image_topic',
                                      '/depth_cam/rgb/image_raw'))

        # Shared with the tuner thread: it reads the last drawn frame and
        # the counters, and writes conf/classes through apply_settings().
        self._lock = threading.Lock()
        self.latest_frame = None
        self.last_count = 0
        self.last_ms = 0.0
        self.baseline = yolo_settings.merge(yolo_settings.defaults(), {
            'conf': self._param('conf', 0.5),
            'classes': self._param('classes', []) or []})
        self.apply_settings(json.loads(json.dumps(self.baseline)))
        self._load_tuned()

        self.model = _load_yolo(_resolve_model(model_path), self.task)
        self._warm_up()

        # maxsize 2, and a full queue drops the frame rather than blocking.
        # Inference is slower than the 15 Hz camera, and a growing backlog
        # would feed pick_and_place a steadily staler view of the world. Same
        # shape as upstream's competition/yolo_node.py:35.
        self.frames = queue.Queue(maxsize=2)
        self.running = True

        self.objects_pub = self.create_publisher(
            ObjectsInfo, '/yolo/object_detect', 1)
        self.image_pub = self.create_publisher(Image, '~/image_result', 1)
        self.create_subscription(Image, image_topic, self.image_callback, 1)

        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()
        self.get_logger().info(
            f'{model_path} ({self.task}) on {image_topic}, '
            f'classes {sorted(self.model.names.values())}')

    def _warm_up(self):
        """Run one inference on a blank frame before subscribing to anything.

        The first inference compiles CUDA kernels and costs about 10 s on this
        machine, against 19 ms for every one after it. Without this the node
        advertises, starts receiving frames, and produces nothing for ten
        seconds -- and pick_and_place's LOOK state gives up on a colour after
        state_timeout, 15 s. That is exactly how a run with auto_start logged
        "never saw red; skipping" while the same model, probed a minute later
        on the same scene, detected red at 0.946 confidence.

        Paying it here means the "ready" log line below is true when it is
        printed.
        """
        started = time.monotonic()
        blank = np.zeros((480, 640, 3), dtype=np.uint8)
        kwargs = {'conf': self.conf, 'verbose': False}
        if self.device:
            kwargs['device'] = self.device
        self.model(blank, **kwargs)
        self.get_logger().info(
            f'warmed up in {time.monotonic() - started:.1f}s')

    def _param(self, name, default):
        """Read a parameter, falling back when the YAML does not carry it.

        Parameters are declared from overrides, so anything absent from
        config/yolo.yaml comes back as None. Same helper, same reason, as
        color_detect.py.
        """
        value = self.get_parameter(name).value
        return default if value is None else value

    # ------------------------------------------------------------- settings

    def apply_settings(self, settings):
        """Adopt conf and the class filter as the live values."""
        with self._lock:
            self.settings = settings
            self.conf = float(settings['conf'])
            allow = settings['classes']
            self.allow = set(allow) if allow else None

    def _load_tuned(self):
        """Overlay the tuned JSON on the YAML baseline, if the file exists.

        One-way and logged, as in color_detect: the YAML is the committed
        truth, the JSON a tuning override, and saying which is live keeps a
        forgotten JSON from silently shadowing the YAML.
        """
        if not self.tuned_path.is_file():
            self.get_logger().info(
                f'settings from config/yolo.yaml '
                f'(no tuned file at {self.tuned_path})')
            return
        try:
            with self.tuned_path.open() as handle:
                override = json.load(handle)
            self.apply_settings(yolo_settings.merge(self.baseline, override))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.get_logger().error(
                f'ignoring unreadable tuned file {self.tuned_path}: {exc}; '
                'using config/yolo.yaml')
            return
        self.get_logger().warn(
            f'settings OVERRIDDEN by {self.tuned_path} '
            '(delete it to go back to config/yolo.yaml)')

    def save_tuned(self):
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
        self.get_logger().info('reverted to the config/yolo.yaml baseline')

    def yaml_block(self):
        with self._lock:
            return yolo_settings.yaml_block(self.settings)

    def image_callback(self, msg):
        try:
            self.frames.put_nowait(msg)
        except queue.Full:
            pass

    def _run(self):
        while self.running:
            try:
                msg = self.frames.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._process_image(msg)
            except Exception:
                self.get_logger().error(
                    'inference failed on this frame; skipping it',
                    exc_info=True)

    def _process_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        with self._lock:
            conf, allow = self.conf, self.allow
        kwargs = {'conf': conf, 'verbose': False}
        if self.device:
            kwargs['device'] = self.device
        started = time.monotonic()
        result = self.model(frame, **kwargs)[0]
        elapsed_ms = (time.monotonic() - started) * 1000.0

        objects = ObjectsInfo()
        for name, box, score in self._detections(result):
            if allow is not None and name not in allow:
                continue
            info = ObjectInfo()
            info.class_name = name
            info.box = [int(round(v)) for v in box]
            info.score = float(score)
            info.width = int(frame.shape[1])
            info.height = int(frame.shape[0])
            objects.objects.append(info)
            if self.draw:
                self._draw(frame, name, info.box, score)

        # Published even when empty, so a consumer can tell "looking, saw
        # nothing" from "the node is dead". Matches color_detect.py.
        self.objects_pub.publish(objects)
        if self.draw:
            self.image_pub.publish(to_image_msg(frame, msg.header))
        with self._lock:
            self.latest_frame = frame
            self.last_count = len(objects.objects)
            self.last_ms = elapsed_ms

    def _detections(self, result):
        """(class_name, box, score) per detection.

        An OBB model yields the four corners as eight numbers, which is what
        upstream's yolo_node.py publishes and what pick_and_place.py's
        _box_centroid_and_area averages. A plain detect model yields the
        axis-aligned [x1, y1, x2, y2]. Both are legal ObjectInfo.box values.
        """
        obb = getattr(result, 'obb', None)
        if self.task == 'obb' and obb is not None and len(obb):
            for corners, cls, conf in zip(obb.xyxyxyxy.cpu().numpy(),
                                          obb.cls.cpu().numpy(),
                                          obb.conf.cpu().numpy()):
                yield (self.model.names[int(cls)],
                       np.asarray(corners).reshape(-1).tolist(),
                       float(conf))
            return
        if result.boxes is None:
            return
        for xyxy, cls, conf in zip(result.boxes.xyxy.cpu().numpy(),
                                   result.boxes.cls.cpu().numpy(),
                                   result.boxes.conf.cpu().numpy()):
            yield self.model.names[int(cls)], xyxy.tolist(), float(conf)

    @staticmethod
    def _draw(frame, name, box, score):
        points = np.array(box, dtype=np.int32).reshape(-1, 2)
        if len(points) == 2:
            points = np.array([[box[0], box[1]], [box[2], box[1]],
                               [box[2], box[3]], [box[0], box[3]]],
                              dtype=np.int32)
        cv2.polylines(frame, [points], True, (0, 255, 0), 2)
        cv2.putText(frame, f'{name} {score:.2f}',
                    (int(points[:, 0].min()), int(points[:, 1].min()) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    def destroy_node(self):
        self.running = False
        self.worker.join(timeout=1.0)
        super().destroy_node()

    # ---------------------------------------------------------------- tuner

    def run_tuner(self):
        """Tk window. Runs on the main thread; Tk requires that.

        Deliberately small -- a workshop needs "how sure must the model be"
        and "which classes may the picker see", nothing else. Detection
        keeps running throughout, so a change shows on the overlay and on
        /yolo/object_detect at once. Closing the window leaves the node
        running with the last values.
        """
        root = tk.Tk()
        root.title(TUNE_TITLE)

        image_label = ttk.Label(root)
        image_label.grid(row=0, column=0, padx=6, pady=6, sticky='n')
        side = ttk.Frame(root)
        side.grid(row=0, column=1, padx=6, pady=6, sticky='n')

        with self._lock:
            settings = json.loads(json.dumps(self.settings))
        names = sorted(self.model.names.values())
        message = tk.StringVar()

        conf = tk.DoubleVar(value=float(settings['conf']))
        tkview.LabeledScale(side, 'confidence', conf, 0.05, 0.95, 0.05,
                            lambda: apply()).grid(row=0, column=0,
                                                  sticky='ew')

        # One checkbox per class the model knows. All ticked means the
        # filter is off (classes: [] -> publish everything), which is what
        # the YAML default says.
        classes_box = ttk.LabelFrame(side, text='classes to publish')
        classes_box.grid(row=1, column=0, sticky='ew', pady=(6, 0))
        ticked = {}
        allowed = set(settings['classes']) or set(names)
        for i, name in enumerate(names):
            var = tk.BooleanVar(value=name in allowed)
            ticked[name] = var
            ttk.Checkbutton(classes_box, text=name, variable=var,
                            command=lambda: apply()).grid(
                row=i, column=0, sticky='w')

        status = ttk.Label(side, text='', wraplength=260, justify='left')
        status.grid(row=2, column=0, sticky='w', pady=(6, 0))

        def read_widgets():
            chosen = [name for name in names if ticked[name].get()]
            return yolo_settings.merge(yolo_settings.defaults(), {
                'conf': conf.get(),
                # Everything ticked is "no filter", not a list of all
                # names: a model swapped later would otherwise be filtered
                # down to this model's classes.
                'classes': [] if len(chosen) == len(names) else chosen})

        def apply():
            try:
                self.apply_settings(read_widgets())
            except (ValueError, KeyError, tk.TclError) as exc:
                message.set(f'not applied: {exc}')
                return
            message.set('')

        def do_revert():
            self.revert()
            with self._lock:
                base = json.loads(json.dumps(self.settings))
            conf.set(base['conf'])
            allowed = set(base['classes']) or set(names)
            for name, var in ticked.items():
                var.set(name in allowed)
            message.set('reverted to config/yolo.yaml')

        def do_yaml():
            self.get_logger().info('current settings as YAML:\n'
                                   + self.yaml_block())
            message.set('YAML printed to the terminal')

        buttons = ttk.Frame(side)
        buttons.grid(row=3, column=0, sticky='ew', pady=(6, 0))
        ttk.Button(buttons, text='Save',
                   command=lambda: message.set(self.save_tuned())).pack(
            side='left')
        ttk.Button(buttons, text='Revert', command=do_revert).pack(side='left')
        ttk.Button(buttons, text='Print YAML', command=do_yaml).pack(side='left')
        ttk.Label(side, textvariable=message, wraplength=260).grid(
            row=4, column=0, sticky='w')

        def refresh():
            with self._lock:
                frame = self.latest_frame
                count, ms = self.last_count, self.last_ms
                live_conf = self.conf
                allow = self.allow
            if frame is not None:
                photo = tkview.photo_from_bgr(frame)
                image_label.configure(image=photo)
                image_label.image = photo      # keep it alive
            shown = 'all classes' if allow is None else ', '.join(sorted(allow))
            status.configure(
                text=f'{count} object(s), {ms:.0f} ms, conf >= {live_conf:.2f}'
                     f'\npublishing: {shown}')
            root.after(50, refresh)
        refresh()

        self.get_logger().info(
            f'tuning window open: Save writes {self.tuned_path}, Revert '
            'reloads config/yolo.yaml, Print YAML logs a paste-ready block')
        tkview.mainloop_until_shutdown(root)
        self.get_logger().info('tuning window closed; still detecting')


def main():
    rclpy.init()
    node = YoloDetectNode()
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
