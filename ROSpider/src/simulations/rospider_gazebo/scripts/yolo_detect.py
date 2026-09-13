#!/usr/bin/env python3
"""YOLO object detector for the simulation.

A drop-in replacement for color_detect.py: same message, same topic, same
image_result. pick_and_place.py needs no change to consume it -- its
_box_centroid_and_area already reads both the 4-number axis-aligned box and
the 8-number oriented box an OBB model produces.

The two detectors are alternatives, never both at once:

    ros2 launch rospider_gazebo pick_place.launch.py detector:=yolo

ultralytics and torch are imported inside __init__, not at module scope, and
are not declared in package.xml. The colour path, colcon build and every test
must all work on a machine that has never installed them, so a missing torch
has to produce one clear sentence rather than an import traceback at launch.
"""

import logging
import os
import queue
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from interfaces.msg import ObjectInfo, ObjectsInfo
from rclpy.node import Node
from rospider_gazebo.ros_image import to_image_msg
from sensor_msgs.msg import Image


def _package_dir():
    """Where a relative model_path is resolved from.

    The package share directory, not this file's parent: in the install space
    scripts land in lib/rospider_gazebo while CMakeLists installs models/ to
    share/rospider_gazebo, so walking up from __file__ lands in the wrong
    tree. With --symlink-install the share copy points back at the source, so
    a model trained into models/yolo/ is picked up without rebuilding.
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
        self.conf = float(self._param('conf', 0.5))
        self.device = str(self._param('device', ''))
        self.draw = bool(self._param('draw', True))
        allow = self._param('classes', [])
        self.allow = set(allow) if allow else None
        image_topic = str(self._param('image_topic',
                                      '/depth_cam/rgb/image_raw'))

        self.model = _load_yolo(_resolve_model(model_path), self.task)

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

    def _param(self, name, default):
        """Read a parameter, falling back when the YAML does not carry it.

        Parameters are declared from overrides, so anything absent from
        config/yolo.yaml comes back as None. Same helper, same reason, as
        color_detect.py.
        """
        value = self.get_parameter(name).value
        return default if value is None else value

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
        kwargs = {'conf': self.conf, 'verbose': False}
        if self.device:
            kwargs['device'] = self.device
        result = self.model(frame, **kwargs)[0]

        objects = ObjectsInfo()
        for name, box, score in self._detections(result):
            if self.allow is not None and name not in self.allow:
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


def main():
    rclpy.init()
    node = YoloDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
