#!/usr/bin/env python3
"""HSV cube detector for the simulation.

Publishes interfaces/ObjectsInfo on /yolo/object_detect, the same topic and
message competition/yolo_node.py uses on the real robot, so a YOLO node can
replace this one without the pick node changing.
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from interfaces.msg import ObjectInfo, ObjectsInfo
from rclpy.node import Node
from sensor_msgs.msg import Image

DRAW_BGR = {'red': (0, 0, 255), 'green': (0, 255, 0), 'blue': (255, 0, 0)}


class ColorDetectNode(Node):

    def __init__(self):
        super().__init__('color_detect',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.bridge = CvBridge()
        self.min_area = int(self.get_parameter('min_area_px').value)
        kernel_px = int(self.get_parameter('kernel_px').value)
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_px, kernel_px))

        self.ranges = {}
        for color in self.get_parameter('colors').value:
            lower = list(self.get_parameter(f'{color}.lower').value)
            upper = list(self.get_parameter(f'{color}.upper').value)
            self.ranges[color] = [
                (np.array(lower[i:i + 3], dtype=np.uint8),
                 np.array(upper[i:i + 3], dtype=np.uint8))
                for i in range(0, len(lower), 3)
            ]

        self.objects_pub = self.create_publisher(
            ObjectsInfo, '/yolo/object_detect', 1)
        self.image_pub = self.create_publisher(Image, '~/image_result', 1)
        self.create_subscription(
            Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.get_logger().info(
            f'watching for {sorted(self.ranges)} on /depth_cam/rgb/image_raw')

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

        result = ObjectsInfo()
        for color, ranges in self.ranges.items():
            mask = None
            for lower, upper in ranges:
                band = cv2.inRange(hsv, lower, upper)
                mask = band if mask is None else cv2.bitwise_or(mask, band)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            survivors = 0
            for contour in contours:
                if cv2.contourArea(contour) < self.min_area:
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

                cv2.rectangle(frame, (x, y), (x + w, y + h),
                              DRAW_BGR[color], 2)
                cv2.putText(frame, color, (x, max(0, y - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, DRAW_BGR[color], 1)

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


def main():
    rclpy.init()
    node = ColorDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
