#!/usr/bin/env python3
"""Publish a USB webcam, so the MediaPipe demos have a person to look at.

The simulated world has no people in it, and the hand, face and body demos
need one. This node reads the PC's own camera and publishes it on
/webcam/image_raw, which vision_demo.launch.py points those demos at when it
is given `source:=webcam`: the person sits in front of the laptop, and the
simulated robot reacts in Gazebo.

The CameraInfo it publishes alongside is an ideal pinhole guess (focal length
= image width, principal point at the centre, no distortion), not a
calibration. It is there so a demo that wants intrinsics has something
plausible; a pose solved against it is a rough one.

    ros2 run rospider_gazebo webcam_publisher.py --ros-args -p device:=0
"""

import cv2
import rclpy
from rclpy.node import Node
from rospider_gazebo.ros_image import to_image_msg
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header


class WebcamNode(Node):

    def __init__(self):
        super().__init__('webcam_publisher',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.topic = str(self._param('topic', '/webcam/image_raw'))
        self.frame_id = str(self._param('frame_id', 'webcam'))
        device = self._param('device', 0)
        width = int(self._param('width', 640))
        height = int(self._param('height', 480))
        rate = float(self._param('rate', 30.0))

        self.capture = cv2.VideoCapture(int(device) if str(device).isdigit()
                                        else str(device))
        if not self.capture.isOpened():
            raise SystemExit(f'cannot open camera {device!r}')
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self.image_pub = self.create_publisher(Image, self.topic, 1)
        info_topic = self.topic.rsplit('/', 1)[0] + '/camera_info'
        self.info_pub = self.create_publisher(CameraInfo, info_topic, 1)
        self.create_timer(1.0 / rate, self.publish_frame)
        self.get_logger().info(
            f'camera {device} -> {self.topic} at {rate:.0f} Hz')

    def _param(self, name, default):
        value = self.get_parameter(name).value
        return default if value is None else value

    def publish_frame(self):
        ok, frame = self.capture.read()
        if not ok:
            self.get_logger().warning('dropped a frame',
                                      throttle_duration_sec=5.0)
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        self.image_pub.publish(to_image_msg(frame, header, 'bgr8'))
        self.info_pub.publish(self._camera_info(header, frame.shape[1],
                                                frame.shape[0]))

    @staticmethod
    def _camera_info(header, width, height):
        info = CameraInfo()
        info.header = header
        info.width = width
        info.height = height
        focal = float(width)
        info.k = [focal, 0.0, width / 2.0,
                  0.0, focal, height / 2.0,
                  0.0, 0.0, 1.0]
        info.p = [focal, 0.0, width / 2.0, 0.0,
                  0.0, focal, height / 2.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.distortion_model = 'plumb_bob'
        return info

    def destroy_node(self):
        self.capture.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = WebcamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
