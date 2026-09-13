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

The maths lives in rospider_gazebo/tags.py so it can be tested without a
simulator; this file is only the ROS plumbing.
"""

import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from interfaces.msg import ApriltagInfo, ApriltagsInfo
from rclpy.node import Node
from rospider_gazebo import tags
from rospider_gazebo.ros_image import to_image_msg
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
        self.max_reproj_error = float(self._param('max_reproj_error_px', 3.0))
        self.publish_tf = bool(self._param('publish_tf', True))
        self.draw = bool(self._param('draw', True))
        image_topic = str(self._param('image_topic',
                                      '/depth_cam/rgb/image_raw'))

        self.camera_matrix = None
        self.dist_coeffs = None

        self.tags_pub = self.create_publisher(ApriltagsInfo,
                                              '~/apriltag_info', 1)
        self.image_pub = self.create_publisher(Image, '~/image_result', 1)
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Image, image_topic, self.image_callback, 1)
        self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info',
                                 self.info_callback, 1)
        self.get_logger().info(
            f'watching for {tags.FAMILY} tags of {self.tag_size} m on '
            f'{image_topic}')

    def _param(self, name, default):
        """Read a parameter, falling back when the YAML does not carry it.

        Parameters are declared from overrides, so anything absent from
        config/apriltag.yaml comes back as None rather than raising. Same
        helper, same reason, as color_detect.py.
        """
        value = self.get_parameter(name).value
        return default if value is None else value

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

        info = ApriltagsInfo()
        transforms = []
        for tag_id, corners in tags.detect_tags(gray):
            solved = tags.solve_tag_pose(
                corners, self.camera_matrix, self.dist_coeffs, self.tag_size)
            if solved is None:
                continue
            rvec, tvec, error = solved
            if error > self.max_reproj_error:
                # The two IPPE_SQUARE solutions are both poor; whichever won
                # the argmin is not trustworthy. Drop it rather than publish a
                # frame that may be the mirror pose.
                continue

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

    @staticmethod
    def _tag_info(tag_id, corners, tvec):
        centre = corners.mean(axis=0)
        width = float(np.linalg.norm(corners[1] - corners[0]))
        info = ApriltagInfo()
        info.id = int(tag_id)
        info.x = int(round(float(centre[0])))
        info.y = int(round(float(centre[1])))
        info.w = int(round(width))
        info.d = int(round(float(tvec[2][0]) * 1000.0))   # millimetres
        return info

    def _transform(self, tag_id, rvec, tvec, header):
        transform = TransformStamped()
        transform.header = header          # same stamp and frame as the image
        transform.child_frame_id = f'tag_{tag_id}'
        transform.transform.translation.x = float(tvec[0][0])
        transform.transform.translation.y = float(tvec[1][0])
        transform.transform.translation.z = float(tvec[2][0])
        x, y, z, w = tags.quaternion_from_rvec(rvec)
        transform.transform.rotation.x = x
        transform.transform.rotation.y = y
        transform.transform.rotation.z = z
        transform.transform.rotation.w = w
        return transform

    def _draw(self, frame, corners, tag_id, rvec, tvec):
        cv2.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 2)
        centre = corners.mean(axis=0).astype(int)
        cv2.putText(frame, f'id {tag_id}', (int(centre[0]) - 20, int(centre[1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs,
                          rvec, tvec, self.tag_size * 0.5)


def main():
    rclpy.init()
    node = AprilTagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
