#!/usr/bin/env python3
"""Read out the tag under the camera -- the AprilTag position window.

A port of example/opencv_example/include/apriltag_position.py. The detections
come from this package's own scripts/apriltag_detect.py, which publishes the
same interfaces/ApriltagsInfo on /apriltag_detect/apriltag_info and the same
overlay on /apriltag_detect/image_result that upstream's node does.

One field reads differently, and better: `d` is millimetres here, where
upstream's detector builds its object points from unit corners and so reports
a distance in units of half a tag width. The window prints the units it is
given.

    ros2 launch rospider_gazebo gazebo.launch.py
    ros2 launch rospider_gazebo vision_demo.launch.py demo:=apriltag_position
"""

from interfaces.msg import ApriltagsInfo
from rospider_gazebo import vision_demo
from rospider_gazebo.vision_demo import VisionDemo

#: Upstream's init_process() pose, servo ids 19-24.
LOOK_POSE = ((19, 500), (20, 750), (21, 200), (22, 150), (23, 500), (24, 700))


class AprilTagPositionNode(VisionDemo):

    window = 'image'

    def __init__(self):
        super().__init__('apriltag_position',
                         image_topic='/apriltag_detect/image_result',
                         servos=True)
        self.tags = []
        self.create_subscription(
            ApriltagsInfo,
            str(self.param('tags_topic', '/apriltag_detect/apriltag_info')),
            self.tags_callback, 1)

    def on_start(self):
        self.servos.set_servo_position(1.0, LOOK_POSE)
        self.get_logger().info(
            'start apriltag_detect.py as well, or this window stays empty')

    def tags_callback(self, message):
        self.tags = list(message.data)

    def process(self, frame):
        """Upstream shows the detector's frame and logs the first tag."""
        if self.tags:
            first = self.tags[0]
            self.get_logger().info(
                f'ID: {first.id}, X: {first.x:.2f}, Y: {first.y:.2f}, '
                f'W: {first.w:.2f}')
        return frame


def main():
    vision_demo.main(AprilTagPositionNode)


if __name__ == '__main__':
    main()
