#!/usr/bin/env python3
"""Walk up to a tag and hold station -- the AprilTag tracking window.

A port of example/opencv_example/include/apriltag_track.py: a yaw PID keeps
the tag in the middle of the frame, a distance PID holds `stop_distance`, and
both let go the moment the target tag is out of sight.

The distance loop works in metres here. Upstream's runs on its detector's `d`
field, which is neither millimetres nor metres but half tag widths (see
scripts/apriltag_detect.py's header), so its 0.002 gain and d_stop of 15 mean
nothing in this workspace. Everything else -- the pixel deadband, the yaw
gain, the 0.2 rad/s ceiling -- is upstream's.

This demo steers the robot. scripts/apriltag_detect.py can steer it too, with
behaviours enabled; run only one of the two.

    ros2 launch rospider_gazebo gazebo.launch.py
    ros2 launch rospider_gazebo vision_demo.launch.py demo:=apriltag_track target_tag:=1
"""

from interfaces.msg import ApriltagsInfo
from rospider_gazebo import vision_demo
from rospider_gazebo.pid import PID, set_range
from rospider_gazebo.vision_demo import VisionDemo

#: Upstream's init_process() pose, servo ids 19-24.
LOOK_POSE = ((19, 500), (20, 750), (21, 200), (22, 150), (23, 500), (24, 700))

#: Pixels off centre that still count as centred, as upstream's x deadband.
YAW_DEADBAND = 20
#: Metres off the standoff that still count as arrived.
DISTANCE_DEADBAND = 0.03


class AprilTagTrackNode(VisionDemo):

    window = 'image'

    def __init__(self):
        super().__init__('apriltag_track',
                         image_topic='/apriltag_detect/image_result',
                         cmd_vel=True, servos=True)
        self.target_tag = int(self.param('target_tag', 1))
        self.stop_distance = float(self.param('stop_distance', 0.35))
        self.speed_limit = float(self.param('speed_limit', 0.05))
        self.turn_limit = float(self.param('turn_limit', 0.2))
        self.pid_yaw = PID(0.005, 0.0, 0.000001)
        self.pid_distance = PID(1.0, 0.0, 0.0)
        self.tag = None
        self.create_subscription(
            ApriltagsInfo,
            str(self.param('tags_topic', '/apriltag_detect/apriltag_info')),
            self.tags_callback, 1)

    def on_start(self):
        self.servos.set_servo_position(1.0, LOOK_POSE)
        self.get_logger().info(
            f'following tag {self.target_tag}, stopping {self.stop_distance} m '
            'short; start apriltag_detect.py as well')

    def tags_callback(self, message):
        self.tag = next((t for t in message.data if t.id == self.target_tag),
                        None)

    def process(self, frame):
        width = frame.shape[1]
        if self.tag is None:
            self.stop()
            self.pid_yaw.clear()
            self.pid_distance.clear()
            return frame

        distance = self.tag.d / 1000.0
        linear = angular = 0.0

        if abs(distance - self.stop_distance) > DISTANCE_DEADBAND:
            self.pid_distance.SetPoint = 0.0
            self.pid_distance.update(distance - self.stop_distance)
            # The PID sees 0 - (distance - standoff), so its output is
            # negative when the tag is too far; the sign flip turns that
            # into walking forward.
            linear = -set_range(self.pid_distance.output,
                                -self.speed_limit, self.speed_limit)
        else:
            self.pid_distance.clear()

        if abs(self.tag.x - width / 2) > YAW_DEADBAND:
            self.pid_yaw.SetPoint = 0.0
            self.pid_yaw.update(self.tag.x - width / 2)
            angular = set_range(self.pid_yaw.output, -1.0, 1.0) * self.turn_limit
        else:
            self.pid_yaw.clear()

        self.drive(linear, angular)
        return frame

    def on_stop(self):
        self.stop()


def main():
    vision_demo.main(AprilTagTrackNode)


if __name__ == '__main__':
    main()
