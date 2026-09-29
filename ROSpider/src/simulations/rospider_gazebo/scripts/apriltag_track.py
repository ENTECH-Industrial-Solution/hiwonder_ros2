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
Off (start:=false or ~/set_running false) it publishes nothing on /controller/cmd_vel, so Nav2 or
track_and_grab can drive; target_tag can be changed at run time with `ros2 param set`.

    ros2 launch rospider_gazebo gazebo.launch.py
    ros2 launch rospider_gazebo vision_demo.launch.py demo:=apriltag_track target_tag:=1
"""

from interfaces.msg import ApriltagsInfo
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import SetBool
from rospider_gazebo import vision_demo
from rospider_gazebo.pid import PID, set_range
from rospider_gazebo.vision_demo import VisionDemo

#: Upstream's init_process() pose, servo ids 19-24.
LOOK_POSE = ((19, 500), (20, 750), (21, 150), (22, 150), (23, 500), (24, 700))

#: Pixels off centre that still count as centred, as upstream's x deadband.
YAW_DEADBAND = 20
#: Metres off the standoff that still count as arrived. Upstream's is 1 of
#: its d_stop 15, about 7%; 0.5 here swallowed the whole standoff, so the
#: robot stopped 0.85 m out and never backed away when the tag came closer.
DISTANCE_DEADBAND = 0.05


class AprilTagTrackNode(VisionDemo):

    window = 'image'

    def __init__(self):
        super().__init__('apriltag_track',
                         image_topic='/apriltag_detect/image_result',
                         cmd_vel=True, servos=True)
        self.target_tag = int(self.param('target_tag', 1))
        self.stop_distance = float(self.param('stop_distance', 0.5))
        self.speed_limit = float(self.param('speed_limit', 0.05))
        self.turn_limit = float(self.param('turn_limit', 0.2))
        # start: false waits for ~/set_running true (the AprilTag exercise's checker starts
        # the walk, so it sees it from the first step); the demo starts at once.
        self.following = bool(self.param('start', True))
        self.create_service(SetBool, '~/set_running', self.set_running_callback)
        self.pid_yaw = PID(0.005, 0.0, 0.000001)
        self.pid_distance = PID(0.5, 0.0, 0.0)
        self.tag = None
        self.create_subscription(
            ApriltagsInfo,
            str(self.param('tags_topic', '/apriltag_detect/apriltag_info')),
            self.tags_callback, 1)
        self.add_on_set_parameters_callback(self._on_parameters)

    def on_start(self):
        self.servos.set_servo_position(1.0, LOOK_POSE)
        self.get_logger().info(
            f'following tag {self.target_tag}, stopping {self.stop_distance} m '
            'short; start apriltag_detect.py as well')

    def set_running_callback(self, request, response):
        self.following = request.data
        if not self.following:
            self.stop()
            self.pid_yaw.clear()
            self.pid_distance.clear()
        else:
            # Another node (track_and_grab, pick_and_place) may have moved the arm since start-up.
            self.servos.set_servo_position(1.0, LOOK_POSE)
        response.success = True
        response.message = 'set_running'
        return response

    def _on_parameters(self, parameters):
        """target_tag may change at run time (check_mission.py's go_to_tag steps)."""
        for parameter in parameters:
            if parameter.name == 'target_tag':
                self.target_tag = int(parameter.value)
                self.tag = None
                self.pid_yaw.clear()
                self.pid_distance.clear()
        return SetParametersResult(successful=True)

    def tags_callback(self, message):
        self.tag = next((t for t in message.data if t.id == self.target_tag),
                        None)

    def process(self, frame):
        width = frame.shape[1]
        if not self.following:
            return frame                # off: /controller/cmd_vel belongs to someone else
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
