#!/usr/bin/env python3
"""Click on the line in the picture and the robot walks along it -- the
app's line following (docs 6 Line Following,
`ros2 service call /line_following/enter`).

A port of app/app/line_following.py. A left click in the `image` window
samples the colour under the cursor for ten frames (the growing ring
upstream draws, rospider_gazebo/color_picker.py), and from then on three
thin strips across the lower half of the frame look for that colour, the
weighted centre becomes a deflection angle and one PID turns it into a
twist on /controller/cmd_vel (rospider_gazebo/line_follower.py). The
LiDAR parks the robot while anything is within 0.4 m in front. Strips,
weights, gains and speed are upstream's; the start pose is the robot's
init action rather than upstream's enter pose, see LOOK_POSE.

The phone app's services are kept so the docs' commands work: ~/enter
(re-pose, forget the line), ~/exit, ~/set_running (SetBool),
~/set_threshold (SetFloat64) and ~/set_target_color (SetPoint, normalised
0..1; -1,-1 clears). Upstream starts stopped and waits for set_running;
here start:=true (the default) follows as soon as a colour is picked, so
one launch and one click is the whole demo. Left out: ~/set_color, which
needs the robot's lab_config.yaml.

The sim's LiDAR scan starts at -pi where the robot's starts at 0, so the
front sector is picked by angle rather than by index; see line_follower.py.

    ros2 launch rospider_gazebo line_following.launch.py   # sim + window
"""

import cv2
from interfaces.srv import SetFloat64, SetPoint
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rospider_gazebo import vision_demo
from rospider_gazebo.color_picker import PICK_REPEAT, ColorPicker, lab_band
from rospider_gazebo.line_follower import LineControl, ObstacleStop, find_line
from rospider_gazebo.vision_demo import VisionDemo
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool, Trigger

#: The robot's init action (controller/config/init_pose.yaml), camera 52
#: degrees down at the floor. Upstream enters with 22 at 150, 21 at 130 and
#: 20 at 720; on the simulated arm that folds the wrist until the gripper
#: sits across the lens -- the depth image reads under 15 cm everywhere --
#: so the floor is never seen. Ids 19-24.
LOOK_POSE = ((24, 500), (23, 500), (22, 215), (21, 40), (20, 650), (19, 500))

YELLOW = (0, 255, 255)
RED = (0, 0, 255)


class LineFollowingNode(VisionDemo):

    window = 'image'

    def __init__(self):
        super().__init__('line_following', cmd_vel=True, servos=True)
        self.threshold = float(self.param('threshold', 0.5))
        self.following = bool(self.param('start', True))
        self.repeat = int(self.param('pick_repeat', PICK_REPEAT))
        # Upstream's constants unless given (the Line Following exercise hands them out)
        self.control = LineControl(kp=float(self.param('kp', 1.1)),
                                   speed=float(self.param('speed', 0.05)),
                                   max_turn=float(self.param('max_turn', 0.35)))
        self.guard = ObstacleStop(float(self.param('stop_threshold', 0.4)))
        self.picker = None
        self.target = None          # (lab, bgr) once picked
        self.band = None
        self.create_subscription(
            LaserScan, str(self.param('scan_topic', '/scan')), self.scan_callback,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.create_service(Trigger, '~/enter', self.enter_callback)
        self.create_service(Trigger, '~/exit', self.exit_callback)
        self.create_service(SetBool, '~/set_running', self.set_running_callback)
        self.create_service(SetFloat64, '~/set_threshold',
                            self.set_threshold_callback)
        self.create_service(SetPoint, '~/set_target_color',
                            self.set_target_color_callback)

    def on_start(self):
        self.servos.set_servo_position(1.0, LOOK_POSE)
        hint = '' if self.following else '; then ~/set_running true'
        self.get_logger().info(
            f'left-click the line in the window to pick its colour{hint}')

    def scan_callback(self, message):
        was = self.guard.stopped
        now = self.guard.update(message.ranges, message.angle_min,
                                message.angle_increment)
        if now and not was:
            self.get_logger().info('obstacle ahead, holding')
        elif was and not now:
            self.get_logger().info('path clear')

    # ------------------------------------------------------------ picking

    def on_mouse(self, event, x, y):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info(f'x:{x} y{y}')
            self._pick(x, y)

    def _pick(self, x, y):
        self._forget()
        self.picker = ColorPicker(x, y, self.repeat)

    def _forget(self):
        self.picker = None
        self.target = None
        self.band = None
        self.control.clear()
        self.stop()

    # ------------------------------------------------------------ services

    def enter_callback(self, _request, response):
        self.get_logger().info('line following enter')
        self._forget()
        self.threshold = 0.5
        self.servos.set_servo_position(1.0, LOOK_POSE)
        response.success = True
        response.message = 'enter'
        return response

    def exit_callback(self, _request, response):
        self.get_logger().info('line following exit')
        self._forget()
        self.following = False
        response.success = True
        response.message = 'exit'
        return response

    def set_running_callback(self, request, response):
        self.get_logger().info(f'set_running {request.data}')
        self.following = request.data
        if not self.following:
            self.control.clear()
            self.stop()
        response.success = True
        response.message = 'set_running'
        return response

    def set_threshold_callback(self, request, response):
        self.threshold = float(request.data)
        if self.target is not None:
            self.band = lab_band(self.target[0], self.threshold)
        response.success = True
        response.message = 'set_threshold'
        return response

    def set_target_color_callback(self, request, response):
        x, y = request.data.x, request.data.y
        if x == -1 and y == -1:
            self._forget()
        else:
            # The app sends the click as a fraction of the picture.
            self._pick(x * self.frame_size[0], y * self.frame_size[1])
        response.success = True
        response.message = 'set_target_color'
        return response

    # ------------------------------------------------------------ frames

    frame_size = (640, 480)

    def process(self, frame):
        """Upstream's overlay: the pick ring while sampling, then a yellow
        box round the line in each strip with a red dot at its centre."""
        self.frame_size = (frame.shape[1], frame.shape[0])
        if self.picker is not None:
            picked = self.picker.sample(frame)
            centre = (self.picker.x, self.picker.y)
            count = self.picker.count
            cv2.circle(frame, centre, count, self.picker.bgr, 2 * count)
            cv2.circle(frame, centre, count, YELLOW, 5)
            if picked is not None:
                self.target = picked
                self.band = lab_band(picked[0], self.threshold)
                self.picker = None
                self.get_logger().info(
                    f'target colour lab={picked[0]} bgr={picked[1]}')
            return frame

        if self.target is None:
            return frame

        angle, boxes = find_line(frame, *self.band)
        for box, (cx, cy) in boxes:
            cv2.drawContours(frame, [box], -1, YELLOW, 2)
            cv2.circle(frame, (int(cx), int(cy)), 5, RED, -1)
        if angle is None or not self.following or self.guard.stopped:
            self.control.clear()
            if self.following:
                self.stop()
            return frame
        self.drive(*self.control.update(angle))
        return frame

    def on_stop(self):
        self.stop()


def main():
    vision_demo.main(LineFollowingNode)


if __name__ == '__main__':
    main()
