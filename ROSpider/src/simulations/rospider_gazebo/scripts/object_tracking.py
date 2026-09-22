#!/usr/bin/env python3
"""Click on a thing in the picture and the robot walks after it -- the
app's object tracking (docs 6 Object Tracking, `ros2 service call
/object_tracking/enter`).

A port of app/app/object_tracking.py. A left click in the `image` window
samples the colour under the cursor for ten frames (the growing ring
upstream draws), and from then on the biggest patch of that colour is
circled and two PIDs drive /controller/cmd_vel to keep it on the yellow
stop dot: its height in the frame sets the forward speed, its x the turn.
Threshold, picker and PID gains are upstream's
(rospider_gazebo/color_picker.py). One change: once locked on, the patch
nearest the last one is kept rather than the biggest -- the rule upstream
wrote but never wired up -- because the arena's posters carry every colour
and a bigger patch of it on the wall would steal the target.

The phone app's services are kept so the docs' commands work: ~/enter
(re-pose, forget the target), ~/exit, ~/set_running (SetBool),
~/set_threshold (SetFloat64), ~/set_target_color (SetPoint, normalised
0..1; -1,-1 clears) and ~/get_target_color. Upstream starts stopped and
waits for set_running; here start:=true (the default) tracks as soon as a
colour is picked, so one launch and one click is the whole demo. Left out:
~/set_color, which needs the robot's lab_config.yaml.

Upstream shows the window scaled to 960x600 and normalises the click by
that; the sim shows the frame at its own 640x480, so a click is a frame
pixel directly.

    ros2 launch rospider_gazebo pick_place.launch.py        # something to chase
    ros2 launch rospider_gazebo object_tracking.launch.py
"""

import cv2
from interfaces.srv import SetFloat64, SetPoint
from rospider_gazebo import vision_demo
from rospider_gazebo.color_picker import (
    PICK_REPEAT, STOP_POINT, ColorPicker, FollowControl, find_blob, lab_band)
from rospider_gazebo.vision_demo import VisionDemo
from std_srvs.srv import SetBool, Trigger

#: Upstream's enter_srv_callback() pose, servo ids 19-24.
LOOK_POSE = ((19, 500), (20, 720), (21, 130), (22, 150), (23, 500), (24, 500))

YELLOW = (0, 255, 255)      # upstream's (255, 255, 0), RGB


class ObjectTrackingNode(VisionDemo):

    window = 'image'

    def __init__(self):
        super().__init__('object_tracking', cmd_vel=True, servos=True)
        self.threshold = float(self.param('threshold', 0.5))
        self.tracking = bool(self.param('start', True))
        self.repeat = int(self.param('pick_repeat', PICK_REPEAT))
        self.control = FollowControl()
        self.picker = None
        self.target = None          # (lab, bgr) once picked
        self.band = None
        self.last_blob = None
        self.create_service(Trigger, '~/enter', self.enter_callback)
        self.create_service(Trigger, '~/exit', self.exit_callback)
        self.create_service(SetBool, '~/set_running', self.set_running_callback)
        self.create_service(SetFloat64, '~/set_threshold',
                            self.set_threshold_callback)
        self.create_service(SetPoint, '~/set_target_color',
                            self.set_target_color_callback)
        self.create_service(Trigger, '~/get_target_color',
                            self.get_target_color_callback)

    def on_start(self):
        self.servos.set_servo_position(1.0, LOOK_POSE)
        hint = '' if self.tracking else '; then ~/set_running true'
        self.get_logger().info(
            f'left-click the object in the window to pick its colour{hint}')

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
        self.last_blob = None
        self.control.clear()
        self.stop()

    # ------------------------------------------------------------ services

    def enter_callback(self, _request, response):
        self.get_logger().info('object tracking enter')
        self._forget()
        self.threshold = 0.5
        self.servos.set_servo_position(1.0, LOOK_POSE)
        response.success = True
        response.message = 'enter'
        return response

    def exit_callback(self, _request, response):
        self.get_logger().info('object tracking exit')
        self._forget()
        self.tracking = False
        response.success = True
        response.message = 'exit'
        return response

    def set_running_callback(self, request, response):
        self.get_logger().info(f'set_running {request.data}')
        self.tracking = request.data
        if not self.tracking:
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

    def get_target_color_callback(self, _request, response):
        response.success = self.target is not None
        response.message = 'get_target_color'
        if self.target is not None:
            b, g, r = self.target[1]
            response.message = f'{r},{g},{b}'
        return response

    # ------------------------------------------------------------ frames

    frame_size = (640, 480)

    def process(self, frame):
        """Upstream's overlay: the pick ring while sampling, then the
        yellow stop dot and a circle round the target in its own colour."""
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

        blob = find_blob(frame, *self.band, last=self.last_blob)
        self.last_blob = blob
        if blob is None:
            if self.tracking:
                self.stop()
            return frame
        x, y, r = blob
        cv2.circle(frame, STOP_POINT, 15, YELLOW, -1)
        cv2.circle(frame, (int(x), int(y)), int(r), self.target[1], 2)
        linear, angular = self.control.update(x, y)
        if self.tracking:
            self.drive(linear, angular)
        else:
            self.control.clear()
        return frame

    def on_stop(self):
        self.stop()


def main():
    vision_demo.main(ObjectTrackingNode)


if __name__ == '__main__':
    main()
