#!/usr/bin/env python3
"""Pick a thing on screen and follow it -- the KCF tracking window.

A port of example/opencv_example/include/kcf.py. Press `s`, drag a box round
anything in the camera view, press Space or Enter, and the robot turns to keep
that box in the middle of the frame. Press `s` again to re-select, `q` or Esc
to quit.

The tracker is not always KCF. OpenCV moved KCF into contrib and dropped it
from the 5.x main module that pip's `opencv-python` installs, so
vision_demo.create_tracker() asks for KCF, then CSRT, then MIL, and the window
shows which one it got. Set `tracker:=csrt` to pin one.

    ros2 launch rospider_gazebo gazebo.launch.py
    ros2 launch rospider_gazebo vision_demo.launch.py demo:=kcf_track
"""

import cv2
from rospider_gazebo import vision_demo
from rospider_gazebo.pid import PID, set_range
from rospider_gazebo.vision_demo import VisionDemo, create_tracker

#: Upstream's init_process() pose, servo ids 19-24.
LOOK_POSE = ((19, 500), (20, 750), (21, 200), (22, 150), (23, 500), (24, 700))

DRAW = (255, 255, 0)


class KcfTrackNode(VisionDemo):

    window = 'result'

    def __init__(self):
        super().__init__('kcf_track', cmd_vel=True, servos=True)
        self.kind = str(self.param('tracker', 'auto'))
        self.turn_limit = float(self.param('turn_limit', 0.3))
        self.pid_yaw = PID(0.05, 0.0, 0.0)
        self.tracker = None
        self.tracker_name = ''
        self.select_next = False

    def on_start(self):
        self.servos.set_servo_position(1.0, LOOK_POSE)
        self.get_logger().info(
            'press "s" in the window to draw a box round the target, then '
            'Space or Enter to start tracking')

    def on_key(self, key):
        if key == ord('s'):
            self.stop()
            self.tracker = None
            self.select_next = True
            return True
        return False

    def process(self, frame):
        """Upstream's overlay: one cyan rectangle round the tracked box."""
        if self.select_next:
            self.select_next = False
            self._select(frame)
            return frame

        if self.tracker is None:
            return frame

        found, box = self.tracker.update(frame)
        if not found:
            self.stop()
            self.pid_yaw.clear()
            self.get_logger().info('target lost', throttle_duration_sec=2.0)
            return frame

        x, y, w, h = (int(v) for v in box)
        cv2.rectangle(frame, (x, y), (x + w, y + h), DRAW, 2)
        centre_x = x + w / 2.0
        width = frame.shape[1]

        self.pid_yaw.SetPoint = width / 2.0
        self.pid_yaw.update(centre_x)
        # The PID sees width/2 - centre_x, so a target to the right gives a
        # negative output and the robot turns right.
        angular = set_range(self.pid_yaw.output, -10, 10) / 10.0 * self.turn_limit
        self.drive(0.0, angular)
        return frame

    def _select(self, frame):
        """Let the user drag a box; build a tracker if the box has area."""
        roi = cv2.selectROI(self.window, frame, False)
        if roi[2] <= 0 or roi[3] <= 0:
            self.get_logger().info('selection cancelled')
            return
        self.tracker, self.tracker_name = create_tracker(self.kind)
        self.tracker.init(frame, tuple(int(v) for v in roi))
        self.get_logger().info(f'tracking with {self.tracker_name}')

    def on_stop(self):
        self.stop()


def main():
    vision_demo.main(KcfTrackNode)


if __name__ == '__main__':
    main()
