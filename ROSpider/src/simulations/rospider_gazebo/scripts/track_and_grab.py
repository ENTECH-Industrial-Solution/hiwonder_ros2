#!/usr/bin/env python3
"""See a coloured block, follow it, then pick it up -- the track-and-grab
window (docs 8.7, `ros2 launch example track_and_grab.launch.py`).

A port of example/rgbd_example/include/track_and_grab.py. Upstream's window
is the RGB frame and the depth colour map side by side: the block circled in
its colour, a white dot on its centre in both halves, and `Dist: NNNmm` on
the depth half. Two PIDs pan (servo 19) and tilt (servo 22) the camera after
the block; once the camera has held still on it for two seconds the arm
goes and picks it.

Two parts are the simulation's own:

  * The block comes from the colour detector's ObjectsInfo (the leftmost box
    of the wanted colour, as upstream takes the leftmost contour) instead of
    an in-file LAB threshold, so the same bands the LAB_Tool window tunes
    apply here.
  * The pick is scripts/pick_and_place.py's: upstream turns the depth pixel
    into a point for the arm_kinematics IK service (aarch64-only), grabs,
    swings the block to its side and lets go; here `/pick_and_place/pick
    <colour>` looks, grabs and lifts with the sim's closed-form IK, and
    `/pick_and_place/place` then puts the block down at its place_point.
    Tracking pauses while pick_and_place reports it is busy and resumes
    when it is done.

launch/track_and_grab.launch.py starts the whole thing, as upstream's does:
the simulator with the blocks, the colour detector, pick_and_place (with
auto_start:=false, so it waits for this node) and this window.

    ros2 launch rospider_gazebo track_and_grab.launch.py color:=red
"""

import time

import cv2
import numpy as np
from interfaces.msg import ObjectsInfo
from interfaces.srv import SetString
from rclpy.qos import DurabilityPolicy, QoSProfile
from rospider_gazebo import vision_demo
from rospider_gazebo.detections import box_centroid
from rospider_gazebo.pid import PID, set_range
from rospider_gazebo.vision_demo import VisionDemo, depth_color_map
from std_msgs.msg import String

#: Start pose, servo ids 19-24: the camera-levelling arm shape, as in
#: scripts/face_track.py and scripts/color_track.py.
LEVEL_POSE = ((19, 500), (20, 650), (21, 40), (22, 432), (23, 500), (24, 500))

PAN_PULSE = (0, 1000)       # servo 19 travel, upstream's yaw limits
TILT_PULSE = (332, 532)     # servo 22 travel, +/- 100 counts (~24 deg)

#: Hiwonder's circle colours (driver/sdk/sdk/common.py range_rgb), BGR; a
#: colour not in the table draws grey, as upstream's 0x55 fallback.
RANGE_RGB = {'red': (0, 50, 255), 'green': (50, 255, 0), 'blue': (255, 50, 0)}
GREY = (0x55, 0x55, 0x55)

#: The depth map saturates at 2 m, as upstream scales it.
DEPTH_CEILING = 2000
#: Servo pulses the camera may still move by and count as holding still,
#: and how long it must hold before the pick starts -- upstream's numbers.
SETTLE_PULSES = 3
SETTLE_SECONDS = 2.0
#: pick_and_place states in which the arm is free for tracking, and the one
#: in which it holds the block up, waiting to be told where to put it.
FREE_STATES = ('IDLE', 'DONE')
CARRY_STATE = 'CARRY'
#: Where the block is put down, "x y z" in base_footprint metres: off to the
#: left, the way upstream's pick() swings servo 19 to 850 (about 84 deg)
#: before letting go, so the block leaves the camera's view instead of
#: being picked again.
PLACE_POINT = '0.02 0.16 0.035'


class TrackAndGrabNode(VisionDemo):

    window = 'depth'

    def __init__(self):
        super().__init__('track_and_grab', depth=True, servos=True)
        self.color = str(self.param('color', 'red'))
        self.tracking = bool(self.param('start', True))
        self.pan = float(self.param('pan_pulse', 500))
        self.tilt = float(self.param('tilt_pulse', LEVEL_POSE[3][1]))
        self.pid_pan = PID(float(self.param('pan_gain', 0.055)), 0.0, 0.0)
        self.pid_tilt = PID(float(self.param('tilt_gain', 0.05)), 0.0, 0.0)
        self.place_point = str(self.param('place_point', PLACE_POINT))
        self.target = None          # (u, v, radius) of the block, pixels
        self.last_pose = (self.pan, self.tilt)
        self.still_since = time.monotonic()
        self.pick_state = 'IDLE'
        self.pick_client = self.create_client(SetString, '/pick_and_place/pick')
        self.place_client = self.create_client(SetString, '/pick_and_place/place')
        self.create_subscription(
            ObjectsInfo, str(self.param('objects_topic', '/yolo/object_detect')),
            self.objects_callback, 1)
        self.create_subscription(
            String, '/pick_and_place/state', self.state_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_start(self):
        self.servos.set_servo_position(1.5, LEVEL_POSE)
        self.get_logger().info(
            f'tracking {self.color}; the pick goes through /pick_and_place/pick')

    # ----------------------------------------------------------- callbacks

    def objects_callback(self, message):
        """The leftmost box of the wanted colour, as upstream's tracker."""
        boxes = [o.box for o in message.objects if o.class_name == self.color]
        if not boxes:
            self.target = None
            return
        box = min(boxes, key=lambda b: box_centroid(b)[0])
        u, v, area = box_centroid(box)
        self.target = (u, v, (area ** 0.5) / 2.0)

    def state_callback(self, message):
        if message.data == self.pick_state:
            return
        self.get_logger().info(f'pick_and_place: {message.data}')
        if message.data == CARRY_STATE:
            # Grabbed and lifted: put it down beside the robot, as
            # upstream's pick() ends.
            future = self.place_client.call_async(
                SetString.Request(data=self.place_point))
            future.add_done_callback(self._placed)
        elif message.data in FREE_STATES and self.pick_state not in FREE_STATES:
            # The pick left the arm at its own pose; go back to tracking's.
            self.servos.set_servo_position(1.5, LEVEL_POSE)
        self.pick_state = message.data

    def _placed(self, future):
        response = future.result()
        if not response.success:
            self.get_logger().error(f'place refused: {response.message}')

    # ---------------------------------------------------------------- loop

    def process(self, frame):
        depth_mm = self.depth_mm
        if depth_mm is None:
            self.get_logger().info('waiting for depth', throttle_duration_sec=2.0)
            return frame
        if depth_mm.shape[:2] != frame.shape[:2]:
            frame = cv2.resize(frame, (depth_mm.shape[1], depth_mm.shape[0]))
        view = depth_color_map(depth_mm, DEPTH_CEILING)
        height, width = frame.shape[:2]

        busy = self.pick_state not in FREE_STATES
        if self.tracking and not busy and self.target is not None:
            u, v, radius = self.target
            u, v = min(u, width - 1), min(v, height - 1)
            cv2.circle(frame, (int(u), int(v)), int(radius),
                       RANGE_RGB.get(self.color, GREY), 2)
            self._follow(u, v, width, height)
            if self._held_still():
                self._pick()
            dist = int(depth_mm[int(v), int(u)])
            text = 'TOO CLOSE !!!' if dist < 100 else f'Dist: {dist}mm'
            cv2.circle(frame, (int(u), int(v)), 5, (255, 255, 255), -1)
            cv2.circle(view, (int(u), int(v)), 5, (255, 255, 255), -1)
            cv2.putText(view, text, (10, 400 - 20), cv2.FONT_HERSHEY_PLAIN, 2.0,
                        (0, 0, 0), 10, cv2.LINE_AA)
            cv2.putText(view, text, (10, 400 - 20), cv2.FONT_HERSHEY_PLAIN, 2.0,
                        (255, 255, 255), 2, cv2.LINE_AA)
        else:
            self.still_since = time.monotonic()
        return np.concatenate([frame, view], axis=1)

    def _follow(self, u, v, width, height):
        """Two PIDs, servo 19 for x and 22 for y, as scripts/color_track.py."""
        self.pid_pan.SetPoint = width / 2.0
        self.pid_pan.update(u)
        self.pan = set_range(self.pan + self.pid_pan.output, *PAN_PULSE)
        self.pid_tilt.SetPoint = height / 2.0
        self.pid_tilt.update(v)
        self.tilt = set_range(self.tilt + self.pid_tilt.output, *TILT_PULSE)
        self.servos.set_servo_position(0.02, ((19, int(self.pan)),
                                              (22, int(self.tilt))))

    def _held_still(self):
        """True once the camera has moved under SETTLE_PULSES for
        SETTLE_SECONDS: the block is centred and not going anywhere."""
        pan, tilt = self.last_pose
        if abs(pan - self.pan) >= SETTLE_PULSES or abs(tilt - self.tilt) >= SETTLE_PULSES:
            self.still_since = time.monotonic()
        self.last_pose = (self.pan, self.tilt)
        return time.monotonic() - self.still_since > SETTLE_SECONDS

    def _pick(self):
        """Hand the block to pick_and_place; tracking resumes on DONE."""
        self.still_since = time.monotonic()
        if not self.pick_client.service_is_ready():
            self.get_logger().warn(
                '/pick_and_place/pick is not there: start pick_place.launch.py '
                'for the grab; tracking only', throttle_duration_sec=5.0)
            return
        self.get_logger().info(f'stop -- picking {self.color}')
        self.pick_state = 'LOOK'        # busy until the node reports back
        self.pick_client.call_async(SetString.Request(data=self.color))
        self.pan, self.tilt = 500.0, float(LEVEL_POSE[3][1])
        self.pid_pan.clear()
        self.pid_tilt.clear()

    def on_stop(self):
        self.servos.set_servo_position(1.5, LEVEL_POSE)


def main():
    vision_demo.main(TrackAndGrabNode)


if __name__ == '__main__':
    main()
