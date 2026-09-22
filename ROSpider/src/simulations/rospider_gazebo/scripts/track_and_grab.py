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

One more is the workshop's: upstream tracks its one colour from the moment
it starts, this node waits to be told. It opens idle, the camera level and
nothing followed, until a colour is chosen -- a button in the small Tk
control window next to the depth one (gui:=true, the default; one button
per colour of `colors`, Stop, and a status line), or
`ros2 service call /track_and_grab/pick interfaces/srv/SetString
"{data: red}"`. One order is one job: follow, settle, pick, put down, and
back to idle for the next. ~/set_running false (or Stop) drops the job.
start:=true tracks `color` from the start, as upstream does.

launch/track_and_grab.launch.py starts the whole thing, as upstream's does:
the simulator with the blocks, the colour detector, pick_and_place (with
auto_start:=false, so it waits for this node) and this window.

    ros2 launch rospider_gazebo track_and_grab.launch.py
"""

import time
import tkinter as tk

import cv2
import numpy as np
from interfaces.msg import ObjectsInfo
from interfaces.srv import SetString
from rclpy.qos import DurabilityPolicy, QoSProfile
from rospider_gazebo import tkview, vision_demo
from rospider_gazebo.detections import box_centroid
from rospider_gazebo.pid import PID, set_range
from rospider_gazebo.vision_demo import VisionDemo, depth_color_map
from std_msgs.msg import String
from std_srvs.srv import SetBool

#: Start pose, servo ids 19-24: scripts/color_track.py's camera-levelling
#: shape, but with the wrist (servo 22) at the robot's `init` tilt, 52 deg
#: down. Upstream's block sits on a table at camera height; the sim's sit
#: on the 8 cm pedestal 0.235 m ahead, 43 deg below a level camera and out
#: of its tilt travel. From `init` they show at the bottom of the frame and
#: the tilt loop brings them to the centre (about pick_and_place's look
#: pose, servo 22 at 130).
LEVEL_POSE = ((19, 500), (20, 650), (21, 40), (22, 215), (23, 500), (24, 500))

PAN_PULSE = (0, 1000)       # servo 19 travel, upstream's yaw limits
TILT_PULSE = (115, 315)     # servo 22 travel, +/- 100 counts (~24 deg)

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


def tk_colour(bgr):
    """A BGR tuple as the '#rrggbb' Tk wants."""
    return '#%02x%02x%02x' % (bgr[2], bgr[1], bgr[0])


class TrackAndGrabNode(VisionDemo):

    window = 'depth'

    def __init__(self):
        super().__init__('track_and_grab', depth=True, servos=True)
        self.color = str(self.param('color', 'red'))
        self.colors = [str(c) for c in self.param('colors', ['red', 'green', 'blue'])]
        self.tracking = bool(self.param('start', False))
        self.gui = bool(self.param('gui', True)) and self.show
        self.root = None            # the Tk control window, once open
        self.status = None
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
        self.create_service(SetString, '~/pick', self.pick_callback)
        self.create_service(SetBool, '~/set_running', self.set_running_callback)

    def on_start(self):
        self.servos.set_servo_position(1.5, LEVEL_POSE)
        if self.gui:
            self._open_panel()
        if self.tracking:
            self.get_logger().info(f'tracking {self.color}')
        else:
            self.get_logger().info(
                'idle: pick a colour in the control window or call ~/pick')

    # -------------------------------------------------------------- orders

    def start_job(self, color):
        """Follow `color`, pick it and put it down, then go idle again."""
        if color not in self.colors:
            return f'{color!r} is not one of {self.colors}'
        if self.pick_state not in FREE_STATES:
            return f'busy: pick_and_place is in {self.pick_state}'
        self.color = color
        self.target = None
        self.tracking = True
        self.pid_pan.clear()
        self.pid_tilt.clear()
        self.still_since = time.monotonic()
        self.get_logger().info(f'tracking {color}')
        return ''

    def stop_job(self):
        """Drop the current order; the camera goes back to level -- unless
        pick_and_place has the arm, which cannot be called off: moving the
        camera under its LOOK would only make it miss the block."""
        self.tracking = False
        self.pid_pan.clear()
        self.pid_tilt.clear()
        self.pan, self.tilt = 500.0, float(LEVEL_POSE[3][1])
        if self.pick_state in FREE_STATES:
            self.servos.set_servo_position(1.5, LEVEL_POSE)
            self.get_logger().info('idle')
        else:
            self.get_logger().info(
                f'order dropped; pick_and_place is still in {self.pick_state}')

    def status_text(self):
        if self.pick_state not in FREE_STATES:
            return f'{self.pick_state.lower()} {self.color}'
        if self.tracking:
            return f'tracking {self.color}'
        return 'idle -- pick a colour'

    def pick_callback(self, request, response):
        error = self.start_job(request.data or self.colors[0])
        response.success = not error
        response.message = error or f'tracking {self.color}'
        return response

    def set_running_callback(self, request, response):
        if request.data:
            error = self.start_job(self.color)
        else:
            error = ''
            self.stop_job()
        response.success = not error
        response.message = error or self.status_text()
        return response

    # --------------------------------------------------------------- panel

    def _open_panel(self):
        """The control window: a button per colour, Stop, a status line.
        Pumped by on_tick(), since highgui holds the main thread's loop."""
        root = tk.Tk()
        root.title('track_and_grab')
        root.resizable(False, False)
        row = tk.Frame(root)
        row.pack(padx=8, pady=(8, 4))
        for color in self.colors:
            tkview.flat_button(
                row, color, 80, 32, bg=tk_colour(RANGE_RGB.get(color, GREY)),
                command=lambda c=color: self._log_refusal(self.start_job(c))
            ).pack(side='left', padx=2)
        tkview.flat_button(row, 'Stop', 80, 32, bg=tkview.GREY_BUTTON,
                           command=self.stop_job).pack(side='left', padx=(10, 2))
        self.status = tk.StringVar(value=self.status_text())
        tk.Label(root, textvariable=self.status, anchor='w').pack(
            fill='x', padx=8, pady=(0, 8))
        root.protocol('WM_DELETE_WINDOW', self._close_panel)
        self.root = root

    def _log_refusal(self, error):
        if error:
            self.get_logger().warn(error)

    def _close_panel(self):
        if self.root is not None:
            self.root.destroy()
            self.root = None

    def on_tick(self):
        if self.root is None:
            return
        self.status.set(self.status_text())
        try:
            self.root.update()
        except tk.TclError:
            self.root = None

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
        was_busy = self.pick_state not in FREE_STATES
        self.pick_state = message.data
        if was_busy and message.data in FREE_STATES:
            # Picked and put down (or given up): the job is over. The pick
            # left the arm at its own pose; stop_job() levels it again.
            self.stop_job()

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
        self._close_panel()
        self.servos.set_servo_position(1.5, LEVEL_POSE)


def main():
    vision_demo.main(TrackAndGrabNode)


if __name__ == '__main__':
    main()
