#!/usr/bin/env python3
"""See a coloured block, follow it, then pick it up -- the track-and-grab
window (docs 8.7, `ros2 launch example track_and_grab.launch.py`).

A port of example/rgbd_example/include/track_and_grab.py. Upstream's window
is the RGB frame and the depth colour map side by side: the block circled in
its colour, a white dot on its centre in both halves, and `Dist: NNNmm` on
the depth half. Two PIDs pan (servo 19) and tilt (servo 22) the camera after
the block; once the camera has held still on it for two seconds the arm
goes and picks it.

Three parts are the simulation's own:

  * The block comes from the colour detector's ObjectsInfo instead of an
    in-file LAB threshold, so the same bands the LAB_Tool window tunes
    apply here -- the nearest box of the colour that is up on a pedestal,
    where upstream takes the leftmost contour: the room's decorative twin
    of each block lies on the floor, out of the arm's reach.
  * The pick is scripts/pick_and_place.py's: upstream turns the depth pixel
    into a point for the arm_kinematics IK service (aarch64-only), grabs,
    swings the block to its side and lets go; here `/pick_and_place/pick
    <colour>` looks, grabs and lifts with the sim's closed-form IK, and
    `/pick_and_place/place` then puts the block down at its place_point.
  * Upstream tracks its one colour from the moment it starts; this node
    waits to be told. It opens idle, the camera down at the pedestal and
    nothing followed, until a colour is chosen -- a button in the window
    (gui:=true, the default: the camera view with one button per colour of
    `colors`, Stop, Place, the two mode switches and a status line, in one
    Tk window in place of the OpenCV one) or
    `ros2 service call /track_and_grab/pick interfaces/srv/SetString
    "{data: red}"`. One order is one job, then back to idle for the next.
    The two switches:

      - *pick here* / *walk to it*. Here: upstream's behaviour, the camera
        follows the block and the arm picks it where the robot stands --
        which only reaches the pedestal in front. Walk: the camera comes up
        and sweeps for the colour (SEARCH), then the block's pixel and
        depth are put through TF into base_footprint and
        rospider_gazebo/approach.py turns the body to face the block and
        walks it straight in until the block sits dead ahead at the
        pedestal spot (APPROACH), and only then does the settle-and-pick
        start. Only blocks up on a pedestal count: the floor is out of the
        arm's reach, and the room's decorative twins lie on it.
      - *place: auto* / *by button*. Auto puts the block down beside the
        robot the moment it is lifted, as upstream's pick() ends. By button
        holds it up (pick_and_place's CARRY) until Place is pressed -- or
        `~/place` is called -- so the robot can be driven off with it, and
        then puts it down on the floor in front.

    ~/set_running false (or Stop) drops the job while it is tracking or
    walking; once pick_and_place has the arm it runs to the end.
    start:=true tracks `color` from the start, as upstream does.

launch/track_and_grab.launch.py starts the whole thing, as upstream's does:
the simulator with the blocks, the colour detector, pick_and_place (with
auto_start:=false, so it waits for this node) and this window.

    ros2 launch rospider_gazebo track_and_grab.launch.py
"""

import math
import time
import tkinter as tk

import cv2
import numpy as np
import rclpy
import tf2_ros
from interfaces.msg import ObjectsInfo
from interfaces.srv import SetString
from rclpy.qos import DurabilityPolicy, QoSProfile
from rospider_gazebo import depth_probe, tkview, vision_demo
from rospider_gazebo.approach import Approach
from rospider_gazebo.detections import box_centroid
from rospider_gazebo.pid import PID, set_range
from rospider_gazebo.vision_demo import VisionDemo, depth_color_map
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

#: Start pose, servo ids 19-24: upstream color_track's camera-levelling
#: shape, but with the wrist (servo 22) at the robot's `init` tilt, 52 deg
#: down. Upstream's block sits on a table at camera height; the sim's sit
#: on the 8 cm pedestal 0.235 m ahead, 43 deg below a level camera and out
#: of its tilt travel. From `init` they show at the bottom of the frame and
#: the tilt loop brings them to the centre (about pick_and_place's look
#: pose, servo 22 at 130).
LEVEL_POSE = ((19, 500), (20, 650), (21, 40), (22, 215), (23, 500), (24, 500))
#: The wrist while searching for a block to walk to: 12 deg under level
#: (432), where a pedestal block a metre off sits mid-frame.
SEARCH_TILT = 380
#: The pan sweep while searching, servo 19 counts either side of centre
#: (108 deg: the blocks off to the sides are square on), and how far it
#: moves per frame.
SEARCH_PAN = 450
SEARCH_STEP = 12
#: ... and the body turns slowly under the sweep once it has backed off,
#: so a block behind the robot comes round within one turn.
SEARCH_TURN = 0.25

PAN_PULSE = (0, 1000)       # servo 19 travel, upstream's yaw limits
TILT_PULSE = (115, 532)     # servo 22 travel: the look pose up to level

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
#: Where place: auto puts the block down, "x y z" in base_footprint metres:
#: off to the left, the way upstream's pick() swings servo 19 to 850
#: (about 84 deg) before letting go -- in front is the pedestal the block
#: came off. The Place button puts it down in front instead, at
#: pick_and_place's place_point, once the robot has been driven off.
PLACE_POINT = '0.02 0.16 0.035'
#: What counts as a block to go for: its centre this high (base_footprint
#: metres) -- on the 8 cm pedestal a block's centre is at 0.105; the
#: room's decorative twins lie on the floor at 0.025, and the colour
#: patches on the wall posters and the box obstacle sit higher -- and
#: this big across, from the blob's width and its depth (the blocks are
#: 5 cm; a poster patch is a foot or more).
PEDESTAL_Z = (0.07, 0.14)
BLOCK_SIZE = (0.03, 0.10)
#: ... and standing free: the depth just above the blob (one blob height
#: up) must be this much further than the blob, or missing. A block has
#: the floor or the far wall behind it; a poster patch has its wall.
FREE_BEHIND = 0.15
#: How far in front the arm reaches from where the robot stands: pick here
#: refuses a block beyond it rather than sending pick_and_place to time out.
REACH_X = 0.35
#: How far the body backs off before a walk, and the speed. The robot
#: stands 1 cm from the pick pedestal in front, closer than its own
#: corners sweep when it turns, so the first move must be straight back.
UNDOCK_M = 0.2
UNDOCK_SPEED = 0.1

#: The job's phases, in the order a walk-to-it order goes through them.
IDLE, SEARCH, APPROACH, TRACK, PICK, CARRY, PLACE = (
    'idle', 'search', 'approach', 'track', 'pick', 'carry', 'place')
BUSY = (PICK, PLACE)


def tk_colour(bgr):
    """A BGR tuple as the '#rrggbb' Tk wants."""
    return '#%02x%02x%02x' % (bgr[2], bgr[1], bgr[0])


class TrackAndGrabNode(VisionDemo):

    window = 'depth'

    def __init__(self):
        super().__init__('track_and_grab', depth=True, servos=True, cmd_vel=True,
                         camera_info='/depth_cam/depth/camera_info')
        self.color = str(self.param('color', 'red'))
        self.colors = [str(c) for c in self.param('colors', ['red', 'green', 'blue'])]
        self.walk = bool(self.param('walk', False))
        self.auto_place = bool(self.param('auto_place', True))
        self.lost_timeout = float(self.param('lost_timeout', 3.0))
        self.undock_from = None         # odom (x, y) the walk backs off from
        self.approach = Approach(target_x=float(self.param('target_x', 0.235)),
                                 tolerance=float(self.param('tolerance', 0.04)))
        self.gui = bool(self.param('gui', True)) and self.show
        if self.gui:
            self.show = False           # the Tk window shows the frames
        self.root = None
        self.view = None                # the last frame drawn, for the window
        self.pan = float(self.param('pan_pulse', 500))
        self.tilt = float(self.param('tilt_pulse', LEVEL_POSE[3][1]))
        self.pid_pan = PID(float(self.param('pan_gain', 0.055)), 0.0, 0.0)
        self.pid_tilt = PID(float(self.param('tilt_gain', 0.05)), 0.0, 0.0)
        self.place_point = str(self.param('place_point', PLACE_POINT))
        self.phase = IDLE
        self.targets = []           # (u, v, radius) of the colour's blobs
        self.last_seen = 0.0
        self.sweep_dir = 1
        self.last_pose = (self.pan, self.tilt)
        self.still_since = time.monotonic()
        self.pick_state = 'IDLE'
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pick_client = self.create_client(SetString, '/pick_and_place/pick')
        self.place_client = self.create_client(SetString, '/pick_and_place/place')
        self.create_subscription(
            ObjectsInfo, str(self.param('objects_topic', '/yolo/object_detect')),
            self.objects_callback, 1)
        self.create_subscription(
            String, '/pick_and_place/state', self.state_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(SetString, '~/pick', self.pick_callback)
        self.create_service(Trigger, '~/place', self.place_callback)
        self.create_service(SetBool, '~/set_running', self.set_running_callback)
        self.create_service(SetBool, '~/set_walk', self.set_walk_callback)
        self.create_service(SetBool, '~/set_auto_place', self.set_auto_place_callback)

    def on_start(self):
        self.servos.set_servo_position(1.5, LEVEL_POSE)
        if self.gui:
            self._open_panel()
        if bool(self.param('start', False)):
            self.start_job(self.color)
        else:
            self.get_logger().info(
                'idle: pick a colour in the window or call ~/pick')

    # -------------------------------------------------------------- orders

    def start_job(self, color):
        """Order `color`: walk to it if the switch says so, pick it, put it
        down (or hold it for Place), then go idle again."""
        if color not in self.colors:
            return f'{color!r} is not one of {self.colors}'
        if self.phase != IDLE:
            return f'busy: {self.status_text()}'
        self.color = color
        self.targets = []
        self.pid_pan.clear()
        self.pid_tilt.clear()
        self.still_since = self.last_seen = time.monotonic()
        if self.walk:
            self.phase = SEARCH
            self.pan, self.tilt = 500.0, float(SEARCH_TILT)
            self.sweep_dir = 1
            self.undock_from = self._odom_xy() or (None, None)
            self.get_logger().info(f'searching for {color}')
        else:
            self.phase = TRACK
            self.get_logger().info(f'tracking {color}')
        return ''

    def stop_job(self):
        """Drop the current order; the camera goes back down -- unless
        pick_and_place has the arm, which cannot be called off: moving the
        camera under its LOOK would only make it miss the block."""
        if self.phase in BUSY:
            self.get_logger().info(
                f'order kept; pick_and_place is in {self.pick_state}')
            return
        if self.phase == CARRY:
            self.get_logger().info('still holding the block; press Place')
            return
        self.stop()
        self.phase = IDLE
        self.undock_from = None
        self.pid_pan.clear()
        self.pid_tilt.clear()
        self.pan, self.tilt = 500.0, float(LEVEL_POSE[3][1])
        self.servos.set_servo_position(1.5, LEVEL_POSE)
        self.get_logger().info('idle')

    def place_now(self, point=''):
        """Put the held block down at `point`, "x y z" in base_footprint;
        empty is in front, at pick_and_place's own place_point."""
        if self.phase != CARRY:
            return f'nothing held: {self.status_text()}'
        self.phase = PLACE
        future = self.place_client.call_async(SetString.Request(data=point))
        future.add_done_callback(self._placed)
        return ''

    def _placed(self, future):
        response = future.result()
        if not response.success:
            self.get_logger().error(f'place refused: {response.message}')

    def status_text(self):
        if self.phase in BUSY:
            return f'{self.pick_state.lower()} {self.color}'
        if self.phase == CARRY:
            return f'holding {self.color} -- press Place'
        if self.phase == IDLE:
            return 'idle -- pick a colour'
        return f'{self.phase} {self.color}'

    # ------------------------------------------------------------ services

    def pick_callback(self, request, response):
        error = self.start_job(request.data or self.colors[0])
        response.success = not error
        response.message = error or self.status_text()
        return response

    def place_callback(self, _request, response):
        error = self.place_now()
        response.success = not error
        response.message = error or 'placing'
        return response

    def set_running_callback(self, request, response):
        error = self.start_job(self.color) if request.data else ''
        if not request.data:
            self.stop_job()
        response.success = not error
        response.message = error or self.status_text()
        return response

    def set_walk_callback(self, request, response):
        self.walk = request.data
        response.success = True
        response.message = 'walk to it' if self.walk else 'pick here'
        return response

    def set_auto_place_callback(self, request, response):
        self.auto_place = request.data
        response.success = True
        response.message = 'place: auto' if self.auto_place else 'place: by button'
        return response

    # ----------------------------------------------------------- callbacks

    def objects_callback(self, message):
        """Every box of the wanted colour, for _choose_target()."""
        self.targets = []
        for box in (o.box for o in message.objects if o.class_name == self.color):
            u, v, area = box_centroid(box)
            self.targets.append((u, v, (area ** 0.5) / 2.0))

    def state_callback(self, message):
        if message.data == self.pick_state:
            return
        self.get_logger().info(f'pick_and_place: {message.data}')
        was_busy = self.pick_state not in FREE_STATES
        self.pick_state = message.data
        if message.data == CARRY_STATE and self.phase == PICK:
            # Grabbed and lifted: put it down beside the robot, as
            # upstream's pick() ends -- the pedestal it came off is still
            # in front -- or hold it for the Place button.
            self.phase = CARRY
            if self.auto_place:
                self.place_now(self.place_point)
        elif was_busy and message.data in FREE_STATES:
            # Placed (or given up): the job is over. The pick left the arm
            # at its own pose; stop_job() brings the camera back.
            self.phase = IDLE
            self.stop_job()

    # --------------------------------------------------------------- panel

    def _open_panel(self):
        """The window: the camera view over a button per colour, Stop and
        Place, the two mode switches and a status line. Pumped by
        on_tick(), since the frame loop owns the main thread."""
        root = tk.Tk()
        root.title('track_and_grab')
        root.resizable(False, False)
        self.image_label = tk.Label(root)
        self.image_label.pack(padx=8, pady=(8, 4))
        row = tk.Frame(root)
        row.pack(padx=8, pady=4)
        for color in self.colors:
            tkview.flat_button(
                row, color, 80, 32, bg=tk_colour(RANGE_RGB.get(color, GREY)),
                command=lambda c=color: self._log_refusal(self.start_job(c))
            ).pack(side='left', padx=2)
        tkview.flat_button(row, 'Stop', 80, 32, bg=tkview.GREY_BUTTON,
                           command=self.stop_job).pack(side='left', padx=(10, 2))
        self.place_button = tkview.flat_button(
            row, 'Place', 80, 32, bg=tkview.GREY_BUTTON,
            command=lambda: self._log_refusal(self.place_now()))
        self.place_button.pack(side='left', padx=2)
        modes = tk.Frame(root)
        modes.pack(padx=8, pady=4, anchor='w')
        self.walk_var = tk.BooleanVar(value=self.walk)
        self.auto_place_var = tk.BooleanVar(value=self.auto_place)
        for text, value in (('pick here', False), ('walk to it', True)):
            tk.Radiobutton(modes, text=text, variable=self.walk_var, value=value,
                           command=self._modes_changed).pack(side='left')
        tk.Label(modes, text='    place:').pack(side='left')
        for text, value in (('auto', True), ('by button', False)):
            tk.Radiobutton(modes, text=text, variable=self.auto_place_var, value=value,
                           command=self._modes_changed).pack(side='left')
        self.status = tk.StringVar(value=self.status_text())
        tk.Label(root, textvariable=self.status, anchor='w').pack(
            fill='x', padx=8, pady=(0, 8))
        root.protocol('WM_DELETE_WINDOW', self._close_panel)
        self.root = root

    def _modes_changed(self):
        self.walk = bool(self.walk_var.get())
        self.auto_place = bool(self.auto_place_var.get())

    def _log_refusal(self, error):
        if error:
            self.get_logger().warn(error)

    def _close_panel(self):
        if self.root is not None:
            self.root.destroy()
            self.root = None
        self.running = False            # closing the window quits, as q does

    def on_tick(self):
        if self.root is None:
            return
        self.status.set(self.status_text())
        # The switches follow the services too (Tk is only touched here,
        # on the main thread; the services run on the executor's).
        if self.walk_var.get() != self.walk:
            self.walk_var.set(self.walk)
        if self.auto_place_var.get() != self.auto_place:
            self.auto_place_var.set(self.auto_place)
        self.place_button.configure(
            state='normal' if self.phase == CARRY else 'disabled')
        try:
            if self.view is not None:
                self.photo = tkview.photo_from_bgr(self.view, max_width=960)
                self.image_label.configure(image=self.photo)
                self.view = None
            self.root.update()
        except tk.TclError:
            self.root = None

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

        target = self._choose_target(depth_mm)
        if self.phase == SEARCH and self.undock_from is not None:
            self._undock()
        if self.phase in (SEARCH, APPROACH, TRACK) and target is not None:
            self.last_seen = time.monotonic()
            u, v, radius = target
            u, v = min(u, width - 1), min(v, height - 1)
            cv2.circle(frame, (int(u), int(v)), int(radius),
                       RANGE_RGB.get(self.color, GREY), 2)
            self._follow(u, v, width, height)
            if self.phase == SEARCH:
                if self.undock_from is None:        # backed off; go
                    self.phase = APPROACH
                    self.get_logger().info(f'{self.color} found; walking to it')
            elif self.phase == APPROACH:
                self._walk(u, v, depth_mm)
            elif self._held_still():
                self._pick(u, v, depth_mm)
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
            if self.phase == SEARCH:
                self._sweep()
            elif self.phase == APPROACH:
                self._lost(depth_mm)
        self.view = np.concatenate([frame, view], axis=1)
        return self.view

    def _choose_target(self, depth_mm):
        """The nearest blob of the colour that is up on a pedestal.

        Upstream takes the leftmost contour; here the room's decorative
        twin of each block lies on the floor a little further off, at the
        same image x, and the leftmost rule picked it half the time.
        """
        nearest = None
        for target in self.targets:
            u, v, radius = target
            point = self.localize(u, v, depth_mm)
            if point is None or not PEDESTAL_Z[0] <= point[2] <= PEDESTAL_Z[1]:
                continue
            depth = self._depth_m(u, v, depth_mm)
            size = 2 * radius * depth / self.intrinsics[0]
            if not BLOCK_SIZE[0] <= size <= BLOCK_SIZE[1]:
                continue
            above = self._depth_m(u, max(0, v - 2 * radius), depth_mm)
            if above is not None and above < depth + FREE_BEHIND:
                continue
            if nearest is None or point[0] < nearest[0][0]:
                nearest = (point, target)
        return None if nearest is None else nearest[1]

    @staticmethod
    def _depth_m(u, v, depth_mm):
        depth = depth_probe.patch_depth(depth_mm, u, v)
        return None if depth is None else depth / 1000.0

    def localize(self, u, v, depth_mm):
        """Pixel + depth -> (x, y, z) in base_footprint, or None."""
        depth = self._depth_m(u, v, depth_mm)
        if depth is None or self.intrinsics is None:
            return None
        point = depth_probe.camera_point(u, v, depth, self.intrinsics)
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_footprint', 'depth_cam_frame', rclpy.time.Time())
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(f'no transform: {exc}', throttle_duration_sec=2.0)
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return depth_probe.transform_point(point, (t.x, t.y, t.z), (r.x, r.y, r.z, r.w))

    def _follow(self, u, v, width, height):
        """Two PIDs, servo 19 for x and 22 for y, as upstream color_track."""
        self.pid_pan.SetPoint = width / 2.0
        self.pid_pan.update(u)
        self.pan = set_range(self.pan + self.pid_pan.output, *PAN_PULSE)
        self.pid_tilt.SetPoint = height / 2.0
        self.pid_tilt.update(v)
        self.tilt = set_range(self.tilt + self.pid_tilt.output, *TILT_PULSE)
        self.servos.set_servo_position(0.02, ((19, int(self.pan)),
                                              (22, int(self.tilt))))

    def _odom_xy(self):
        """The body's (x, y) in odom, or None before TF has it."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    def _undock(self):
        """Straight back until UNDOCK_M from where the walk began."""
        here = self._odom_xy()
        if self.undock_from[0] is None:
            self.undock_from = here or self.undock_from
            return
        if here is None:
            return
        if math.dist(here, self.undock_from) < UNDOCK_M:
            self.drive(-UNDOCK_SPEED)
            return
        self.stop()
        self.undock_from = None
        self.get_logger().info('backed off the pedestal')

    def _sweep(self):
        """Pan left and right at SEARCH_TILT, the body turning under it
        once it has backed off, until the colour shows."""
        self.pan += self.sweep_dir * SEARCH_STEP
        if abs(self.pan - 500) >= SEARCH_PAN:
            self.sweep_dir = -self.sweep_dir
            self.pan = 500 + self.sweep_dir * -SEARCH_PAN
        self.servos.set_servo_position(0.1, ((19, int(self.pan)),
                                             (22, int(self.tilt))))
        if self.undock_from is None:
            self.drive(0.0, SEARCH_TURN)

    def _walk(self, u, v, depth_mm):
        """One step of the approach: the block's point in base_footprint
        into a body twist, until it sits at the pedestal spot in front."""
        point = self.localize(u, v, depth_mm)
        if point is None:
            return
        vx, vy, wz, done = self.approach.step(point[0], point[1])
        self.drive(vx, wz, vy)
        if done:
            self.phase = TRACK
            self.still_since = time.monotonic()
            self.get_logger().info(
                f'{self.color} at ({point[0]:.2f}, {point[1]:.2f}); stopping to pick')

    def _lost(self, depth_mm):
        """The block dropped out of view while walking: hold, then search."""
        self.stop()
        if time.monotonic() - self.last_seen > self.lost_timeout:
            self.phase = SEARCH
            self.pan, self.tilt = 500.0, float(SEARCH_TILT)
            self.get_logger().info(f'lost {self.color}; searching again')

    def _held_still(self):
        """True once the camera has moved under SETTLE_PULSES for
        SETTLE_SECONDS: the block is centred and not going anywhere."""
        pan, tilt = self.last_pose
        if abs(pan - self.pan) >= SETTLE_PULSES or abs(tilt - self.tilt) >= SETTLE_PULSES:
            self.still_since = time.monotonic()
        self.last_pose = (self.pan, self.tilt)
        return time.monotonic() - self.still_since > SETTLE_SECONDS

    def _pick(self, u, v, depth_mm):
        """Hand the block to pick_and_place; the job ends on its DONE."""
        self.still_since = time.monotonic()
        point = self.localize(u, v, depth_mm)
        if point is not None and (point[0] > REACH_X or abs(point[1]) > REACH_X / 2):
            self.get_logger().warn(
                f'{self.color} is {math.hypot(point[0], point[1]):.2f} m off, out of '
                'reach from here; switch to "walk to it"', throttle_duration_sec=5.0)
            return
        if not self.pick_client.service_is_ready():
            self.get_logger().warn(
                '/pick_and_place/pick is not there: start pick_place.launch.py '
                'for the grab; tracking only', throttle_duration_sec=5.0)
            return
        self.get_logger().info(f'stop -- picking {self.color}')
        self.phase = PICK
        self.pick_state = 'LOOK'        # busy until the node reports back
        self.pick_client.call_async(SetString.Request(data=self.color))
        self.pan, self.tilt = 500.0, float(LEVEL_POSE[3][1])
        self.pid_pan.clear()
        self.pid_tilt.clear()

    def on_stop(self):
        self.stop()
        self._close_panel()
        self.servos.set_servo_position(1.5, LEVEL_POSE)


def main():
    vision_demo.main(TrackAndGrabNode)


if __name__ == '__main__':
    main()
