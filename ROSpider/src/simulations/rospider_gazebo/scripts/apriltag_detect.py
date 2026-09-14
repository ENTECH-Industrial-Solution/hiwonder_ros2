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

The node can also act on tags (rospider_gazebo/tag_behavior.py): approach a
tag and stop at a standoff, place the held cube on its station, or stop.
Off by default; switched on and configured live in the Tk tuner that
tune:=true opens, and saved to a JSON that overrides config/apriltag.yaml
the way color_detect's tuned HSV file does.

The maths lives in rospider_gazebo/tags.py so it can be tested without a
simulator; this file is only the ROS plumbing.
"""

import json
import os
import threading
import time
import traceback
from copy import deepcopy
from pathlib import Path

import tkinter as tk
from tkinter import ttk

import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, Twist
from interfaces.msg import ApriltagInfo, ApriltagsInfo
from interfaces.srv import SetString
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rospider_gazebo import labelling, tag_settings, tags, tkview
from rospider_gazebo.ros_image import to_image_msg
from rospider_gazebo.tag_behavior import TagBehavior
from sensor_msgs.msg import CameraInfo, Image

TUNE_TITLE = 'apriltag_detect tune'

# (key, label, from, to, resolution) for the detector sliders; corner
# refinement is a pair of radio buttons instead.
DETECTOR_SLIDERS = (
    ('adaptive_thresh_win_size_min', 'thresh win min', 3, 51, 2),
    ('adaptive_thresh_win_size_max', 'thresh win max', 3, 101, 2),
    ('adaptive_thresh_win_size_step', 'thresh win step', 1, 50, 1),
    ('adaptive_thresh_constant', 'thresh constant', 0.0, 30.0, 0.5),
    ('min_marker_perimeter_rate', 'min perimeter rate', 0.005, 0.2, 0.005),
    ('polygonal_approx_accuracy_rate', 'polygon accuracy', 0.01, 0.2, 0.005),
)
CONTROL_FIELDS = tuple(tag_settings.CONTROL_DEFAULTS)


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
        self.publish_tf = bool(self._param('publish_tf', True))
        self.draw = bool(self._param('draw', True))
        self.tune = bool(self._param('tune', False))
        self.tuned_path = Path(os.path.expanduser(
            str(self._param('tuned_path', '~/.ros/apriltag_tuned.json'))))
        image_topic = str(self._param('image_topic',
                                      '/depth_cam/rgb/image_raw'))

        # Shared with the tuner thread: it reads the last overlay frame and
        # the status, and writes settings through apply_settings().
        self._lock = threading.Lock()
        self.latest_frame = None
        self.seen_ids = set()
        self.status = 'no tag'
        self.place_response = ''
        self.memory_frame = str(self._param('memory_frame', 'odom'))
        self.memory = {}          # tag id -> (R, t) of the tag in memory_frame

        self.baseline = tag_settings.from_flat(self._flat_params())
        self.behavior = TagBehavior(
            self.baseline['control'], self.baseline['behaviors'],
            enabled=bool(self._param('behaviors_enabled', False)))
        # A copy, not self.baseline itself: apply_settings stores whatever
        # it is given as self.settings, and a later live edit (parameter
        # set, tuner slider) would otherwise mutate the baseline too, so
        # Revert would have nothing to revert to.
        self.apply_settings(json.loads(json.dumps(self.baseline)))
        self._load_tuned()

        self.camera_matrix = None
        self.dist_coeffs = None

        self.tags_pub = self.create_publisher(ApriltagsInfo,
                                              '~/apriltag_info', 1)
        self.image_pub = self.create_publisher(Image, '~/image_result', 1)
        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.place_client = self.create_client(SetString,
                                               '/pick_and_place/place')
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, image_topic, self.image_callback, 1)
        self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info',
                                 self.info_callback, 1)
        self.add_on_set_parameters_callback(self._on_parameters)
        self.get_logger().info(
            f'watching for {tags.FAMILY} tags of {self.tag_size} m on '
            f'{image_topic}; behaviours '
            f'{"ENABLED" if self.behavior.enabled else "disabled"}')

    def _param(self, name, default):
        """Read a parameter, falling back when the YAML does not carry it.

        Parameters are declared from overrides, so anything absent from
        config/apriltag.yaml comes back as None rather than raising. Same
        helper, same reason, as color_detect.py.
        """
        value = self.get_parameter(name).value
        return default if value is None else value

    def _flat_params(self):
        """Every declared parameter as {dotted_name: value}: the nested YAML
        arrives as one flat parameter per leaf, and an empty prefix matches
        them all."""
        return {name: param.value for name, param
                in self.get_parameters_by_prefix('').items()}

    # ------------------------------------------------------------- settings

    def apply_settings(self, settings):
        """Adopt a settings dict as the live detector and behaviour config.

        Validates by building the aruco parameters first, so a bad value
        from the GUI or the JSON is rejected before anything changes.
        """
        params = tags.detector_parameters(settings['detector'])
        with self._lock:
            self.settings = settings
            self.detector_params = params
            self.max_reproj_error = float(settings['max_reproj_error_px'])
            self.behavior.configure(settings['control'],
                                    settings['behaviors'])

    def _load_tuned(self):
        """Overlay the tuned JSON on the YAML baseline, if the file exists.

        Precedence is deliberately one-way and logged: the YAML is the
        committed truth, the JSON is a tuning override. Saying which one is
        live keeps a forgotten JSON from silently shadowing the YAML.
        """
        if not self.tuned_path.is_file():
            self.get_logger().info(
                f'settings from config/apriltag.yaml '
                f'(no tuned file at {self.tuned_path})')
            return
        try:
            with self.tuned_path.open() as handle:
                override = json.load(handle)
            self.apply_settings(tag_settings.merge(self.baseline, override))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.get_logger().error(
                f'ignoring unreadable tuned file {self.tuned_path}: {exc}; '
                'using config/apriltag.yaml')
            return
        self.get_logger().warn(
            f'settings OVERRIDDEN by {self.tuned_path} '
            '(delete it to go back to config/apriltag.yaml)')

    def save_tuned(self):
        """Write the live settings to the tuned JSON; returns a message."""
        with self._lock:
            settings = json.loads(json.dumps(self.settings))
        try:
            self.tuned_path.parent.mkdir(parents=True, exist_ok=True)
            with self.tuned_path.open('w') as handle:
                json.dump(settings, handle, indent=2)
                handle.write('\n')
        except OSError as exc:
            message = f'could not save {self.tuned_path}: {exc}'
            self.get_logger().error(message)
            return message
        message = f'saved {self.tuned_path}'
        self.get_logger().info(message)
        return message

    def revert(self):
        self.apply_settings(json.loads(json.dumps(self.baseline)))
        self.get_logger().info('reverted to the config/apriltag.yaml baseline')

    def _on_parameters(self, params):
        """Apply `ros2 param set` changes live.

        The tuner is one way to switch behaviours on and change a tag's
        action; this is the other, for a script or another node (the
        combined Nav2 launch will need it). Only the behaviour switch and
        the four settings sections are live; everything else is read once
        at startup. The GUI does not re-read these, so a slider touched
        afterwards re-applies whatever its widgets hold.
        """
        override = {}
        enabled = None      # applied last, only once the rest is known-good
        for param in params:
            name, value = param.name, param.value
            if name == 'behaviors_enabled':
                enabled = bool(value)
            elif name == 'max_reproj_error_px':
                override[name] = value
            else:
                parts = name.split('.')
                if parts[0] in ('detector', 'control') and len(parts) == 2:
                    override.setdefault(parts[0], {})[parts[1]] = value
                elif parts[0] == 'behaviors' and len(parts) == 3:
                    override.setdefault('behaviors', {}).setdefault(
                        parts[1], {})[parts[2]] = value
        if override:
            try:
                with self._lock:
                    current = deepcopy(self.settings)
                self.apply_settings(tag_settings.merge(current, override))
            except (KeyError, ValueError, TypeError) as exc:
                # A batch with both a bad value and behaviors_enabled must
                # not flip the switch while rejecting the rest -- that
                # would start the robot moving with settings the caller
                # was just told were refused.
                return SetParametersResult(successful=False,
                                           reason=str(exc))
            self.get_logger().info(f'settings changed by parameter: '
                                   f'{override}')
        if enabled is not None:
            with self._lock:
                self.behavior.enabled = enabled
            self.get_logger().info(
                f'behaviours {"ENABLED" if enabled else "disabled"} '
                '(parameter)')
        return SetParametersResult(successful=True)

    def yaml_block(self):
        with self._lock:
            return tag_settings.yaml_block(self.settings)

    # ------------------------------------------------------------ detection

    def info_callback(self, msg):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        if self.dist_coeffs.size == 0:
            self.dist_coeffs = np.zeros(5)

    def image_callback(self, msg):
        try:
            self._process_image(msg)
        except Exception:
            # rclpy's logger has no exc_info kwarg (unlike stdlib logging);
            # passing one raises TypeError from inside this except block,
            # which would otherwise crash the whole spin thread instead of
            # just skipping the bad frame as intended.
            self.get_logger().error(
                'image_callback failed on this frame; skipping it:\n'
                + traceback.format_exc())

    def _process_image(self, msg):
        if self.camera_matrix is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        with self._lock:
            params = self.detector_params
            max_error = self.max_reproj_error

        info = ApriltagsInfo()
        transforms = []
        poses = {}          # tag id -> (rvec, tvec), for the behaviour
        for tag_id, corners in tags.detect_tags(gray, params):
            solved = tags.solve_tag_pose(
                corners, self.camera_matrix, self.dist_coeffs, self.tag_size)
            if solved is None:
                continue
            rvec, tvec, error = solved
            if not np.all(np.isfinite(tvec)) or not np.all(np.isfinite(rvec)):
                # A transient NaN solve was seen in the sim; drop just this
                # tag rather than the whole frame's behaviour update.
                continue
            if error > max_error:
                # The two IPPE_SQUARE solutions are both poor; whichever won
                # the argmin is not trustworthy. Drop it rather than publish a
                # frame that may be the mirror pose.
                continue

            poses[tag_id] = (rvec, tvec)
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

        self._remember(poses, msg.header)
        recalled = self._recall(poses, msg.header)
        self._act({**recalled, **poses}, msg.header, set(recalled))
        with self._lock:
            self.seen_ids.update(poses)
            self.latest_frame = frame

    # ----------------------------------------------------------- behaviours

    def _act(self, poses, header, virtual_ids=frozenset()):
        """Run the behaviour on this frame's tags and publish what it says.

        Wall-clock time, not the image stamp: lost_timeout is about how long
        the robot holds still for a flickering detection, which is a
        real-time question, and a paused simulation must not freeze it.
        """
        # Steer in a level frame at the robot's base, not in the camera's.
        # The camera is on the wrist: in CARRY it points down and sideways,
        # and a controller working in camera x/z then "reaches" the standoff
        # somewhere meaningless. (-Y, -Z, X) of the base_footprint position
        # is the optical convention TagBehavior speaks (x right, z forward),
        # measured from the base along the heading.
        mapped = self._tf_matrix('base_footprint', header.frame_id)
        if mapped is None:
            if poses:
                self.get_logger().warn(
                    f'no TF base_footprint <- {header.frame_id}; not '
                    'steering this frame', throttle_duration_sec=5.0)
            # Still run the behaviour, on an empty frame: returning here
            # would leave whatever twist was last published standing
            # (hardware's move_controller has no watchdog), skipping the
            # hold -> one zero -> release sequence _lost() exists for.
            # Never steer in raw camera coordinates -- an empty frame is
            # the only thing given to the behaviour when the TF is down.
            rot_bc = t_bc = None
            sightings = []
        else:
            rot_bc, t_bc = mapped
            sightings = []
            for tag_id, (_, tvec) in poses.items():
                base = rot_bc @ tvec.ravel() + t_bc
                sightings.append(
                    (tag_id, np.array([-base[1], -base[2], base[0]])))
        with self._lock:
            decision = self.behavior.update(time.monotonic(), sightings)
            self.status = decision.status + ('' if mapped is not None
                                             else ' (no TF)')
            if self.behavior.target in virtual_ids:
                self.status += ' [remembered]'
        if decision.twist is not None:
            twist = Twist()
            twist.linear.x, twist.angular.z = (float(v)
                                               for v in decision.twist)
            self.cmd_pub.publish(twist)
        if decision.place is not None:
            # Only reachable when sightings was non-empty, which requires
            # mapped to be set, so rot_bc/t_bc are real here.
            self.get_logger().info(decision.status)
            self._request_place(decision.place, *poses[decision.place],
                                rot_bc, t_bc)

    def _tf_matrix(self, target, source):
        """(R, t) of `source` expressed in `target`, latest, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(target, source,
                                                 rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        q = tf.transform.rotation
        t = tf.transform.translation
        return (labelling.rotation_matrix((q.x, q.y, q.z, q.w)),
                np.array([t.x, t.y, t.z]))

    def _remember(self, poses, header):
        """Store every tag seen this frame as a pose in memory_frame."""
        if not self.memory_frame or not poses:
            return
        mapped = self._tf_matrix(self.memory_frame, header.frame_id)
        if mapped is None:
            return
        # Uses the latest TF, not header.stamp: the arm's camera TF lags
        # joint_states, so a pose remembered while the arm is moving is
        # slightly smeared. Accepted rather than fixed, because the next
        # clean sighting (arm holding still) just replaces this entry.
        rot_mc, t_mc = mapped
        updates = {}
        for tag_id, (rvec, tvec) in poses.items():
            rot_ct, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
            updates[tag_id] = (rot_mc @ rot_ct, rot_mc @ tvec.ravel() + t_mc)
        with self._lock:
            self.memory.update(updates)

    def _recall(self, poses, header):
        """Virtual sightings: remembered approach/place tags not seen now.

        Mapped from memory_frame into the camera frame so the behaviour and
        _request_place see exactly what a live detection would give them.
        `stop` tags are never recalled: a remembered stop sign would freeze
        the robot forever.
        """
        with self._lock:
            enabled = self.behavior.enabled
            wanted = [tag_id for tag_id in self.memory
                      if tag_id not in poses
                      and self.behavior.action(tag_id) in ('approach', 'place')]
            memory = {tag_id: self.memory[tag_id] for tag_id in wanted}
        if not enabled or not memory:
            return {}
        mapped = self._tf_matrix(header.frame_id, self.memory_frame)
        if mapped is None:
            return {}
        rot_cm, t_cm = mapped
        recalled = {}
        for tag_id, (rot_mt, t_mt) in memory.items():
            rvec, _ = cv2.Rodrigues(rot_cm @ rot_mt)
            tvec = (rot_cm @ t_mt + t_cm).reshape(3, 1)
            recalled[tag_id] = (rvec, tvec)
        return recalled

    def _request_place(self, tag_id, rvec, tvec, rot_bc, t_bc):
        """Ask pick_and_place to put the held cube on this tag's pedestal.

        The pedestal top is known in the tag frame (tags.tag_to_pedestal_top)
        and the tag pose is known in the camera frame, so the point goes tag
        -> camera -> base_footprint, the frame ~/place reads. rot_bc/t_bc is
        the base_footprint <- camera TF _act already looked up for this same
        frame; reusing it here avoids a second, redundant lookup. No retry:
        a "not reachable" answer means the standoff is too long, and that is
        a number for the user to change in the tuner.
        """
        rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
        point_cam = rotation @ tags.tag_to_pedestal_top() + tvec.ravel()
        point = rot_bc @ point_cam + t_bc
        with self._lock:
            point[2] += float(self.settings['control']['place_height'])

        if not self.place_client.service_is_ready():
            self._note_place('/pick_and_place/place is not available; is '
                             'pick_place.launch.py running?')
            return
        request = SetString.Request()
        request.data = f'{point[0]:.3f} {point[1]:.3f} {point[2]:.3f}'
        self._note_place(f'tag {tag_id}: ~/place {request.data} ...')
        future = self.place_client.call_async(request)
        future.add_done_callback(
            lambda done: self._note_place(
                f'tag {tag_id}: ~/place -> '
                + (done.result().message if done.exception() is None
                   else f'failed: {done.exception()}')))

    def _note_place(self, message):
        self.get_logger().info(message)
        with self._lock:
            self.place_response = message

    # ---------------------------------------------------------------- tuner

    def run_tuner(self):
        """Tk window. Runs on the main thread; Tk requires that.

        Detection and behaviours keep running throughout, so a slider's
        effect is visible on the overlay and on /controller/cmd_vel at once.
        Closing the window leaves the node running with the last values.
        """
        root = tk.Tk()
        root.title(TUNE_TITLE)
        root.columnconfigure(0, weight=1)

        image_label = ttk.Label(root)
        image_label.grid(row=0, column=0, rowspan=2, padx=6, pady=6,
                         sticky='n')
        side = ttk.Frame(root)
        side.grid(row=0, column=1, padx=6, pady=6, sticky='n')
        advanced = ttk.Frame(side)

        with self._lock:
            settings = json.loads(json.dumps(self.settings))
            enabled = self.behavior.enabled

        # ---- detector
        detector_box = ttk.LabelFrame(advanced, text='Detector')
        detector_box.grid(row=0, column=0, sticky='ew', pady=(6, 0))
        detector_vars = {}
        for key, label, lo, hi, step in DETECTOR_SLIDERS:
            var = tk.DoubleVar(value=float(settings['detector'][key]))
            detector_vars[key] = var
            tkview.LabeledScale(detector_box, label, var, lo, hi, step,
                                self._tuner_apply).pack(fill='x')
        refinement = tk.StringVar(value=settings['detector']['corner_refinement'])
        detector_vars['corner_refinement'] = refinement
        row = ttk.Frame(detector_box)
        row.pack(fill='x')
        ttk.Label(row, text='corner refinement').pack(side='left')
        for choice in ('none', 'subpix'):
            ttk.Radiobutton(row, text=choice, value=choice,
                            variable=refinement,
                            command=self._tuner_apply).pack(side='left')
        reproj = tk.DoubleVar(value=float(settings['max_reproj_error_px']))
        tkview.LabeledScale(detector_box, 'max reproj error px', reproj,
                            0.5, 10.0, 0.1, self._tuner_apply).pack(fill='x')

        # ---- tags
        tags_box = ttk.LabelFrame(side, text='Tags')
        tags_box.grid(row=0, column=0, sticky='ew')
        ttk.Label(tags_box, text='id').grid(row=0, column=0)
        ttk.Label(tags_box, text='action').grid(row=0, column=1)
        ttk.Label(tags_box, text='standoff m').grid(row=0, column=2)
        tag_rows = {}       # tag id -> (action StringVar, standoff StringVar)

        def add_row(tag_id, row_settings):
            if tag_id in tag_rows:
                return
            action = tk.StringVar(value=row_settings['action'])
            standoff = tk.StringVar(value=str(row_settings['standoff']))
            tag_rows[tag_id] = (action, standoff)
            r = len(tag_rows)
            ttk.Label(tags_box, text=str(tag_id)).grid(row=r, column=0)
            box = ttk.Combobox(tags_box, textvariable=action, width=9,
                               values=tag_settings.ACTIONS, state='readonly')
            box.grid(row=r, column=1)
            box.bind('<<ComboboxSelected>>', lambda _e: self._tuner_apply())
            entry = ttk.Entry(tags_box, textvariable=standoff, width=7)
            entry.grid(row=r, column=2)
            entry.bind('<Return>', lambda _e: self._tuner_apply())
            entry.bind('<FocusOut>', lambda _e: self._tuner_apply())

        for key in sorted(settings['behaviors'], key=tag_settings.tag_id):
            add_row(tag_settings.tag_id(key), settings['behaviors'][key])

        add_box = ttk.Frame(side)
        add_box.grid(row=1, column=0, sticky='ew')
        new_id = tk.StringVar()
        ttk.Entry(add_box, textvariable=new_id, width=6).pack(side='left')

        def add_typed():
            try:
                tag_id = int(new_id.get())
                tag_settings.tag_key(tag_id)   # rejects a negative id early
            except ValueError:
                # Without this, a negative id still reaches add_row: its
                # key ('tag-1') fails tag_settings' own pattern, and every
                # later read_widgets() -> merge() then raises on that row
                # forever, so no widget change -- anywhere -- ever applies
                # again.
                message.set('tag id must be a non-negative integer')
                return
            add_row(tag_id, {'action': 'none',
                             'standoff': tag_settings.DEFAULT_STANDOFF})
            new_id.set('')
            self._tuner_apply()
        ttk.Button(add_box, text='add id', command=add_typed).pack(side='left')

        # ---- control
        control_box = ttk.LabelFrame(advanced, text='Control')
        control_box.grid(row=1, column=0, sticky='ew', pady=(6, 0))
        control_vars = {}
        for i, key in enumerate(CONTROL_FIELDS):
            ttk.Label(control_box, text=key).grid(row=i, column=0, sticky='w')
            var = tk.StringVar(value=str(settings['control'][key]))
            control_vars[key] = var
            entry = ttk.Entry(control_box, textvariable=var, width=8)
            entry.grid(row=i, column=1)
            entry.bind('<Return>', lambda _e: self._tuner_apply())
            entry.bind('<FocusOut>', lambda _e: self._tuner_apply())
        enabled_var = tk.BooleanVar(value=enabled)

        def toggle():
            with self._lock:
                self.behavior.enabled = enabled_var.get()
            self.get_logger().info(
                f'behaviours {"ENABLED" if enabled_var.get() else "disabled"}')
        ttk.Checkbutton(side, text='Enable behaviors',
                        variable=enabled_var, command=toggle).grid(
            row=2, column=0, sticky='w', pady=(6, 0))
        status = ttk.Label(side, text='', wraplength=320, justify='left')
        status.grid(row=3, column=0, sticky='ew', pady=(6, 0))

        # ---- buttons
        buttons = ttk.Frame(side)
        buttons.grid(row=4, column=0, sticky='ew', pady=(6, 0))
        message = tk.StringVar()

        def do_save():
            message.set(self.save_tuned())

        def do_revert():
            self.revert()
            with self._lock:
                base = json.loads(json.dumps(self.settings))
            for key, var in detector_vars.items():
                var.set(base['detector'][key])
            reproj.set(base['max_reproj_error_px'])
            for key, var in control_vars.items():
                var.set(str(base['control'][key]))
            for tag_id, (action, standoff) in tag_rows.items():
                row = base['behaviors'].get(
                    tag_settings.tag_key(tag_id),
                    {'action': 'none',
                     'standoff': tag_settings.DEFAULT_STANDOFF})
                action.set(row['action'])
                standoff.set(str(row['standoff']))
            message.set('reverted to config/apriltag.yaml')

        def do_yaml():
            self.get_logger().info('current settings as YAML:\n'
                                   + self.yaml_block())
            message.set('YAML printed to the terminal')
        ttk.Button(buttons, text='Save', command=do_save).pack(side='left')
        ttk.Button(buttons, text='Revert', command=do_revert).pack(side='left')
        ttk.Button(buttons, text='Print YAML', command=do_yaml).pack(side='left')
        ttk.Label(side, textvariable=message).grid(row=5, column=0, sticky='w')

        # Workshop view: tags, the switch, and the buttons. The detector
        # sliders and the controller gains are one click away, not in the
        # way -- a ten-minute demo needs the first three, and a puzzled
        # afternoon needs the rest.
        advanced.grid(row=7, column=0, sticky='ew')
        advanced.grid_remove()
        advanced_shown = tk.BooleanVar(value=False)

        def toggle_advanced():
            if advanced_shown.get():
                advanced.grid_remove()
                advanced_button.configure(text='Advanced ▸')
            else:
                advanced.grid()
                advanced_button.configure(text='Advanced ▾')
            advanced_shown.set(not advanced_shown.get())
        advanced_button = ttk.Button(side, text='Advanced ▸',
                                     command=toggle_advanced)
        advanced_button.grid(row=6, column=0, sticky='w', pady=(6, 0))

        # The widgets are the source of truth for the GUI; this reads them
        # all back into one settings dict and applies it. Any invalid entry
        # is reported in the message line and the previous settings stay.
        def read_widgets():
            new = {
                'detector': {key: var.get() for key, var
                             in detector_vars.items()},
                'max_reproj_error_px': reproj.get(),
                'control': {key: float(var.get()) for key, var
                            in control_vars.items()},
                'behaviors': {
                    tag_settings.tag_key(tag_id): {
                        'action': action.get(),
                        'standoff': float(standoff.get())}
                    for tag_id, (action, standoff) in tag_rows.items()},
            }
            for key in ('adaptive_thresh_win_size_min',
                        'adaptive_thresh_win_size_max',
                        'adaptive_thresh_win_size_step'):
                new['detector'][key] = int(round(new['detector'][key]))
            return tag_settings.merge(tag_settings.defaults(), new)
        self._tuner_read = read_widgets
        self._tuner_message = message

        def refresh():
            with self._lock:
                frame = self.latest_frame
                text = self.status
                response = self.place_response
                seen = sorted(self.seen_ids)
            if frame is not None:
                photo = tkview.photo_from_bgr(frame)
                image_label.configure(image=photo)
                image_label.image = photo      # keep it alive
            status.configure(text=f'{text}\n{response}')
            for tag_id in seen:
                add_row(tag_id, {'action': 'none',
                                 'standoff': tag_settings.DEFAULT_STANDOFF})
            root.after(50, refresh)
        refresh()

        self.get_logger().info(
            f'tuning window open: Save writes {self.tuned_path}, Revert '
            'reloads config/apriltag.yaml, Print YAML logs a paste-ready '
            'block; behaviours run only while "Enable behaviors" is ticked')
        tkview.mainloop_until_shutdown(root)
        self.get_logger().info('tuning window closed; still detecting')

    def _tuner_apply(self):
        try:
            self.apply_settings(self._tuner_read())
        except (ValueError, KeyError, tk.TclError) as exc:
            self._tuner_message.set(f'not applied: {exc}')
            return
        self._tuner_message.set('')

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
        if node.tune:
            # Tk must own the main thread, so spin moves to a worker.
            spinner = threading.Thread(target=rclpy.spin, args=(node,),
                                       daemon=True)
            spinner.start()
            node.run_tuner()
            spinner.join()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
