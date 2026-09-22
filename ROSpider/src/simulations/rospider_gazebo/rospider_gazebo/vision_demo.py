"""The plumbing every ported Hiwonder demo window needs.

Upstream's demos are all built the same way: subscribe to the camera, push
frames into a two-deep queue, and run a worker loop that draws an overlay,
cv2.imshow()es it and quits on q or Esc. VisionDemo is that skeleton, so each
ported script is only the part that differs -- what it draws and what it does
with the robot.

What a window shows is exactly what the upstream script draws: the same
primitives, colours (translated to BGR), fonts and window titles. States
the simulation has and the robot does not -- waiting for a topic, a
calibration pass, a lost target -- are logged, never drawn.

Three things differ from upstream and are the same in every ported demo:

  * The display loop owns the main thread and rclpy.spin() runs on a worker,
    rather than the other way round. highgui needs the main thread, and doing
    it this way means Ctrl-C in the terminal reaches rclpy.
  * Robot commands go to the simulated controllers. ServoClient takes
    upstream's servo pulses and publishes radians on the joint trajectory
    topics; see rospider_gazebo/servo_map.py.
  * Every window is also published on ~/image_result, so a demo can be
    watched over the network (rqt_image_view, web_video_server) or run with
    show:=false on a machine with no display.
"""

import os
import queue
import threading
import time

# The pip-installed opencv-python bundles a Qt build with no fonts of its own,
# and highgui then draws window text as blank strips. Point Qt at the system
# fonts before cv2 is imported, which is when Qt initialises; an existing
# value wins. Same fix as scripts/color_detect.py.
for _font_dir in ('/usr/share/fonts/truetype/dejavu',
                  '/usr/share/fonts/truetype/liberation'):
    if os.path.isdir(_font_dir):
        os.environ.setdefault('QT_QPA_FONTDIR', _font_dir)
        break

import cv2  # noqa: E402  (must follow the QT_QPA_FONTDIR default above)
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rospider_gazebo import servo_map  # noqa: E402
from rospider_gazebo.depth_probe import to_millimetres  # noqa: E402
from rospider_gazebo.ros_image import to_image_msg  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, JointState  # noqa: E402
from std_msgs.msg import Header  # noqa: E402
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # noqa: E402

RGB_TOPIC = '/depth_cam/rgb/image_raw'
DEPTH_TOPIC = '/depth_cam/depth/image_raw'
CMD_VEL_TOPIC = '/controller/cmd_vel'

QUIT_KEYS = (ord('q'), 27)


class FPS:
    """Smoothed frame rate with upstream's overlay (driver/sdk/sdk/fps.py)."""

    def __init__(self, confidence=0.1):
        self.fps = 0.0
        self.confidence = confidence
        self.last_time = 0.0
        self.current_time = 0.0

    def update(self):
        self.last_time = self.current_time
        self.current_time = time.time()
        elapsed = self.current_time - self.last_time
        if elapsed <= 0:
            return self.fps
        new_fps = 1.0 / elapsed
        if self.fps == 0.0:
            self.fps = new_fps if self.last_time != 0 else 0.0
        else:
            self.fps = new_fps * self.confidence + self.fps * (1 - self.confidence)
        return float(self.fps)

    def show_fps(self, image):
        text = 'FPS: {:.2f}'.format(self.fps)
        cv2.putText(image, text, (11, 20), cv2.FONT_HERSHEY_PLAIN, 1.0,
                    (32, 32, 32), 4, cv2.LINE_AA)
        cv2.putText(image, text, (10, 20), cv2.FONT_HERSHEY_PLAIN, 1.0,
                    (240, 240, 240), 1, cv2.LINE_AA)
        return image


def depth_color_map(depth_mm, ceiling=None):
    """A depth frame as the JET colour map upstream shows.

    `ceiling` is the distance in millimetres that saturates the map; left
    out, upstream's convertScaleAbs(alpha=0.45) scaling is used, which
    saturates around 570 mm.
    """
    if ceiling is None:
        gray = cv2.convertScaleAbs(depth_mm, alpha=0.45)
    else:
        gray = np.clip(depth_mm, 0, ceiling).astype(np.float64) / ceiling * 255
        gray = gray.astype(np.uint8)
    return cv2.applyColorMap(gray, cv2.COLORMAP_JET)


def create_tracker(kind='auto'):
    """A single-object tracker, and the name of the one actually built.

    Upstream's kcf.py asks for TrackerKCF. OpenCV moved KCF into contrib and
    then dropped it from the 5.x main module, which is the build a pip
    `opencv-python` installs, so the sim asks for KCF, then CSRT, then MIL,
    and reports what it got.
    """
    names = {'kcf': 'TrackerKCF', 'csrt': 'TrackerCSRT', 'mil': 'TrackerMIL'}
    order = ('kcf', 'csrt', 'mil') if kind == 'auto' else (kind,)
    for name in order:
        if name not in names:
            raise ValueError(f'unknown tracker {name!r}; '
                             f'expected auto, {", ".join(sorted(names))}')
        for holder in (cv2, getattr(cv2, 'legacy', None)):
            factory = getattr(holder, names[name] + '_create', None)
            if factory is not None:
                return factory(), name
    raise RuntimeError(
        f'this OpenCV build ({cv2.__version__}) has none of '
        f'{", ".join(names[n] for n in order)}')


class ServoClient:
    """Upstream's servo-pulse commands, sent to the simulated controllers.

    set_servo_position() takes exactly what
    servo_controller.bus_servo_control.set_servo_position() takes, so a
    ported demo keeps the pulses it was written with.

    Every group goes out as a full trajectory even when one joint is being
    commanded -- the pan/tilt loops send servos 19 and 22 alone --
    because joint_trajectory_controller rejects a partial goal ("Joints on
    incoming trajectory don't match the controller joints") unless
    allow_partial_joints_goal is set. The joints left out are filled from
    the last /joint_states; a partial command that arrives before the first
    joint_states is dropped rather than filled with zeros, which would
    fling the arm to its zero pose.
    """

    def __init__(self, node):
        self.node = node
        self.arm_pub = node.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 1)
        self.gripper_pub = node.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 1)
        self.leg_pub = node.create_publisher(
            JointTrajectory, '/leg_controller/joint_trajectory', 1)
        self.joint_state = {}
        node.create_subscription(JointState, '/joint_states',
                                 self._joint_state_callback, 10)

    def _joint_state_callback(self, msg):
        self.joint_state = dict(zip(msg.name, msg.position))

    def set_servo_position(self, duration, positions):
        """duration in seconds, positions as ((servo id, pulse), ...)."""
        self.set_joint_angles(servo_map.positions_to_angles(positions),
                              duration)

    def set_joint_angles(self, angles, duration=1.0):
        """angles as {joint name: radians}, moved into over `duration`."""
        groups = (
            (self.arm_pub, servo_map.ARM_JOINTS),
            (self.gripper_pub, servo_map.GRIPPER_JOINTS),
            (self.leg_pub, servo_map.LEG_JOINTS),
        )
        for publisher, joints in groups:
            if not any(j in angles for j in joints):
                continue
            missing = [j for j in joints
                       if j not in angles and j not in self.joint_state]
            if missing:
                self.node.get_logger().warn(
                    f'no /joint_states yet for {missing}; dropping a partial '
                    'command rather than filling it with zeros',
                    throttle_duration_sec=2.0)
                continue
            values = [float(angles.get(j, self.joint_state.get(j)))
                      for j in joints]
            self._publish(publisher, list(joints), values, duration)

    def _publish(self, publisher, names, positions, duration):
        # The first command usually goes out before the controller has
        # matched the publisher, and a message with no subscriber is lost;
        # a demo's start pose then never happens. Give it a moment: with
        # the whole sim up, discovery has been measured at 2 s and more.
        for _ in range(100):
            if publisher.get_subscription_count() > 0:
                break
            time.sleep(0.1)
        else:
            self.node.get_logger().warn(
                f'{publisher.topic_name} has no subscriber after 10 s; '
                'sending anyway')
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        message = JointTrajectory()
        message.joint_names = names
        message.points = [point]
        publisher.publish(message)


class VisionDemo(Node):
    """A camera demo with one OpenCV window.

    A subclass implements process(frame) -> the image to show, and may
    override on_key(), on_start() and on_stop(). run() drives the loop and
    must be called from the main thread; main() below does that.
    """

    #: Window title; the node name is used when this is left as None.
    window = None
    #: Draw upstream's FPS caption. Only the demos whose original calls
    #: fps.show_fps() turn this on.
    show_fps = False

    def __init__(self, name, *, image_topic=RGB_TOPIC, depth=False,
                 depth_frames=False, camera_info=None, flip=False,
                 cmd_vel=False, servos=False):
        """depth_frames drives the loop from the depth camera instead of the
        colour one: process() is then handed a float millimetre array rather
        than a BGR frame, which is what the two floor-probing demos want."""
        super().__init__(name,
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.bridge = CvBridge()
        self.fps = FPS()
        self.running = True
        self.window = self.window or name
        self.show = bool(self.param('show', True))
        self.flip = bool(self.param('flip', flip))
        self.depth_frames = depth_frames
        self.depth_topic = str(self.param('depth_topic', DEPTH_TOPIC))
        self.image_topic = (self.depth_topic if depth_frames
                            else str(self.param('image_topic', image_topic)))

        self._frames = queue.Queue(maxsize=2)
        self.depth_mm = None
        self.intrinsics = None

        self.result_pub = self.create_publisher(Image, '~/image_result', 1)
        if not depth_frames:
            self.create_subscription(Image, self.image_topic,
                                     self._image_callback, 1)
        if depth or depth_frames:
            self.create_subscription(Image, self.depth_topic,
                                     self._depth_callback, 1)
        if camera_info:
            self.create_subscription(CameraInfo, camera_info,
                                     self._info_callback, 1)
        self.cmd_vel_pub = self.create_publisher(
            Twist, CMD_VEL_TOPIC, 1) if cmd_vel else None
        self.servos = ServoClient(self) if servos else None

    # ------------------------------------------------------------ parameters

    def param(self, name, default):
        """A parameter's value, or `default` when the YAML does not carry it.

        Parameters are declared from overrides, so anything the config file
        leaves out reads back as None instead of raising.
        """
        value = self.get_parameter(name).value
        return default if value is None else value

    # ------------------------------------------------------------ callbacks

    def _image_callback(self, message):
        # Copy: imgmsg_to_cv2 can hand back a view onto the message's own
        # buffer, and every demo draws its overlay straight onto the frame.
        # cv2.flip already returns a new array.
        frame = self.bridge.imgmsg_to_cv2(message, 'bgr8')
        self._enqueue(cv2.flip(frame, 1) if self.flip else frame.copy())

    def _depth_callback(self, message):
        self.depth_mm = to_millimetres(
            self.bridge.imgmsg_to_cv2(message, 'passthrough'))
        if self.depth_frames:
            self._enqueue(self.depth_mm)

    def _enqueue(self, frame):
        """Keep the newest frames only; a slow process() must not build a
        backlog of stale ones."""
        if self._frames.full():
            try:
                self._frames.get_nowait()
            except queue.Empty:
                pass
        self._frames.put(frame)

    def _info_callback(self, message):
        self.intrinsics = list(message.k)

    # ------------------------------------------------------------ robot

    def drive(self, linear_x=0.0, angular_z=0.0):
        """Publish a body twist on /controller/cmd_vel."""
        if self.cmd_vel_pub is None:
            raise RuntimeError('this demo was built without cmd_vel=True')
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)

    def stop(self):
        """Stop the robot where it stands."""
        if self.cmd_vel_pub is not None:
            self.cmd_vel_pub.publish(Twist())

    # ------------------------------------------------------------ overrides

    def on_start(self):
        """Called once, before the first frame."""

    def process(self, frame):
        """Return the image to display for one camera frame."""
        raise NotImplementedError

    def on_key(self, key):
        """Handle a key press. Return True if it was handled."""
        return False

    def on_tick(self):
        """Called once per loop pass on the main thread, frame or no frame:
        for a demo's own Tk window, which must be pumped from here."""

    def on_mouse(self, event, x, y):
        """Handle a mouse event in the window; (x, y) are frame pixels."""

    def on_stop(self):
        """Called once the window closes, before the node is destroyed."""

    # ------------------------------------------------------------ main loop

    def run(self):
        """Display frames until q, Esc or shutdown. Main thread only."""
        self.on_start()
        self.get_logger().info(
            f'reading {self.image_topic}; press q or Esc in the '
            f'"{self.window}" window to quit')
        mouse_hooked = False
        while self.running and rclpy.ok():
            self.on_tick()
            try:
                frame = self._frames.get(timeout=1.0)
            except queue.Empty:
                continue
            try:
                result = self.process(frame)
            except Exception as exc:                      # noqa: BLE001
                self.get_logger().error(f'{type(exc).__name__}: {exc}')
                continue
            if result is None:
                continue
            self.fps.update()
            if self.show_fps:
                self.fps.show_fps(result)
            self.publish_result(result)
            if not self.show:
                continue
            cv2.imshow(self.window, result)
            if not mouse_hooked:
                # The window exists only after the first imshow.
                cv2.setMouseCallback(
                    self.window,
                    lambda event, x, y, _flags, _param: self.on_mouse(event, x, y))
                mouse_hooked = True
            key = cv2.waitKey(1)
            if key == -1:
                continue
            if not self.on_key(key) and key in QUIT_KEYS:
                self.running = False
        self.on_stop()
        if self.show:
            cv2.destroyAllWindows()

    def publish_result(self, image, encoding='bgr8'):
        """Publish an overlay on ~/image_result."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'depth_cam_frame'
        self.result_pub.publish(to_image_msg(image, header, encoding))


def main(node_class, *args, **kwargs):
    """Start a VisionDemo: rclpy on a worker, the window on the main thread."""
    rclpy.init()
    node = node_class(*args, **kwargs)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Shut down first: that is what ends rclpy.spin() on the worker, and
        # destroying the node out from under a running spin raises there.
        rclpy.try_shutdown()
        spinner.join(timeout=2.0)
        node.destroy_node()
