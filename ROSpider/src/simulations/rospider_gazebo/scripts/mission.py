#!/usr/bin/env python3
"""Run a mini-game mission file, one block at a time.

The blocks (rospider_gazebo/mission_plan.py parses and validates them) map
onto things that already exist; this node only sequences them and waits:

  goto      Nav2 NavigateToPose
  survey    goto, then a few seconds of reading /apriltag_detect/apriltag_info
            together with the camera image, in which the mission segments
            the stations' colour panels itself (mission_plan.marker_boxes):
            the panel above each tag says which cube belongs there
  pick      drive `dock` metres straight ahead (the pedestal is too close
            to the pick pose for Nav2 to plan from), /pick_and_place/pick,
            wait for CARRY, back out the same distance
  deliver   a Nav2 goal in front of the remembered tag (TF
            tag_<id>_remembered, kept by apriltag_detect), then the tag
            behaviour `place` switched on through apriltag_detect's
            parameters for the last standoff metres and the ~/place call
  place     /pick_and_place/place on the floor here
  say       a log line

Every block logs "[k/n] block ... ok/failed (t s)" and the node publishes
/mission/elapsed while it runs and /mission/summary (latched) at the end, for
tools/score.py and the judges.
"""

import math
import os
import threading
import time

import numpy as np
import rclpy
import tf2_ros
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from interfaces.msg import ApriltagsInfo
from interfaces.srv import SetString
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rospider_gazebo import labelling, mission_plan
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class StepFailed(Exception):
    pass


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission')
        self.declare_parameter('mission_file', '')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('survey_seconds', 4.0)
        self.declare_parameter('nav_timeout', 600.0)
        self.declare_parameter('pick_timeout', 90.0)
        self.declare_parameter('place_timeout', 120.0)
        self.declare_parameter('memory_frame', 'odom')

        path = os.path.expanduser(self.get_parameter('mission_file').value)
        if not path:
            raise ValueError('mission_file is required')
        with open(path) as handle:
            self.mission = mission_plan.parse(yaml.safe_load(handle))
        self.get_logger().info(
            f'{path}: {len(self.mission.steps)} steps, waypoints '
            f'{sorted(self.mission.waypoints)}')

        self.survey_seconds = float(self.get_parameter('survey_seconds').value)
        self.nav_timeout = float(self.get_parameter('nav_timeout').value)
        self.pick_timeout = float(self.get_parameter('pick_timeout').value)
        self.place_timeout = float(self.get_parameter('place_timeout').value)

        # Shared with the run thread.
        self._lock = threading.Lock()
        self.pp_state = None          # pick_and_place's latest state
        self.pp_state_seq = 0         # bumps on every state message
        self.pp_result = None
        self.pp_result_seq = 0
        self.place_result = None
        self.place_result_seq = 0
        self.tag_pixels = {}          # id -> (x, y), latest frame
        self.boxes = []               # (colour, x1, y1, x2, y2), latest
        self.stations = {}            # tag id -> marker colour, from survey
        self.held = None              # colour of the cube being carried
        self.started_at = None
        self.times = []               # (block, ok, seconds)

        self.create_subscription(String, '/pick_and_place/state',
                                 self._on_pp_state, LATCHED)
        self.create_subscription(String, '/pick_and_place/result',
                                 self._on_pp_result, LATCHED)
        self.create_subscription(String, '/apriltag_detect/place_result',
                                 self._on_place_result, LATCHED)
        self.create_subscription(ApriltagsInfo, '/apriltag_detect/apriltag_info',
                                 self._on_tags, 5)
        # The camera itself, for the survey's marker colours; the team's
        # detector (colour or YOLO) is for the cubes and is not consulted
        # here, so a YOLO model that never saw a colour panel cannot break
        # the survey. Subscribed only while a survey runs -- the bridge
        # renders lazily and every subscriber costs the simulator.
        self.bridge = CvBridge()
        self.image_sub = None
        self.image_topic = '/depth_cam/rgb/image_raw'
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.odom = None
        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pick_client = self.create_client(SetString, '/pick_and_place/pick')
        self.place_client = self.create_client(SetString, '/pick_and_place/place')
        self.param_client = self.create_client(
            SetParameters, '/apriltag_detect/set_parameters')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.elapsed_pub = self.create_publisher(Float32, '~/elapsed', 1)
        self.summary_pub = self.create_publisher(String, '~/summary', LATCHED)
        self.create_timer(1.0, self._tick)
        self.create_service(Trigger, '~/start', self._start_callback)

        self._go = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        if bool(self.get_parameter('auto_start').value):
            self.get_logger().info('auto_start: waiting for Nav2 and '
                                   'pick_and_place, then going')
            self._go.set()
        else:
            self.get_logger().info('waiting for ~/start')

    # -------------------------------------------------------------- inputs

    def _on_pp_state(self, msg):
        with self._lock:
            self.pp_state = msg.data
            self.pp_state_seq += 1

    def _on_pp_result(self, msg):
        with self._lock:
            self.pp_result = msg.data
            self.pp_result_seq += 1

    def _on_place_result(self, msg):
        with self._lock:
            self.place_result = msg.data
            self.place_result_seq += 1

    def _on_tags(self, msg):
        with self._lock:
            self.tag_pixels = {int(t.id): (float(t.x), float(t.y))
                               for t in msg.data}

    def _on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:       # cv_bridge raises its own hierarchy
            self.get_logger().warn(f'survey image: {exc}',
                                   throttle_duration_sec=5.0)
            return
        boxes = mission_plan.marker_boxes(frame)
        with self._lock:
            self.boxes = boxes

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self.odom = (p.x, p.y, yaw)

    def _start_callback(self, _request, response):
        if self._go.is_set():
            response.success = False
            response.message = 'already running'
        else:
            self._go.set()
            response.success = True
            response.message = 'starting'
        return response

    def _tick(self):
        if self.started_at is not None and self.thread.is_alive():
            self.elapsed_pub.publish(
                Float32(data=float(time.monotonic() - self.started_at)))

    # ----------------------------------------------------------- the run

    def _run(self):
        self._go.wait()
        try:
            self._wait_for_stack()
        except StepFailed as exc:
            self.get_logger().error(str(exc))
            return
        self.started_at = time.monotonic()
        steps = self.mission.steps
        for index, step in enumerate(steps, 1):
            label = f'[{index}/{len(steps)}] {step}'
            attempts = 2 if self.mission.on_fail == 'retry' else 1
            ok = False
            started = time.monotonic()
            for attempt in range(attempts):
                try:
                    self.get_logger().info(f'{label} ...')
                    self._do(step)
                    ok = True
                    break
                except StepFailed as exc:
                    self.get_logger().warn(f'{label} failed: {exc}')
            seconds = time.monotonic() - started
            self.times.append((str(step), ok, seconds))
            self.get_logger().info(
                f'{label} {"ok" if ok else "FAILED"} ({seconds:.0f} s)')
            if not ok and self.mission.on_fail == 'stop':
                break
        total = time.monotonic() - self.started_at
        lines = [f'mission finished in {total:.0f} s']
        for name, ok, seconds in self.times:
            lines.append(f'  {"ok    " if ok else "FAILED"} {seconds:6.0f} s  {name}')
        summary = '\n'.join(lines)
        self.get_logger().info(summary)
        self.summary_pub.publish(String(data=summary))

    def _wait_for_stack(self):
        deadline = time.monotonic() + 120.0
        while time.monotonic() < deadline:
            if (self.nav.server_is_ready()
                    and self.pick_client.service_is_ready()
                    and self.param_client.service_is_ready()):
                return
            time.sleep(0.5)
        raise StepFailed('Nav2, pick_and_place or apriltag_detect never '
                         'came up')

    def _do(self, step):
        handler = getattr(self, f'_block_{step.block}')
        handler(step.arg)

    # ------------------------------------------------------------- blocks

    def _block_say(self, text):
        self.get_logger().info(f'say: {text}')

    def _block_goto(self, name):
        x, y, yaw = self.mission.waypoints[name]
        self._navigate(x, y, yaw, f'waypoint {name}')

    def _block_survey(self, name):
        """Look at the stations from three headings: a tag at the edge of
        the frame, or one whose marker sits behind a poster of the same
        hue, is caught by the neighbouring view."""
        self._block_goto(name)
        votes, seen_tags, seen_colours = {}, set(), set()
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self._on_image, 1)
        for pan in (0.0, 0.35, -0.7, 0.35):
            self._turn(pan)
            deadline = time.monotonic() + self.survey_seconds / 3.0
            while time.monotonic() < deadline:
                with self._lock:
                    tags, boxes = dict(self.tag_pixels), list(self.boxes)
                seen_tags.update(tags)
                seen_colours.update(b[0] for b in boxes)
                for tag_id, colour in mission_plan.pair_tags_with_markers(
                        tags, boxes).items():
                    votes.setdefault(tag_id, []).append(colour)
                time.sleep(0.1)
        self.destroy_subscription(self.image_sub)
        self.image_sub = None
        found = mission_plan.majority(votes)
        self.get_logger().info(
            f'survey saw tags {sorted(seen_tags)} and colours '
            f'{sorted(seen_colours)}')
        if not found:
            raise StepFailed('saw no tag with a colour marker above it')
        self.stations.update(found)
        self.get_logger().info('stations: ' + ', '.join(
            f'tag {t} = {c}' for t, c in sorted(self.stations.items())))

    def _turn(self, delta, speed=0.3):
        """Rotate in place by `delta` radians, by odometry."""
        if abs(delta) < 1e-3:
            return
        with self._lock:
            start = self.odom
        if start is None:
            raise StepFailed('no /odom yet; cannot turn')
        turned, last = 0.0, start[2]
        twist = Twist()
        twist.angular.z = math.copysign(speed, delta)
        deadline = time.monotonic() + abs(delta) / speed * 3 + 5
        while time.monotonic() < deadline:
            with self._lock:
                yaw = self.odom[2]
            step = (yaw - last + math.pi) % (2 * math.pi) - math.pi
            turned += step
            last = yaw
            if abs(turned) >= abs(delta):
                break
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)

    def _block_pick(self, colour):
        """Dock, pick, undock. The dock is driven to an exact map pose
        (`pick_table` moved `dock` metres along its heading), not just
        `dock` metres ahead: Nav2 delivers the robot within 15 cm and a few
        degrees of pick_table, and the arm's reach window is about 5 cm."""
        if self.held is not None:
            # A delivery failed and the cube is still in the gripper. Put it
            # down here (no points for it) rather than fail every block
            # that follows.
            self.get_logger().warn(
                f'still holding {self.held}; dropping it here first')
            try:
                self._place_and_wait('')
            except StepFailed as exc:
                self.get_logger().warn(f'could not drop {self.held}: {exc}')
            self.held = None
        table = self.mission.waypoints.get('pick_table')
        if table is None:
            raise StepFailed("pick needs a 'pick_table' waypoint")
        tx, ty, tyaw = table
        docked = (tx + self.mission.dock * math.cos(tyaw),
                  ty + self.mission.dock * math.sin(tyaw), tyaw)
        self._goto_precise(*docked)
        try:
            with self._lock:
                state_seq = self.pp_state_seq
                result_seq = self.pp_result_seq
            response = self._call(self.pick_client, colour, 'pick')
            if not response.success:
                raise StepFailed(response.message)
            deadline = time.monotonic() + self.pick_timeout
            while time.monotonic() < deadline:
                with self._lock:
                    if self.pp_result_seq != result_seq and \
                            self.pp_result.startswith('abandoned'):
                        raise StepFailed(self.pp_result)
                    if self.pp_state_seq != state_seq and \
                            self.pp_state == 'CARRY':
                        self.held = colour
                        return
                time.sleep(0.2)
            raise StepFailed(f'pick_and_place did not reach CARRY in '
                             f'{self.pick_timeout:.0f} s')
        finally:
            # Back out whatever happened, so the next Nav2 goal starts from
            # free space rather than from inside the pedestal's inflation.
            # By odometry, the exact reverse of the dock: AMCL is at its
            # least certain right after a strafing manoeuvre, and a map-frame
            # creep-back once ended 0.4 m off.
            self._drive(-self.mission.dock)

    def _base_pose(self):
        """(x, y, yaw) of base_footprint in map, or None."""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint',
                                                 rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        q = tf.transform.rotation
        t = tf.transform.translation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def _goto_precise(self, gx, gy, gyaw, tol=0.02, yaw_tol=0.03,
                      speed=0.05, timeout=90.0):
        """Creep to an exact map pose with the hexapod's strafe.

        A P-controller on all three axes, no planner: this is the last
        quarter metre onto a docking spot, with nothing in the way.
        """
        deadline = time.monotonic() + timeout
        settled = 0
        while time.monotonic() < deadline:
            pose = self._base_pose()
            if pose is None:
                time.sleep(0.1)
                continue
            x, y, yaw = pose
            ex, ey = gx - x, gy - y
            eyaw = (gyaw - yaw + math.pi) % (2 * math.pi) - math.pi
            # Errors into the robot frame.
            bx = math.cos(yaw) * ex + math.sin(yaw) * ey
            by = -math.sin(yaw) * ex + math.cos(yaw) * ey
            if abs(bx) < tol and abs(by) < tol and abs(eyaw) < yaw_tol:
                settled += 1
                if settled >= 3:
                    self.cmd_pub.publish(Twist())
                    time.sleep(0.5)
                    return
            else:
                settled = 0
            twist = Twist()
            twist.linear.x = max(-speed, min(speed, 1.0 * bx))
            twist.linear.y = max(-speed, min(speed, 1.0 * by))
            twist.angular.z = max(-0.3, min(0.3, 1.5 * eyaw))
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())
        raise StepFailed(f'could not settle at ({gx:.2f}, {gy:.2f}) in '
                         f'{timeout:.0f} s')

    def _drive(self, distance, speed=0.05):
        """Straight ahead (or back) by odometry, no planner involved."""
        if abs(distance) < 1e-3:
            return
        with self._lock:
            start = self.odom
        if start is None:
            raise StepFailed('no /odom yet; cannot dock')
        twist = Twist()
        twist.linear.x = math.copysign(speed, distance)
        deadline = time.monotonic() + abs(distance) / speed * 3 + 5
        while time.monotonic() < deadline:
            with self._lock:
                x, y, _ = self.odom
            if math.hypot(x - start[0], y - start[1]) >= abs(distance):
                break
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
        else:
            self.cmd_pub.publish(Twist())
            raise StepFailed(f'did not cover {distance:+.2f} m docking')
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)

    def _block_place(self, _arg):
        self._place_and_wait('')

    def _block_deliver(self, target):
        if target == 'by_marker':
            if self.held is None:
                raise StepFailed('not holding anything; pick first')
            matches = [t for t, c in self.stations.items() if c == self.held]
            if not matches:
                raise StepFailed(
                    f'no station has a {self.held} marker (survey found '
                    f'{self.stations or "nothing"})')
            tag_id = matches[0]
        else:
            tag_id = int(target)

        position, rotation = self._remembered(tag_id)
        x, y, yaw = mission_plan.goal_in_front_of_tag(
            position, rotation, self.mission.standoff + mission_plan.NAV_MARGIN)
        self._navigate(x, y, yaw, f'station of tag {tag_id}')

        # Hand the last standoff metres to apriltag_detect: place on tag N.
        with self._lock:
            result_seq = self.place_result_seq
            pp_result_seq = self.pp_result_seq
        self._set_tag_params(tag_id, 'place', True)
        try:
            deadline = time.monotonic() + self.place_timeout
            placing = False
            while time.monotonic() < deadline:
                with self._lock:
                    if self.place_result_seq != result_seq and \
                            '~/place ->' in (self.place_result or ''):
                        result_seq = self.place_result_seq
                        if 'placing' in self.place_result:
                            placing = True
                        else:
                            raise StepFailed(self.place_result)
                    if placing and self.pp_result_seq != pp_result_seq:
                        if self.pp_result.startswith('placed'):
                            self.held = None
                            return
                        raise StepFailed(self.pp_result)
                time.sleep(0.2)
            raise StepFailed('place did not complete in '
                             f'{self.place_timeout:.0f} s')
        finally:
            self._set_tag_params(tag_id, 'none', False)

    # ------------------------------------------------------------ helpers

    def _navigate(self, x, y, yaw, what):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.get_logger().info(
            f'  nav to {what}: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)')
        # bt_navigator's action server exists before its lifecycle node is
        # active and rejects goals until then; the first goal of a run can
        # land in that window, so a rejection is retried for a while.
        deadline = time.monotonic() + 90.0
        while True:
            handle = self._wait(self.nav.send_goal_async(goal), 10.0,
                                'Nav2 goal')
            if handle.accepted:
                break
            if time.monotonic() > deadline:
                raise StepFailed('Nav2 rejected the goal for 90 s')
            time.sleep(2.0)
        result = self._wait(handle.get_result_async(), self.nav_timeout,
                            'Nav2 result')
        # 4 == SUCCEEDED in action_msgs/GoalStatus.
        if result.status != 4:
            raise StepFailed(f'Nav2 ended with status {result.status}')

    def _remembered(self, tag_id):
        """(position, rotation 3x3) of tag_<id>_remembered in map."""
        frame = f'tag_{tag_id}_remembered'
        deadline = time.monotonic() + 5.0
        while True:
            try:
                tf = self.tf_buffer.lookup_transform('map', frame,
                                                     rclpy.time.Time())
                break
            except tf2_ros.TransformException as exc:
                if time.monotonic() > deadline:
                    raise StepFailed(
                        f'tag {tag_id} was never seen (no TF {frame}): {exc}')
                time.sleep(0.5)
        q = tf.transform.rotation
        t = tf.transform.translation
        return (np.array([t.x, t.y, t.z]),
                labelling.rotation_matrix((q.x, q.y, q.z, q.w)))

    def _set_tag_params(self, tag_id, action, enabled):
        request = SetParameters.Request()
        request.parameters = [
            Parameter(f'behaviors.tag{tag_id}.action', Parameter.Type.STRING,
                      action).to_parameter_msg(),
            Parameter(f'behaviors.tag{tag_id}.standoff', Parameter.Type.DOUBLE,
                      float(self.mission.standoff)).to_parameter_msg(),
            Parameter('behaviors_enabled', Parameter.Type.BOOL,
                      bool(enabled)).to_parameter_msg(),
        ]
        response = self._wait(self.param_client.call_async(request), 5.0,
                              'apriltag_detect parameters')
        for result in response.results:
            if not result.successful:
                raise StepFailed(f'apriltag_detect refused: {result.reason}')

    def _place_and_wait(self, target):
        with self._lock:
            result_seq = self.pp_result_seq
        response = self._call(self.place_client, target, 'place')
        if not response.success:
            raise StepFailed(response.message)
        deadline = time.monotonic() + self.place_timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self.pp_result_seq != result_seq:
                    if self.pp_result.startswith('placed'):
                        self.held = None
                        return
                    raise StepFailed(self.pp_result)
            time.sleep(0.2)
        raise StepFailed('place did not complete in '
                         f'{self.place_timeout:.0f} s')

    def _call(self, client, data, what):
        if not client.service_is_ready():
            raise StepFailed(f'{what} service is not available')
        request = SetString.Request()
        request.data = data
        return self._wait(client.call_async(request), 10.0, what)

    @staticmethod
    def _wait(future, timeout, what):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise StepFailed(f'{what} timed out after {timeout:.0f} s')
            time.sleep(0.05)
        if future.exception() is not None:
            raise StepFailed(f'{what} failed: {future.exception()}')
        return future.result()


def main():
    rclpy.init()
    node = MissionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
