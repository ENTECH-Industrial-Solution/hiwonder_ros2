#!/usr/bin/env python3
"""Pick coloured cubes off the pedestal and move them to the drop marker.

Look-then-move: the camera rides on link4, so moving the arm moves the camera
off the target and inside the depth sensor's 0.15 m near clip. The node detects
from one fixed look pose, latches the target in base_link, and only then moves.
"""

import math
from enum import Enum

import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from interfaces.msg import ObjectsInfo
from interfaces.srv import SetString
from rclpy.node import Node
from rospider_gazebo import arm_ik
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# A few consecutive misses, not a single one: LOOK already required
# stable_frames consecutive detections before latching onto a colour, so one
# dropped ObjectsInfo message right after the LOOK -> LOCALIZE transition
# should not throw the whole colour away.
_LOCALIZE_MISS_LIMIT = 3


def _box_centroid_and_area(box):
    """Pixel centroid (u, v) and area of a detector box, or None if `box` is
    neither shape ObjectInfo.box can legally take.

    Hiwonder's own competition/yolo_node.py (around line 135) publishes an
    8-number box -- the four (x, y) corners of an oriented box -- whenever
    the underlying YOLO model runs OBB, and its own consumer,
    competition/pick_and_place.py (around lines 796-798), averages those four
    corners for the centroid. `box` is an unbounded int32[], so reading it as
    a 4-number [x1, y1, x2, y2] box would silently compute the midpoint of
    two corners instead of the true centre. The spec's swappability promise
    -- a real YOLO node can replace color_detect.py without touching this
    file -- is only true if both conventions are handled here.
    """
    if len(box) == 4:
        x1, y1, x2, y2 = box
        return (x1 + x2) / 2.0, (y1 + y2) / 2.0, abs(x2 - x1) * abs(y2 - y1)
    if len(box) == 8:
        xs = box[0::2]
        ys = box[1::2]
        u = sum(xs) / 4.0
        v = sum(ys) / 4.0
        return u, v, (max(xs) - min(xs)) * (max(ys) - min(ys))
    return None


class State(Enum):
    IDLE = 'IDLE'
    LOOK = 'LOOK'
    LOCALIZE = 'LOCALIZE'
    PRE_GRASP = 'PRE_GRASP'
    DESCEND = 'DESCEND'
    GRASP = 'GRASP'
    LIFT = 'LIFT'
    TO_DROP = 'TO_DROP'
    LOWER = 'LOWER'
    RELEASE = 'RELEASE'
    RETREAT = 'RETREAT'
    DONE = 'DONE'


class PickAndPlaceNode(Node):

    def __init__(self):
        super().__init__('pick_and_place',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.bridge = CvBridge()
        p = self.get_parameter
        self.look_pose = np.array(p('look_pose').value, dtype=float)
        self.pitches = list(p('approach_pitches_deg').value)
        self.approach_distance = float(p('approach_distance').value)
        self.grasp_z_offset = float(p('grasp_z_offset').value)
        self.gripper_open = float(p('gripper_open').value)
        self.grasp_joint_value = float(p('grasp_joint_value').value)
        # ROS 2 parameters can't hold a list of lists, so drop_slots is
        # written as nested "slot0/slot1/slot2" parameters in the yaml;
        # get_parameters_by_prefix returns them keyed by their relative
        # name, and sorting by key recovers placement order (slot0, slot1,
        # slot2, ...).
        slot_params = self.get_parameters_by_prefix('drop_slots')
        self.drop_slots = [
            np.array(slot_params[key].value, dtype=float)
            for key in sorted(slot_params.keys())]
        self.colors = list(p('colors').value)
        self.stable_frames = int(p('stable_frames').value)
        self.joint_tolerance = float(p('joint_tolerance').value)
        self.move_duration = float(p('move_duration').value)
        self.settle_time = float(p('settle_time').value)
        self.state_timeout = float(p('state_timeout').value)
        self.depth_window = int(p('depth_window_px').value)

        # drop_slots arrive in base_footprint; arm_ik works in base_link.
        for slot in self.drop_slots:
            slot[2] -= arm_ik.BASE_LINK_HEIGHT

        self.state = State.IDLE
        self.state_entered = self.get_clock().now()
        self.goal = None
        self.target_color = None
        self.plan = None
        self.remaining = list(self.colors)
        self.placed_count = 0
        self.detections = []
        self.streak = 0
        self.localize_misses = 0
        self._look_commanded = False
        self.joint_state = {}
        self.depth_image = None
        self.intrinsics = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 1)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 1)
        self.attach_pubs = {
            c: self.create_publisher(Empty, f'/grasp/{c}/attach', 1)
            for c in self.colors}
        self.detach_pubs = {
            c: self.create_publisher(Empty, f'/grasp/{c}/detach', 1)
            for c in self.colors}

        self.create_subscription(
            ObjectsInfo, '/yolo/object_detect', self.objects_callback, 1)
        self.create_subscription(
            Image, '/depth_cam/depth/image_raw', self.depth_callback, 1)
        self.create_subscription(
            CameraInfo, '/depth_cam/depth/camera_info', self.info_callback, 1)
        self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)

        self.create_service(SetString, '~/start', self.start_callback)
        self.create_service(Trigger, '~/stop', self.stop_callback)

        # Gazebo welds all three cubes to link5 the moment they spawn (proven
        # in Task 4: with zero /grasp/... messages ever published, moving the
        # arm dragged all three cubes off the pedestal together). A detach
        # published in the same breath as its own publisher's construction is
        # dropped before discovery completes, so a single release_all() here
        # in __init__ is a no-op. Instead, fire it repeatedly on a timer and
        # hold the node in IDLE -- never commanding the arm -- until the
        # sequence has actually run.
        #
        # Leaving IDLE is gated on observable state (all five arm joints
        # present in /joint_states), not on a fixed firing count. A fixed
        # count fails two ways: (a) gz-sim's DetachableJoint only subscribes
        # to its detach topic once it has found its child model and created
        # the joint, gz-transport does not latch, and a detach published
        # before a slow-spawning cube's subscriber exists is silently
        # dropped -- so firing until the node actually leaves IDLE, rather
        # than for a fixed ~3 s, makes this self-correcting for any spawn
        # delay; and (b) if the timer stopped and commanded the arm before
        # arm_controller had actually activated, the one-shot look-pose
        # publish in _on_look would go nowhere and every colour would time
        # out (this happened once during implementation, see
        # task-6-report.md's final-fix correction note).
        self._startup_done = False
        self._startup_warned = False
        self._startup_firings = 0
        self._startup_started = self.get_clock().now()
        self._startup_deadline_s = 30.0   # generous: normally ready in ~5 s
        self._pending_colors = list(self.colors) if bool(p('auto_start').value) else None
        self.startup_timer = self.create_timer(0.5, self._startup_tick)

        self.create_timer(0.1, self.tick)

    # ------------------------------------------------------------- startup

    def _startup_tick(self):
        self.release_all()
        self._startup_firings += 1
        if not all(name in self.joint_state for name in arm_ik.JOINT_NAMES):
            elapsed = (self.get_clock().now()
                       - self._startup_started).nanoseconds * 1e-9
            # Not logged every firing (that would spam at 2 Hz for however
            # long a slow spawn takes) but the firing count in the eventual
            # "ready" log, and this loud one-shot warning if a real problem
            # is dragging startup out, are the evidence that this is still
            # retrying rather than having silently given up.
            if elapsed > self._startup_deadline_s and not self._startup_warned:
                self._startup_warned = True
                self.get_logger().error(
                    f'arm joints not seen in /joint_states after '
                    f'{elapsed:.0f} s ({self._startup_firings} detach '
                    'firings so far) -- arm_controller may not be active. '
                    'Still detaching every 0.5 s and waiting; the arm will '
                    'not be commanded until the joints appear.')
            return
        self.startup_timer.cancel()
        self._startup_done = True
        self.get_logger().info(
            f'arm joints present in /joint_states after '
            f'{self._startup_firings} startup detach firings; ready to '
            'command the arm')
        if self._pending_colors is not None:
            self._begin_now(self._pending_colors)
            self._pending_colors = None

    # ---------------------------------------------------------------- inputs

    def objects_callback(self, msg):
        self.detections = [(o.class_name, o.box) for o in msg.objects]

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

    def info_callback(self, msg):
        self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def joint_callback(self, msg):
        self.joint_state.update(zip(msg.name, msg.position))

    def start_callback(self, request, response):
        wanted = [request.data] if request.data else list(self.colors)
        wanted = [c for c in wanted if c in self.colors]
        self.begin(wanted)
        response.success = True
        response.message = (
            f'picking {wanted}' if self._startup_done
            else f'picking {wanted} once startup detach completes')
        return response

    def stop_callback(self, _request, response):
        # Without this, a /stop called between DESCEND and LOWER leaves the
        # cube welded to link5 forever -- tick() returns immediately once in
        # DONE, so nothing ever detaches it or opens the gripper (Finding 3).
        self.remaining = []
        self._pending_colors = None
        self.release_all()
        self.send_gripper(self.gripper_open)
        self.target_color = None
        self.enter(State.DONE)
        response.success = True
        response.message = 'stopped'
        return response

    # --------------------------------------------------------------- helpers

    def begin(self, colors):
        # The node must never command the arm before the startup detach
        # sequence has finished (see __init__). If it hasn't, remember the
        # request and let _startup_tick start it once the sequence completes.
        if not self._startup_done:
            self._pending_colors = list(colors)
            return
        self._begin_now(colors)

    def _begin_now(self, colors):
        # A /start mid-carry (or auto_start racing a leftover pending run)
        # must not carry whatever cube is currently welded into the new run.
        self.release_all()
        self.send_gripper(self.gripper_open)
        self.target_color = None
        # A fresh run must not inherit the previous run's stack height --
        # releasing straight at n=3 leaves only ~7.2 deg of joint-limit
        # margin (Finding 3) -- but only when this request actually starts a
        # new row. A single-colour /start (e.g. retrying one colour after a
        # timeout) must NOT reset placed_count: that would re-target slot0,
        # which may already hold a cube placed earlier in this same run.
        if set(colors) == set(self.colors):
            self.placed_count = 0
        self.remaining = list(colors)
        self.enter(State.LOOK)

    def enter(self, state):
        self.get_logger().info(f'{self.state.value} -> {state.value}')
        self.state = state
        self.state_entered = self.get_clock().now()
        self.streak = 0
        self.localize_misses = 0
        # Only meaningful for LOOK (see _on_look), reset unconditionally here
        # so every entry into LOOK -- including a re-entry after a timeout or
        # an abandoned colour -- gets exactly one fresh look-pose publish.
        self._look_commanded = False

    def elapsed(self):
        return (self.get_clock().now() - self.state_entered).nanoseconds * 1e-9

    def send_arm(self, q):
        self.goal = np.asarray(q, dtype=float)
        msg = JointTrajectory()
        msg.joint_names = list(arm_ik.JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in self.goal]
        point.time_from_start.sec = int(self.move_duration)
        point.time_from_start.nanosec = int(
            (self.move_duration % 1.0) * 1e9)
        msg.points.append(point)
        self.arm_pub.publish(msg)

    def send_gripper(self, value):
        msg = JointTrajectory()
        msg.joint_names = ['r_joint']
        point = JointTrajectoryPoint()
        point.positions = [float(value)]
        point.time_from_start.sec = 1
        msg.points.append(point)
        self.gripper_pub.publish(msg)

    def arrived(self):
        if self.goal is None:
            return False
        for name, want in zip(arm_ik.JOINT_NAMES, self.goal):
            if name not in self.joint_state:
                return False
            if abs(self.joint_state[name] - want) > self.joint_tolerance:
                return False
        return True

    def release_all(self):
        for pub in self.detach_pubs.values():
            pub.publish(Empty())

    def abandon(self, reason):
        self.get_logger().warn(f'{self.target_color}: {reason}; skipping')
        self.release_all()
        self.send_gripper(self.gripper_open)
        if self.target_color in self.remaining:
            self.remaining.remove(self.target_color)
        self.target_color = None
        self.enter(State.LOOK)

    def localize(self, u, v):
        """Pixel centroid + depth -> a point in base_link, or None."""
        if self.depth_image is None or self.intrinsics is None:
            return None
        u = int(u)
        v = int(v)
        half = self.depth_window // 2
        patch = self.depth_image[max(0, v - half):v + half + 1,
                                 max(0, u - half):u + half + 1]
        patch = patch[np.isfinite(patch) & (patch > 0.0)]
        if patch.size == 0:
            return None
        depth = float(np.median(patch))

        fx, fy, cx, cy = self.intrinsics
        camera_point = np.array([(u - cx) * depth / fx,
                                 (v - cy) * depth / fy,
                                 depth])
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', 'depth_cam_frame', rclpy.time.Time())
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(f'no transform: {exc}')
            return None

        t = tf.transform.translation
        r = tf.transform.rotation
        rotation = _quaternion_matrix(r.x, r.y, r.z, r.w)
        return rotation @ camera_point + np.array([t.x, t.y, t.z])

    def plan_for(self, point):
        return arm_ik.plan_grasp(
            point, self.pitches, self.approach_distance)

    # ----------------------------------------------------------- state machine

    def tick(self):
        if self.state in (State.IDLE, State.DONE):
            return
        if (self.state not in (State.LOOK,)
                and self.elapsed() > self.state_timeout):
            self.abandon(f'timed out in {self.state.value}')
            return

        handler = getattr(self, f'_on_{self.state.name.lower()}')
        handler()

    def _on_look(self):
        if not self.remaining:
            self.send_arm(self.look_pose)
            self.get_logger().info('all cubes placed')
            self.enter(State.DONE)
            return

        # Publish once per entry into LOOK, not every tick, and not merely
        # when self.goal differs from look_pose: send_arm() records the goal
        # regardless of whether arm_controller actually acted on it, so if
        # the very first publish went out before the controller finished
        # activating, comparing against the stored goal would suppress every
        # resend forever -- this happened once during implementation (every
        # colour timed out because the arm never reached the look pose; see
        # task-6-report.md's final-fix correction note). enter() resets
        # _look_commanded on every transition into LOOK, including a
        # re-entry after a timeout or an abandoned colour, so this is a
        # publish-once-per-entry guard rather than a publish-once-ever guard.
        # It still only fires once per entry: a joint_trajectory_controller
        # restarts the trajectory on each message, so republishing at 10 Hz
        # would keep the arm perpetually 2 s from its goal and arrived()
        # would never hold.
        if not self._look_commanded:
            self.send_arm(self.look_pose)
            self._look_commanded = True

        wanted = self.remaining[0]
        # Checked before arrived(), so a stuck arm gives up instead of hanging.
        if self.elapsed() > self.state_timeout:
            self.get_logger().warn(f'never saw {wanted}; skipping')
            self.remaining.remove(wanted)
            self.enter(State.LOOK)
            return
        if not self.arrived():
            return
        if any(name == wanted for name, _ in self.detections):
            self.streak += 1
        else:
            self.streak = 0
        if self.streak >= self.stable_frames:
            self.target_color = wanted
            self.enter(State.LOCALIZE)

    def _on_localize(self):
        # /yolo/object_detect carries two boxes per colour: the graspable
        # cube on the pedestal (near, large) and a same-coloured decorative
        # cube further away (small) -- see config/color_detect.yaml. Take the
        # box with the largest area, not the first match, or half the time
        # this reaches for a decoration bolted to the world. _box_centroid_
        # and_area() also copes with an 8-number OBB box, which Hiwonder's
        # own yolo_node.py can publish (see its docstring).
        candidates = []
        for name, box in self.detections:
            if name != self.target_color:
                continue
            parsed = _box_centroid_and_area(box)
            if parsed is None:
                self.get_logger().warn(
                    f'{self.target_color}: box of length {len(box)} is '
                    'neither a 4-number [x1,y1,x2,y2] nor an 8-number OBB '
                    'corner box; skipping this detection')
                continue
            candidates.append(parsed)
        if not candidates:
            self.localize_misses += 1
            if self.localize_misses < _LOCALIZE_MISS_LIMIT:
                return
            self.abandon('lost the target')
            return
        self.localize_misses = 0
        u, v, _area = max(candidates, key=lambda c: c[2])
        point = self.localize(u, v)
        if point is None:
            self.abandon('no usable depth')
            return
        # Logged separately from the offset-applied point below so the raw
        # value can be checked against ground truth without grasp_z_offset
        # in the loop -- a check against the post-offset point can never
        # catch an error in the offset itself (see Finding 1).
        self.get_logger().info(
            f'{self.target_color} raw {np.round(point, 3)}')
        point[2] += self.grasp_z_offset
        plan = self.plan_for(point)
        if plan is None:
            # Candidate is unreachable -- most likely we still picked up the
            # decorative cube's box (e.g. its blob briefly out-sized the real
            # one). Skip this colour rather than reaching for a decoration.
            self.abandon(f'unreachable at {np.round(point, 3)}')
            return
        self.plan = plan
        self.get_logger().info(
            f'{self.target_color} at {np.round(point, 3)} '
            f'pitch {math.degrees(plan.pitch):.0f} deg '
            f'margin {math.degrees(plan.margin):.0f} deg')
        self.send_gripper(self.gripper_open)
        self.send_arm(plan.approach)
        self.enter(State.PRE_GRASP)

    def _on_pre_grasp(self):
        if self.arrived():
            self.send_arm(self.plan.grasp)
            self.enter(State.DESCEND)

    def _on_descend(self):
        if self.arrived():
            self.send_gripper(self.grasp_joint_value)
            self.attach_pubs[self.target_color].publish(Empty())
            self.enter(State.GRASP)

    def _on_grasp(self):
        if self.elapsed() > self.settle_time:
            self.send_arm(self.plan.approach)
            self.enter(State.LIFT)

    def _on_lift(self):
        if self.arrived():
            # Fix round 2: a stacked third release is not reachable with a
            # steep approach at all (the arm's 2R sub-chain is too short to
            # back off from shoulder height -- see config/pick_place.yaml),
            # so each cube now goes into its own ground-level slot instead of
            # on top of the previous one. placed_count indexes drop_slots in
            # placement order, same role stack_height's multiplier used to
            # play.
            if self.placed_count >= len(self.drop_slots):
                self.get_logger().error(
                    f'placed_count {self.placed_count} has no drop slot '
                    f'(only {len(self.drop_slots)} configured); releasing '
                    'the held cube and stopping')
                # Otherwise this is the last remaining path that could leave
                # a cube welded to link5 forever: DONE returns immediately
                # from tick(), so nothing else would ever detach or open the
                # gripper.
                self.release_all()
                self.send_gripper(self.gripper_open)
                self.enter(State.DONE)
                return
            drop = self.drop_slots[self.placed_count].copy()
            plan = self.plan_for(drop)
            if plan is None:
                self.abandon('drop slot unreachable')
                return
            self.plan = plan
            self.send_arm(plan.approach)
            self.enter(State.TO_DROP)

    def _on_to_drop(self):
        if self.arrived():
            self.send_arm(self.plan.grasp)
            self.enter(State.LOWER)

    def _on_lower(self):
        if self.arrived():
            self.detach_pubs[self.target_color].publish(Empty())
            self.send_gripper(self.gripper_open)
            # Counted here, where the cube actually leaves the hand, not in
            # _on_retreat: if RELEASE or RETREAT times out, tick() calls
            # abandon() (which never touches placed_count), and the cube is
            # physically on the stack but uncounted -- the next release then
            # lands one cube-height too low, straight into the one just
            # placed (Finding 2).
            self.placed_count += 1
            self.enter(State.RELEASE)

    def _on_release(self):
        if self.elapsed() > self.settle_time:
            self.send_arm(self.plan.approach)
            self.enter(State.RETREAT)

    def _on_retreat(self):
        if self.arrived():
            self.get_logger().info(f'{self.target_color} placed')
            if self.target_color in self.remaining:
                self.remaining.remove(self.target_color)
            self.target_color = None
            self.enter(State.LOOK)


def _quaternion_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rclpy.init()
    node = PickAndPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
