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
        self.release_z_offset = float(p('release_z_offset').value)
        self.stack_height = float(p('stack_height').value)
        self.gripper_open = float(p('gripper_open').value)
        self.grasp_joint_value = float(p('grasp_joint_value').value)
        self.drop_point = np.array(p('drop_point').value, dtype=float)
        self.colors = list(p('colors').value)
        self.stable_frames = int(p('stable_frames').value)
        self.joint_tolerance = float(p('joint_tolerance').value)
        self.move_duration = float(p('move_duration').value)
        self.settle_time = float(p('settle_time').value)
        self.state_timeout = float(p('state_timeout').value)
        self.depth_window = int(p('depth_window_px').value)

        # drop_point arrives in base_footprint; arm_ik works in base_link.
        self.drop_point[2] -= arm_ik.BASE_LINK_HEIGHT

        self.state = State.IDLE
        self.state_entered = self.get_clock().now()
        self.goal = None
        self.target_color = None
        self.plan = None
        self.remaining = list(self.colors)
        self.placed_count = 0
        self.detections = []
        self.streak = 0
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
        # sequence has actually run. Firings are counted rather than timed
        # off the clock because use_sim_time is true and /clock may not have
        # started ticking yet when this constructor runs.
        self._startup_firings = 0
        self._startup_total = 6   # ~3 s at 0.5 s/firing
        self._startup_done = False
        self._pending_colors = list(self.colors) if bool(p('auto_start').value) else None
        self.startup_timer = self.create_timer(0.5, self._startup_tick)

        self.create_timer(0.1, self.tick)

    # ------------------------------------------------------------- startup

    def _startup_tick(self):
        self.release_all()
        self._startup_firings += 1
        if self._startup_firings >= self._startup_total:
            self.startup_timer.cancel()
            self._startup_done = True
            self.get_logger().info(
                f'startup detach complete ({self._startup_firings} firings); '
                'ready to command the arm')
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
        # must not carry whatever cube is currently welded into the new run,
        # and a fresh run must not inherit the previous run's stack height --
        # releasing straight at n=3 leaves only ~7.2 deg of joint-limit
        # margin (Finding 3).
        self.release_all()
        self.send_gripper(self.gripper_open)
        self.target_color = None
        self.placed_count = 0
        self.remaining = list(colors)
        self.enter(State.LOOK)

    def enter(self, state):
        self.get_logger().info(f'{self.state.value} -> {state.value}')
        self.state = state
        self.state_entered = self.get_clock().now()
        self.streak = 0

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

    def localize(self, box):
        """Box centroid + depth -> a point in base_link, or None."""
        if self.depth_image is None or self.intrinsics is None:
            return None
        u = int((box[0] + box[2]) / 2)
        v = int((box[1] + box[3]) / 2)
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

        # Publish once, not every tick: a joint_trajectory_controller restarts
        # the trajectory on each message, so republishing at 10 Hz would keep
        # the arm perpetually 2 s from its goal and arrived() would never hold.
        if self.goal is None or not np.array_equal(self.goal, self.look_pose):
            self.send_arm(self.look_pose)

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
        # this reaches for a decoration bolted to the world.
        boxes = [b for name, b in self.detections if name == self.target_color]
        if not boxes:
            self.abandon('lost the target')
            return
        box = max(boxes, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]))
        point = self.localize(box)
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
            # All three colours share one drop_point, so from the second cube
            # on there is already a cube sitting there. Releasing at a fixed
            # height (empirically, drop_point.z + release_z_offset -- only
            # ~5 mm above one resting cube's top) drove each new cube's
            # gripper straight into the previous one: full-demo testing
            # showed this cascade knock cubes clean off the pedestal (one
            # colour ended up 16 cm from the marker, on the floor -- see
            # task-6-report.md). Releasing placed_count cube-heights higher
            # each time targets the current top of the stack instead, so
            # cubes land on top of each other rather than colliding.
            drop = self.drop_point.copy()
            drop[2] += self.placed_count * self.stack_height
            drop[2] += self.release_z_offset
            plan = self.plan_for(drop)
            if plan is None:
                self.abandon('drop point unreachable')
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
