"""Nav2's planner and navigator as scripts/check_nav.py and scripts/check_mission.py use them.

Plans each goal first (a blocked doorway is `no_path` at once instead of after Nav2's recoveries),
resends a goal whose response was lost (nav_check.send_with_retry), times on the sim clock with a
wall-clock backstop, and cancels the goal in flight on request (Ctrl+C). Needs a spinning-capable
rclpy node; not unit tested (it is a thin action client), verified by the two checkers' live runs.
"""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from rospider_gazebo import nav_check
from rospider_gazebo.nav_check import GoalResult

SERVER_WAIT = 30.0
#: How long to wait for Nav2 to finish coming up. The action servers exist
#: from lifecycle configure on, but reject goals until activated; a check run
#: straight after a relaunch would otherwise read that as "no path".
ACTIVE_WAIT = 120.0
NAV2_NODES = ('planner_server', 'controller_server', 'bt_navigator')
STATUS = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_ABORTED: 'aborted',
          GoalStatus.STATUS_CANCELED: 'canceled'}


class Nav2Client:
    """The Nav2 planner and navigator for one checker node."""

    def __init__(self, node):
        self.node = node
        self.plan = ActionClient(self.node, ComputePathToPose, 'compute_path_to_pose')
        self.nav = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.handle = None          # the goal in flight, cancelled on Ctrl+C
        self.pending = None         # its send_goal future, until accepted

    def sim_now(self):
        return self.node.get_clock().now().nanoseconds / 1e9

    def wait(self, future, wall_timeout):
        end = time.monotonic() + wall_timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return future.result() if future.done() else None

    def ready(self):
        end = time.monotonic() + 10.0
        while self.sim_now() == 0.0 and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return (self.sim_now() > 0.0 and self.plan.wait_for_server(timeout_sec=SERVER_WAIT)
                and self.nav.wait_for_server(timeout_sec=SERVER_WAIT))

    def active(self):
        """Wait until every Nav2 node that serves the course is active."""
        clients = [self.node.create_client(GetState, f'/{name}/get_state') for name in NAV2_NODES]
        end = time.monotonic() + ACTIVE_WAIT
        told = False
        while time.monotonic() < end:
            states = []
            for client in clients:
                answer = (self.wait(client.call_async(GetState.Request()), 2.0)
                          if client.wait_for_service(timeout_sec=1.0) else None)
                states.append(answer is not None and answer.current_state.label == 'active')
            if all(states):
                return True
            if not told:
                print('รอ Nav2 เปิดให้ครบ ...', flush=True)
                told = True
            time.sleep(1.0)
        return False

    def pose(self, x, y, yaw=0.0):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        return msg

    def plan_to(self, x, y, yaw=0.0):
        """'ok', 'no_path', or 'rejected' when the planner refused the request itself."""
        goal = ComputePathToPose.Goal()
        goal.goal = self.pose(x, y, yaw)
        handle = nav_check.send_with_retry(lambda: self.plan.send_goal_async(goal), self.wait,
                                           timeout=nav_check.PLAN_TIMEOUT)
        if handle is None or not handle.accepted:
            return 'rejected'
        answer = self.wait(handle.get_result_async(), nav_check.PLAN_TIMEOUT)
        found = (answer is not None and answer.status == GoalStatus.STATUS_SUCCEEDED
                 and len(answer.result.path.poses) > 0)
        return 'ok' if found else 'no_path'

    def drive(self, name, x, y, timeout, over_limit_after=math.inf, yaw=0.0):
        """Drive to (x, y); 'timeout' after `timeout` sim seconds, 'over_limit' once the
        course's time budget (`over_limit_after`, sim seconds from now) runs out first."""
        plan = self.plan_to(x, y, yaw)
        if plan != 'ok':
            return GoalResult(name, plan, 0.0)
        goal = NavigateToPose.Goal()
        goal.pose = self.pose(x, y, yaw)
        start = self.sim_now()

        def send():
            self.pending = self.nav.send_goal_async(goal)
            return self.pending
        self.handle = nav_check.send_with_retry(send, self.wait)
        self.pending = None
        if self.handle is None or not self.handle.accepted:
            return GoalResult(name, 'rejected', 0.0)
        future = self.handle.get_result_async()
        # Sim-clock timeout, with a wall-clock backstop in case the sim stalls.
        wall_end = time.monotonic() + timeout * 5
        while not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            elapsed = self.sim_now() - start
            if elapsed > over_limit_after:
                self.cancel()
                return GoalResult(name, 'over_limit', elapsed)
            if elapsed > timeout or time.monotonic() > wall_end:
                self.cancel()
                return GoalResult(name, 'timeout', elapsed)
        self.handle = None
        status = STATUS.get(future.result().status, 'aborted')
        return GoalResult(name, status, self.sim_now() - start)

    def cancel(self):
        """Cancel the goal in flight, including one Nav2 is still accepting."""
        if self.handle is None and self.pending is not None:
            self.handle = self.wait(self.pending, 5.0)
        if self.handle is not None and self.handle.accepted:
            self.wait(self.handle.cancel_goal_async(), 5.0)
        self.handle = self.pending = None
