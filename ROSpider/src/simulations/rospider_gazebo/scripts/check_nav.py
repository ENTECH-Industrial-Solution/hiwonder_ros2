#!/usr/bin/env python3
"""ตรวจโจทย์ Nav2: สั่งหุ่นวิ่งตามเส้นทาง แล้วบอกผลทีละด่าน

    ros2 run rospider_gazebo check_nav.py

Before each goal it asks the planner for a path, so a blocked doorway is
reported at once instead of after Nav2's recovery retries; then it sends the
goal and waits for the result. Times are on the sim clock, so a slow PC is
not penalised. It stops at the first failed goal. Scoring: rospider_gazebo/nav_check.py.
"""

import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
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


class Checker:
    def __init__(self):
        self.node = rclpy.create_node(
            'check_nav', parameter_overrides=[Parameter('use_sim_time', value=True)])
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

    def pose(self, x, y):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.w = 1.0
        return msg

    def plan_to(self, x, y):
        """'ok', 'no_path', or 'rejected' when the planner refused the request itself."""
        goal = ComputePathToPose.Goal()
        goal.goal = self.pose(x, y)
        handle = nav_check.send_with_retry(lambda: self.plan.send_goal_async(goal), self.wait,
                                           timeout=nav_check.PLAN_TIMEOUT)
        if handle is None or not handle.accepted:
            return 'rejected'
        answer = self.wait(handle.get_result_async(), nav_check.PLAN_TIMEOUT)
        found = (answer is not None and answer.status == GoalStatus.STATUS_SUCCEEDED
                 and len(answer.result.path.poses) > 0)
        return 'ok' if found else 'no_path'

    def drive(self, name, x, y, timeout, over_limit_after):
        """Drive to (x, y); 'timeout' after `timeout` sim seconds, 'over_limit' once the
        course's time budget (`over_limit_after`, sim seconds from now) runs out first."""
        plan = self.plan_to(x, y)
        if plan != 'ok':
            return GoalResult(name, plan, 0.0)
        goal = NavigateToPose.Goal()
        goal.pose = self.pose(x, y)
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


def main():
    # rclpy's own SIGINT handler would shut the context down before our
    # KeyboardInterrupt handler could cancel the goal.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    checker = Checker()
    print('ตรวจโจทย์ Nav2 - ถ้าเพิ่งตรวจไปรอบหนึ่ง ให้ปิด-เปิด launch ใหม่ก่อน '
          'หุ่นจะได้เริ่มที่จุดเกิดและเวลาเทียบกันได้')
    try:
        if not checker.ready():
            print('ไม่พบ Nav2 - เปิด launch ของโจทย์ (nav_challenge หรือ vslam_nav_challenge) '
                  'แล้วรอให้ RViz ขึ้นแผนที่ก่อน')
            return 1
        if not checker.active():
            print('Nav2 ยังเปิดไม่ครบ - รอให้ RViz ขึ้นแผนที่ แล้วตรวจใหม่')
            return 1
        results = []
        for name, label, (x, y), timeout in nav_check.COURSE:
            print(f'{name} {label}: กำลังไป ({x:.1f}, {y:.1f}) ...', flush=True)
            spent = sum(r.seconds for r in results)
            result = checker.drive(name, x, y, timeout, nav_check.COURSE_LIMIT - spent)
            print(f'  -> {result.status} ({result.seconds:.0f} วินาที)', flush=True)
            results.append(result)
            if result.status != 'succeeded':
                break
        levels = nav_check.evaluate(results)
        print(nav_check.format_report(levels))
        return 0 if all(level.status == 'pass' for level in levels) else 1
    except KeyboardInterrupt:
        try:
            checker.cancel()
        except (ExternalShutdownException, RuntimeError):
            pass
        print('หยุดตรวจแล้ว (ยกเลิกเป้าหมายให้หุ่นหยุด)')
        return 130
    finally:
        checker.node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
