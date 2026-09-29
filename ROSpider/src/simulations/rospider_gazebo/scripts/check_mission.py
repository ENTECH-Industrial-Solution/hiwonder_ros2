#!/usr/bin/env python3
"""ตรวจภารกิจปิดท้าย: รันไฟล์ภารกิจทีละขั้น แล้วบอกผลทีละด่าน

    ros2 run rospider_gazebo check_mission.py [path/to/mission.yaml]

Reads config/mission_challenge.yaml (or the given path) afresh at every run, checks the stack is up
and the robot and blocks are where a fresh launch puts them, then runs the steps in order through
the nodes mission_challenge.launch.py starts (rospider_gazebo/mission_plan.py describes the step
types). It stops at the first failed step, switches everything it switched on back off (on
Ctrl+C too), reads where the blocks came to rest, and scores the run with
rospider_gazebo/mission_check.py. Times are on the sim clock with wall-clock backstops.
"""

import math
import os
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from interfaces.msg import ApriltagsInfo
from interfaces.srv import SetString
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rospider_gazebo import grasp_check, mission_check, mission_plan
from rospider_gazebo.challenge_params import ChallengeConfigError
from rospider_gazebo.nav_client import Nav2Client
from std_msgs.msg import String
from std_srvs.srv import SetBool

GO_TO_LIMIT = 240.0
TAG_LIMIT = 60.0
TAG_UNSEEN = 20.0         # no sight of the tag this long after switching on: give up
STILL_S = 3.0
PICK_LIMIT = 60.0
PLACE_LIMIT = 40.0
#: After a pick the robot backs straight off this far, at this speed: it stands 0.13 m from the
#: pick pedestal, and Nav2's first turn in place would drag a foot over it.
BACK_OFF_M = 0.2
BACK_OFF_SPEED = 0.1
SETTLE = 2.0
WALL_FACTOR = 5.0         # a sim-time limit becomes this many wall seconds at most
FREE = ('IDLE', 'DONE')
STATUS_THAI = {'no_path': 'Nav2 หาทางไปไม่ได้ (จุดหมายอยู่ในผนังหรือนอกแผนที่?)',
               'aborted': 'Nav2 ไปไม่ถึงแล้วยอมแพ้', 'timeout': 'เดินไม่ถึงภายในเวลา',
               'rejected': 'Nav2 ไม่รับเป้าหมาย', 'canceled': 'ถูกยกเลิก'}


class StepFailed(Exception):
    """A step could not finish; the message is the Thai reason."""


def default_file():
    share = get_package_share_directory('rospider_gazebo')
    return os.path.realpath(os.path.join(share, 'config', 'mission_challenge.yaml'))


def gz_pose(model):
    try:
        out = subprocess.run(['gz', 'model', '-m', model, '-p'], capture_output=True, text=True,
                             timeout=30).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None
    return grasp_check.parse_gz_pose(out)


class Mission:
    def __init__(self):
        self.node = rclpy.create_node('check_mission',
                                      parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.nav = Nav2Client(self.node)
        n = self.node
        self.tag_run = n.create_client(SetBool, '/apriltag_track/set_running')
        self.tag_params = n.create_client(SetParameters, '/apriltag_track/set_parameters')
        self.grab_walk = n.create_client(SetBool, '/track_and_grab/set_walk')
        self.grab_auto = n.create_client(SetBool, '/track_and_grab/set_auto_place')
        self.grab_run = n.create_client(SetBool, '/track_and_grab/set_running')
        self.grab_pick = n.create_client(SetString, '/track_and_grab/pick')
        self.place_srv = n.create_client(SetString, '/pick_and_place/place')
        self.ctrl_set = n.create_client(SetParameters, '/controller_server/set_parameters')
        self.smooth_get = n.create_client(GetParameters, '/velocity_smoother/get_parameters')
        self.smooth_set = n.create_client(SetParameters, '/velocity_smoother/set_parameters')
        self.map_get = n.create_client(GetParameters, '/map_server/get_parameters')
        self.cmd_vel = n.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.own_map = False
        self.odom = None
        self.state = None
        self.tags_seen = {}         # tag id -> sim time last seen
        self.facts = {'in_room_b': False, 'robot': (0.0, 0.0, 0.0), 'parked_tag': None,
                      'lifted': [], 'blocks': {}, 'failure': None, 'failed_step': None}
        n.create_subscription(Odometry, '/odom', self._on_odom, 10)
        n.create_subscription(String, '/pick_and_place/state', self._on_state,
                              QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        n.create_subscription(ApriltagsInfo, '/apriltag_detect/apriltag_info', self._on_tags, 1)
        self.navigating = False

    # ---------------------------------------------------------------- plumbing

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.facts['robot'] = self.odom
        if self.odom[0] > mission_check.ROOM_B_X:
            self.facts['in_room_b'] = True

    def _on_state(self, msg):
        self.state = msg.data

    def _on_tags(self, msg):
        now = self.nav.sim_now()
        for tag in msg.data:
            self.tags_seen[tag.id] = now

    def spin(self, seconds):
        """Spin for `seconds` of sim time (wall backstop)."""
        start, wall_end = self.nav.sim_now(), time.monotonic() + seconds * WALL_FACTOR + 5
        while self.nav.sim_now() - start < seconds and time.monotonic() < wall_end:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def call(self, client, request, timeout=5.0, tries=3):
        """The response, or None; resent when a reply is lost (every call here is idempotent)."""
        for _ in range(tries):
            if not client.wait_for_service(timeout_sec=2.0):
                continue
            answer = self.nav.wait(client.call_async(request), timeout)
            if answer is not None:
                return answer
        return None

    def set_bool(self, client, on):
        request = SetBool.Request()
        request.data = on
        return self.call(client, request)

    def set_param(self, client, name, value):
        param = ParameterMsg(name=name)
        if isinstance(value, int):
            param.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
        elif isinstance(value, float):
            param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
        else:
            param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                         double_array_value=list(value))
        answer = self.call(client, SetParameters.Request(parameters=[param]))
        return answer is not None and all(r.successful for r in answer.results)

    # ---------------------------------------------------------------- set-up

    def ready(self):
        """None when everything is up and in place, else the Thai reason it is not."""
        if not (self.nav.ready() and self.nav.active()):
            return ('ไม่พบ Nav2 - เปิด ros2 launch rospider_gazebo mission_challenge.launch.py '
                    'แล้วรอให้ RViz ขึ้นแผนที่')
        for client in (self.tag_run, self.grab_pick, self.place_srv):
            if not client.wait_for_service(timeout_sec=60.0):
                return f'ไม่พบ {client.srv_name} - รอให้หน้าต่าง track_and_grab ขึ้นก่อนแล้วตรวจใหม่'
        end = time.monotonic() + 30.0
        while (self.odom is None or self.state is None) and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.odom is None or self.state is None:
            return 'ไม่ได้รับ /odom หรือสถานะของแขน - ปิด-เปิด launch ใหม่'
        answer = self.call(self.map_get, GetParameters.Request(names=['yaml_filename']))
        map_file = answer.values[0].string_value if answer is not None and answer.values else '?'
        self.own_map = os.path.basename(map_file) != 'slam_challenge.yaml'
        print(f'แผนที่: {map_file}', flush=True)
        if math.hypot(*self.odom[:2]) > 0.1:
            return 'หุ่นไม่ได้อยู่ที่จุดเริ่ม - ปิด-เปิด launch ใหม่ก่อนตรวจ'
        for color in mission_check.BLOCK_OF_TAG.values():
            pose = gz_pose(f'pick_cube_{color}')
            if pose is None or math.dist(pose, mission_check.block_spawn(color)) > 0.02:
                return f'บล็อกสี {color} ไม่ได้อยู่บนแท่น - ปิด-เปิด launch ใหม่ก่อนตรวจ'
        return None

    def set_speed(self, speed):
        """The go_to speed, live: RPP's walking speed and the smoother's forward limit."""
        self.set_param(self.ctrl_set, 'FollowPath.desired_linear_vel', speed)
        answer = self.call(self.smooth_get, GetParameters.Request(names=['max_velocity']))
        if answer is not None and answer.values:
            limits = list(answer.values[0].double_array_value)
            limits[0] = speed
            self.set_param(self.smooth_set, 'max_velocity', limits)

    # ---------------------------------------------------------------- steps

    def go_to(self, value, speed):
        x, y, yaw = value
        self.set_speed(speed)
        self.navigating = True
        result = self.nav.drive('go_to', x, y, GO_TO_LIMIT, yaw=yaw)
        self.navigating = False
        if result.status != 'succeeded':
            reason = STATUS_THAI.get(result.status, result.status)
            if result.status == 'no_path' and self.own_map:
                reason += ' - ใช้แผนที่ของตัวเองอยู่: แผนที่มีห้อง B และทางเดินครบไหม'
            raise StepFailed(reason)

    def go_to_tag(self, tag):
        if not self.set_param(self.tag_params, 'target_tag', tag):
            raise StepFailed('ตั้งหมายเลขป้ายให้ apriltag_track ไม่ได้')
        if self.set_bool(self.tag_run, True) is None:
            raise StepFailed('apriltag_track ไม่ตอบคำสั่ง')
        start = self.nav.sim_now()
        wall_end = time.monotonic() + TAG_LIMIT * WALL_FACTOR
        anchor, still_since = self.odom, start
        try:
            while True:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                now = self.nav.sim_now()
                if math.dist(self.odom[:2], anchor[:2]) > 0.005 or \
                        abs(math.remainder(self.odom[2] - anchor[2], math.tau)) > 0.02:
                    anchor, still_since = self.odom, now
                seen = self.tags_seen.get(tag)
                if (seen is None or seen < start) and now - start > TAG_UNSEEN:
                    raise StepFailed(f'มองไม่เห็นป้ายหมายเลข {tag}')
                if seen is not None and now - seen < 1.0 and now - still_since > STILL_S:
                    self.facts['parked_tag'] = tag
                    return
                if now - start > TAG_LIMIT or time.monotonic() > wall_end:
                    raise StepFailed(f'ไปไม่ถึงหน้าป้ายหมายเลข {tag} ภายในเวลา')
        finally:
            self.set_bool(self.tag_run, False)

    def pick(self, color):
        if self.state not in FREE:
            raise StepFailed('แขนยังไม่ว่าง (ถือบล็อกอยู่หรือเปล่า?)')
        self.set_bool(self.grab_walk, False)
        self.set_bool(self.grab_auto, False)
        self.set_bool(self.grab_run, False)       # camera down to the look pose
        self.spin(2.0)
        answer = self.call(self.grab_pick, SetString.Request(data=color))
        if answer is None or not answer.success:
            raise StepFailed('track_and_grab ไม่รับคำสั่งหยิบ '
                             f'({getattr(answer, "message", "ไม่ตอบ")})')
        start, wall_end = self.nav.sim_now(), time.monotonic() + PICK_LIMIT * WALL_FACTOR
        while self.state != 'CARRY':
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.nav.sim_now() - start > PICK_LIMIT or time.monotonic() > wall_end:
                self.set_bool(self.grab_run, False)
                raise StepFailed(f'หยิบบล็อกสี {color} ไม่ได้ (ไม่อยู่ในระยะแขน หรือมองไม่เห็น)')
        self.spin(1.0)
        self.back_off()
        for c in mission_check.BLOCK_OF_TAG.values():
            pose = gz_pose(f'pick_cube_{c}')
            if pose is not None and pose[2] > mission_check.LIFTED_Z:
                self.facts['lifted'].append(c)

    def back_off(self):
        """Straight back BACK_OFF_M, clear of the pedestal, before anything turns the robot."""
        twist = Twist()
        twist.linear.x = -BACK_OFF_SPEED
        start = self.nav.sim_now()
        wall_end = time.monotonic() + BACK_OFF_M / BACK_OFF_SPEED * WALL_FACTOR
        while (self.nav.sim_now() - start < BACK_OFF_M / BACK_OFF_SPEED
               and time.monotonic() < wall_end):
            self.cmd_vel.publish(twist)
            self.spin(0.1)
        self.cmd_vel.publish(Twist())
        self.spin(1.0)

    def place(self, point):
        if self.state != 'CARRY':
            raise StepFailed('ไม่ได้ถือบล็อกอยู่ จึงไม่มีอะไรให้วาง')
        answer = self.call(self.place_srv,
                           SetString.Request(data=' '.join(f'{v:g}' for v in point)))
        if answer is None or not answer.success:
            raise StepFailed(f'แขนไม่รับคำสั่งวาง ({getattr(answer, "message", "ไม่ตอบ")})')
        start, wall_end = self.nav.sim_now(), time.monotonic() + PLACE_LIMIT * WALL_FACTOR
        seen_busy = False
        while not (seen_busy and self.state in FREE):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            seen_busy = seen_busy or self.state not in FREE + ('CARRY',)
            if self.nav.sim_now() - start > PLACE_LIMIT or time.monotonic() > wall_end:
                raise StepFailed('วางไม่เสร็จภายในเวลา (จุดวางอยู่นอกระยะแขนหรือเปล่า?)')

    # ---------------------------------------------------------------- run

    def run(self, speed, steps):
        for number, step in enumerate(steps, 1):
            print(f'ขั้น {number}/{len(steps)} {mission_plan.describe(step)} ...', flush=True)
            start = self.nav.sim_now()
            try:
                if step.kind == 'go_to':
                    self.go_to(step.value, speed)
                else:
                    getattr(self, step.kind)(step.value)
            except StepFailed as err:
                print(f'  -> ไม่สำเร็จ: {err}', flush=True)
                self.facts['failure'], self.facts['failed_step'] = str(err), number
                return
            print(f'  -> สำเร็จ ({self.nav.sim_now() - start:.0f} วินาที)', flush=True)

    def stop_all(self):
        """Switch off whatever may still be on; safe to call twice. pick_and_place is left to
        finish a motion it has started: stopping the arm mid-grasp leaves the block welded at an
        odd angle."""
        if self.navigating:
            self.nav.cancel()
            self.navigating = False
        self.set_bool(self.tag_run, False)
        self.set_bool(self.grab_run, False)
        self.cmd_vel.publish(Twist())

    def finish(self):
        self.spin(SETTLE)
        self.facts['blocks'] = {c: gz_pose(f'pick_cube_{c}')
                                for c in mission_check.BLOCK_OF_TAG.values()}
        levels = mission_check.evaluate(self.facts)
        print(mission_check.format_report(levels, self.facts))
        return 0 if all(level.status == 'pass' for level in levels) else 1


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else default_file()
    try:
        speed, steps = mission_plan.load(path)
    except ChallengeConfigError as err:
        print(f'ไฟล์ภารกิจใช้ไม่ได้: {err}')
        return 1
    # rclpy's own SIGINT handler would shut the context down before stop_all() could run.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    mission = Mission()
    print(f'ตรวจภารกิจปิดท้ายจาก {path} (ความเร็ว {speed:g} ม./วินาที, {len(steps)} ขั้น)',
          flush=True)
    try:
        problem = mission.ready()
        if problem:
            print(problem)
            return 1
        mission.run(speed, steps)
        mission.stop_all()
        return mission.finish()
    except KeyboardInterrupt:
        try:
            mission.stop_all()
        except Exception:       # noqa: BLE001 -- shutting down anyway
            pass
        print('หยุดตรวจแล้ว (สั่งทุกอย่างหยุด)')
        return 130
    finally:
        mission.node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
