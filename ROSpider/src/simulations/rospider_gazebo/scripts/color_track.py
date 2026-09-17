#!/usr/bin/env python3
"""Keep a coloured block in the middle of the picture -- the colour-tracking
demo (docs 8.2, `ros2 launch example color_track_node.launch.py`).

A port of example/color_track/color_track_node.py. The colour detector's
biggest blob of the wanted colour is fed to two PIDs: one pans the arm base
(servo 19) after the block's x, the other follows its y. Upstream has no
window of its own -- what the docs show is color_detect's `result` window
with the circle round the block -- so this node shows the detector's frame
untouched, under that title.

Upstream turns the y error into a wrist height through the arm_kinematics
IK service, an aarch64-only library this workspace cannot load. The camera
rides on link4, so, as scripts/face_track.py does, the y PID drives servo 22
(joint4) directly.

Services, as upstream: ~/start and ~/stop (std_srvs/Trigger) pause and
resume the tracking, ~/set_color (interfaces/SetString) changes the colour.
With start:=true (the default, as upstream's launch file) tracking begins
at once.

    ros2 launch rospider_gazebo pick_place.launch.py       # the blocks
    ros2 launch rospider_gazebo color_track_node.launch.py color:=red
"""

from interfaces.msg import ObjectsInfo
from interfaces.srv import SetString
from rospider_gazebo import vision_demo
from rospider_gazebo.detections import largest
from rospider_gazebo.pid import PID, set_range
from rospider_gazebo.vision_demo import VisionDemo
from std_srvs.srv import Trigger

#: Start pose, servo ids 19-24: the arm shape of the robot's init action,
#: with joint4 (servo 22) tipped down so the blocks on the pedestal are in
#: the picture. Upstream starts level because the block is held up in front
#: of the robot; here it sits on the table 0.24 m ahead.
START_POSE = ((19, 500), (20, 650), (21, 40), (22, 250), (23, 500), (24, 500))

PAN_PULSE = (200, 800)      # servo 19 travel, upstream's y_dis limits
TILT_PULSE = (130, 532)     # servo 22: from the pick look-down to level

#: Upstream ignores blobs whose enclosing circle is under 10 px; a box of
#: that size is about as wide.
MIN_BOX_PX = 20


class ColorTrackNode(VisionDemo):

    window = 'result'

    def __init__(self):
        super().__init__('color_track',
                         image_topic='/color_detect/image_result',
                         servos=True)
        self.color = str(self.param('color', 'red'))
        self.tracking = bool(self.param('start', True))
        self.pan = float(self.param('pan_pulse', 500))
        self.tilt = float(self.param('tilt_pulse', START_POSE[3][1]))
        self.pid_pan = PID(float(self.param('pan_gain', 0.055)), 0.0, 0.0)
        self.pid_tilt = PID(float(self.param('tilt_gain', 0.05)), 0.0, 0.0)
        self.center = None          # (u, v, width, height) of the block
        self.create_subscription(
            ObjectsInfo, str(self.param('objects_topic', '/yolo/object_detect')),
            self.objects_callback, 1)
        self.create_service(Trigger, '~/start', self.start_callback)
        self.create_service(Trigger, '~/stop', self.stop_callback)
        self.create_service(SetString, '~/set_color', self.set_color_callback)

    def on_start(self):
        self.servos.set_servo_position(1.5, START_POSE)
        self.get_logger().info(
            f'tracking {self.color}' if self.tracking
            else 'waiting for ~/start')

    # ------------------------------------------------------------ services

    def start_callback(self, _request, response):
        self.get_logger().info('start color track')
        self.tracking = True
        response.success = True
        response.message = 'start'
        return response

    def stop_callback(self, _request, response):
        self.get_logger().info('stop color track')
        self.tracking = False
        self.pid_pan.clear()
        self.pid_tilt.clear()
        response.success = True
        response.message = 'stop'
        return response

    def set_color_callback(self, request, response):
        self.color = request.data.strip()
        self.get_logger().info(f'start_track_{self.color}')
        response.success = True
        response.message = 'set_color'
        return response

    # ------------------------------------------------------------ tracking

    def objects_callback(self, message):
        best = largest([o for o in message.objects if o.class_name == self.color])
        if best is None:
            self.center = None
            return
        obj, (u, v, _area) = best
        if obj.box[2] - obj.box[0] < MIN_BOX_PX:
            self.center = None
            return
        self.center = (u, v, obj.width, obj.height)

    def process(self, frame):
        """Upstream's loop: two PIDs, servo 19 for x and (here) 22 for y."""
        if not self.tracking or self.center is None:
            return frame
        u, v, width, height = self.center
        # Both PIDs see setpoint - measurement, so a block left of centre
        # gives a positive pan step (servo 19 up = turn left) and a block
        # above centre a positive tilt step (servo 22 up = look up).
        self.pid_pan.SetPoint = width / 2.0
        self.pid_pan.update(u)
        self.pan = set_range(self.pan + self.pid_pan.output, *PAN_PULSE)
        self.pid_tilt.SetPoint = height / 2.0
        self.pid_tilt.update(v)
        self.tilt = set_range(self.tilt + self.pid_tilt.output, *TILT_PULSE)
        self.servos.set_servo_position(0.02, ((19, int(self.pan)),
                                              (22, int(self.tilt))))
        return frame

    def on_stop(self):
        self.servos.set_servo_position(1.5, START_POSE)


def main():
    vision_demo.main(ColorTrackNode)


if __name__ == '__main__':
    main()
