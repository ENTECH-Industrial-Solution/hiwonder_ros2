#!/usr/bin/env python3
"""Name the colour, then nod at it -- the colour-recognition window.

A port of example/opencv_example/include/color_recognition_node.py. Once a
colour has held for STABLE_FRAMES the robot plays that colour's little arm
move, exactly the pulses upstream sends, and the window captions the colour.

Two upstream pieces have no simulated counterpart and are dropped:
the buzzer (ros_robot_controller/set_buzzer -- there is no sound in Gazebo,
so the move is the whole acknowledgement) and the interfaces/SetColorDetectParam
call that tells the colour node which colours to look for. Here the detector
is already looking for the colours in config/color_detect.yaml, and the
`colors` parameter below picks which of them this demo reacts to.

    ros2 launch rospider_gazebo gazebo.launch.py
    ros2 launch rospider_gazebo vision_demo.launch.py demo:=color_recognition
"""

import threading
import time

from interfaces.msg import ObjectsInfo
from rospider_gazebo import vision_demo
from rospider_gazebo.detections import largest
from rospider_gazebo.vision_demo import VisionDemo, banner

#: Upstream's init_process() pose, servo ids 19-24.
LOOK_POSE = ((19, 500), (20, 750), (21, 200), (22, 150), (23, 500), (24, 700))

#: The move each colour gets: (servo id, (pulse, pulse, pulse)). Straight
#: from upstream's action() -- red waves the wrist, green the base, blue the
#: roll joint.
MOVES = {
    'red': (22, (200, 100, 150)),
    'green': (19, (400, 600, 500)),
    'blue': (23, (400, 600, 500)),
}

STABLE_FRAMES = 30
STEP_TIME = 0.5


class ColorRecognitionNode(VisionDemo):

    def __init__(self):
        super().__init__('color_recognition',
                         image_topic='/color_detect/image_result',
                         servos=True)
        self.colors = list(self.param('colors', sorted(MOVES)))
        self.color = ''
        self.count = 0
        self.acting = False
        self.target = ''
        self.create_subscription(
            ObjectsInfo, str(self.param('objects_topic', '/yolo/object_detect')),
            self.objects_callback, 1)

    def on_start(self):
        self.servos.set_servo_position(1.0, LOOK_POSE)
        threading.Thread(target=self._act_loop, daemon=True).start()
        self.get_logger().info(
            f'reacting to {self.colors}; start color_detect.py or '
            'yolo_detect.py for the detections')

    def objects_callback(self, message):
        best = largest([o for o in message.objects
                        if o.class_name in self.colors])
        self.color = '' if best is None else best[0].class_name

    def _act_loop(self):
        """Play a colour's move, off the display thread so it can sleep."""
        while self.running:
            if not self.acting:
                time.sleep(0.01)
                continue
            servo_id, pulses = MOVES[self.target]
            self.get_logger().info(f'color: {self.target}')
            for pulse in pulses:
                self.servos.set_servo_position(STEP_TIME,
                                               ((servo_id, pulse),))
                time.sleep(STEP_TIME)
            self.acting = False

    def process(self, frame):
        if self.acting:
            return banner(frame, self.target.upper())
        if self.color in MOVES:
            self.count += 1
            if self.count > STABLE_FRAMES:
                self.count = 0
                self.target = self.color
                self.acting = True
        else:
            self.count = 0
        return banner(frame, self.color.upper() if self.color else 'NONE',
                      color=(255, 255, 0) if self.color else (200, 200, 200))

    def on_stop(self):
        self.acting = False
        self.servos.set_servo_position(1.0, LOOK_POSE)


def main():
    vision_demo.main(ColorRecognitionNode)


if __name__ == '__main__':
    main()
