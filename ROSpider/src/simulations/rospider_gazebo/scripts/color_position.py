#!/usr/bin/env python3
"""Where is the coloured block? -- the simulation's colour-position window.

A port of example/opencv_example/include/color_position.py. Upstream reads
/color_detect/color_info (interfaces/ColorsInfo) from its own colour node;
here the detections come from whatever this workspace's detector publishes on
/yolo/object_detect (interfaces/ObjectsInfo) -- scripts/color_detect.py or
scripts/yolo_detect.py -- so the same window works for either, and the demo
follows whichever detector is already running.

Run it with the simulator up:

    ros2 launch rospider_gazebo gazebo.launch.py
    ros2 launch rospider_gazebo vision_demo.launch.py demo:=color_position
"""

import cv2
from interfaces.msg import ObjectsInfo
from rospider_gazebo import vision_demo
from rospider_gazebo.detections import largest
from rospider_gazebo.vision_demo import VisionDemo, banner

#: Camera-down look pose, as upstream's color_position sets it: servo ids
#: 19-24 at the pulses its init_process() uses.
LOOK_POSE = ((19, 500), (20, 670), (21, 40), (22, 210), (23, 500), (24, 700))

#: Frames a colour has to hold before the window calls it the target, so a
#: single stray detection does not relabel the scene.
STABLE_FRAMES = 30

DRAW = (0, 255, 255)


class ColorPositionNode(VisionDemo):

    def __init__(self):
        super().__init__('color_position',
                         image_topic='/color_detect/image_result',
                         servos=True)
        self.wanted = self.param('color', '')
        self.target = None
        self.stable = 0
        self.last_name = ''
        self.create_subscription(
            ObjectsInfo, str(self.param('objects_topic', '/yolo/object_detect')),
            self.objects_callback, 1)

    def on_start(self):
        self.servos.set_servo_position(1.0, LOOK_POSE)
        self.get_logger().info(
            'the overlay comes from the detector: start color_detect.py or '
            'yolo_detect.py as well, or this window stays empty')

    def objects_callback(self, message):
        """Latch the biggest detection, filtered by the `color` parameter."""
        objects = [o for o in message.objects
                   if not self.wanted or o.class_name == self.wanted]
        best = largest(objects)
        if best is None:
            self.target = None
            self.stable = 0
            self.last_name = ''
            return
        obj, (u, v, _area) = best
        self.target = (obj.class_name, u, v)
        self.stable = self.stable + 1 if obj.class_name == self.last_name else 0
        self.last_name = obj.class_name

    def process(self, frame):
        if self.target is None:
            return banner(frame, 'NO TARGET', color=(200, 200, 200))
        name, u, v = self.target
        cv2.circle(frame, (int(u), int(v)), 5, DRAW, -1)
        cv2.putText(frame, '({:0.1f}, {:0.1f})'.format(u, v),
                    (int(u), int(v) + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    DRAW, 2)
        if self.stable > STABLE_FRAMES:
            banner(frame, name.upper())
        return frame

    def on_stop(self):
        self.get_logger().info('color position stopped')


def main():
    vision_demo.main(ColorPositionNode)


if __name__ == '__main__':
    main()
