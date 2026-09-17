#!/usr/bin/env python3
"""Sort what is on the table by shape and colour -- the classification window.

A port of example/rgbd_example/include/object_classification.py. Same depth
recognition as scripts/object_volume.py, but reported the way the sorting demo
reports it: shape, colour and the object's position in the camera frame, with
a white box round each one on the depth colour map.

What is deliberately missing is the arm. Upstream's node picks each recognised
object up and drops it in a per-shape or per-colour bin, through the
arm_kinematics services -- which are the aarch64-only *.so this workspace
cannot load, and which scripts/pick_and_place.py already replaces for the
simulation with its own closed-form IK. So this demo recognises and reports;
`ros2 launch rospider_gazebo pick_place.launch.py` is the one that picks.

    ros2 launch rospider_gazebo pick_place.launch.py
    ros2 launch rospider_gazebo vision_demo.launch.py demo:=object_classification
"""

import cv2
from rospider_gazebo import shape_detect, vision_demo
from rospider_gazebo.vision_demo import VisionDemo, banner, depth_color_map

#: Upstream's goto_default(), servo ids 19-24.
LOOK_POSE = ((19, 500), (20, 500), (21, 130), (22, 130), (23, 500), (24, 700))

#: Recognition ROI as [y_min, y_max, x_min, x_max].
ROI = [50, 350, 150, 500]

DEPTH_CEILING = 350
CALIBRATION_FRAMES = 50

DRAW_BGR = {'red': (0, 0, 255), 'green': (0, 255, 0), 'blue': (255, 0, 0)}


class ObjectClassificationNode(VisionDemo):

    def __init__(self):
        super().__init__('object_classification', depth=True,
                         camera_info='/depth_cam/depth/camera_info',
                         servos=True)
        self.debug = bool(self.param('debug', False))
        self.plane_distance = float(self.param('plane_distance', 350.0))
        self.roi = [int(v) for v in self.param('roi', ROI)]
        self.shapes = list(self.param('shapes',
                                      ['sphere', 'cuboid', 'cylinder']))
        self.samples = []

    def on_start(self):
        self.servos.set_servo_position(1.0, LOOK_POSE)
        self.get_logger().info(
            'calibrating the floor plane' if self.debug else
            f'floor plane at {self.plane_distance} mm; '
            f'reporting {self.shapes}')

    def process(self, frame):
        if self.depth_mm is None or self.intrinsics is None:
            return banner(frame, 'WAITING FOR DEPTH', scale=0.7,
                          color=(200, 200, 200))
        depth_mm = self.depth_mm
        if depth_mm.shape[:2] != frame.shape[:2]:
            frame = cv2.resize(frame, (depth_mm.shape[1], depth_mm.shape[0]))

        near = shape_detect.nearest_distance(depth_mm, self.roi,
                                             self.plane_distance)
        view = depth_color_map(depth_mm, DEPTH_CEILING)
        if self.debug:
            return self._calibrate(view, near)

        objects = [o for o in shape_detect.recognise(
            depth_mm, frame, self.intrinsics, self.plane_distance, near,
            self.roi) if o.kind in self.shapes]

        for row, obj in enumerate(objects):
            colour = shape_detect.colour_name(obj.bgr)
            x, y, w, h = obj.box
            cv2.rectangle(view, (x, y), (x + w, y + h), (255, 255, 255), 2)
            cv2.putText(view, f'{colour} {obj.name}', (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        DRAW_BGR.get(colour, (255, 255, 255)), 2)
            px, py, pz = obj.position
            cv2.putText(view,
                        f'{colour} {obj.name}  '
                        f'({px:+.3f}, {py:+.3f}, {pz:.3f}) m  '
                        f'{obj.angle:.0f} deg',
                        (20, view.shape[0] - 20 - row * 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.rectangle(view, (self.roi[2], self.roi[0]),
                      (self.roi[3], self.roi[1]), (255, 255, 255), 2)
        if not objects:
            banner(view, 'NOTHING ON THE TABLE', scale=0.6,
                   color=(200, 200, 200))
        return view

    def _calibrate(self, view, near):
        if near > 0:
            self.samples.append(near)
        banner(view, f'CALIBRATING {len(self.samples)}/{CALIBRATION_FRAMES}',
               scale=0.6, color=(0, 200, 255))
        if len(self.samples) >= CALIBRATION_FRAMES:
            self.plane_distance = round(sum(self.samples) / len(self.samples))
            self.debug = False
            self.samples = []
            self.get_logger().info(
                f'floor plane is {self.plane_distance} mm away -- pass '
                f'plane_distance:={self.plane_distance} next time')
        return view


def main():
    vision_demo.main(ObjectClassificationNode)


if __name__ == '__main__':
    main()
