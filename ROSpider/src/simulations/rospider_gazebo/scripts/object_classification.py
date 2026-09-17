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
import numpy as np
from rospider_gazebo import shape_detect, vision_demo
from rospider_gazebo.vision_demo import VisionDemo, depth_color_map

#: Upstream's goto_default(), servo ids 19-24.
LOOK_POSE = ((19, 500), (20, 500), (21, 130), (22, 130), (23, 500), (24, 700))

#: Recognition ROI as [y_min, y_max, x_min, x_max].
ROI = [50, 350, 150, 500]

DEPTH_CEILING = 350
CALIBRATION_FRAMES = 50


class ObjectClassificationNode(VisionDemo):

    window = 'depth'

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
        """Upstream's window: depth map | rgb. Every object gets a white
        box; the nearest one of the wanted shapes -- the one upstream would
        go and pick -- gets its name in the middle and a red rotated box.
        The rgb half carries the ROI. What was seen is logged."""
        if self.depth_mm is None or self.intrinsics is None:
            self.get_logger().info('waiting for depth and camera_info',
                                   throttle_duration_sec=2.0)
            return frame
        depth_mm = self.depth_mm
        if depth_mm.shape[:2] != frame.shape[:2]:
            frame = cv2.resize(frame, (depth_mm.shape[1], depth_mm.shape[0]))

        near = shape_detect.nearest_distance(depth_mm, self.roi,
                                             self.plane_distance)
        view = depth_color_map(depth_mm, DEPTH_CEILING)
        if self.debug:
            self._calibrate(near)
            return np.concatenate([view, frame], axis=1)

        objects = shape_detect.recognise(depth_mm, frame, self.intrinsics,
                                         self.plane_distance, near, self.roi)
        for obj in objects:
            x, y, w, h = obj.box
            cv2.rectangle(view, (x, y), (x + w, y + h), (255, 255, 255), 2)
        wanted = [o for o in objects if o.kind in self.shapes]
        if wanted:
            target = min(wanted, key=lambda o: o.depth)
            x, y, w, h = target.box
            cv2.putText(view, target.kind, (x + w // 2, y + h // 2 - 10),
                        cv2.FONT_HERSHEY_COMPLEX, 1.0, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(view, target.kind, (x + w // 2, y + h // 2 - 10),
                        cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 255, 255), 1)
            cv2.drawContours(view, [np.int32(cv2.boxPoints(target.rect))], -1,
                             (0, 0, 255), 2, cv2.LINE_AA)
            px, py, pz = target.position
            self.get_logger().info(
                f'{shape_detect.colour_name(target.bgr)} {target.name} '
                f'({px:+.3f}, {py:+.3f}, {pz:.3f}) m {target.angle:.0f} deg',
                throttle_duration_sec=1.0)
        cv2.rectangle(frame, (self.roi[2], self.roi[0]),
                      (self.roi[3], self.roi[1]), (255, 255, 0), 1)
        return np.concatenate([view, frame], axis=1)

    def _calibrate(self, near):
        """Average the floor distance, logging each sample as upstream does."""
        if near > 0:
            self.samples.append(near)
        self.get_logger().info(f'Calibrating Ground: {near} mm')
        if len(self.samples) >= CALIBRATION_FRAMES:
            self.plane_distance = round(sum(self.samples) / len(self.samples))
            self.debug = False
            self.samples = []
            self.get_logger().info(
                f'floor plane is {self.plane_distance} mm away -- pass '
                f'plane_distance:={self.plane_distance} next time')


def main():
    vision_demo.main(ObjectClassificationNode)


if __name__ == '__main__':
    main()
