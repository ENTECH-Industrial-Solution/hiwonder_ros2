#!/usr/bin/env python3
"""Name the shape and measure it -- the object-volume window.

A port of example/rgbd_example/include/object_volume_measurement.py. Inside a
fixed ROI the depth image is thresholded against the floor plane, each blob is
called a sphere, a cylinder or a cuboid by its corner count and depth spread,
and its volume is estimated from its pixel size, its distance and how far it
stands off the floor. The window is upstream's: the depth colour map beside
the colour frame, ROI boxed in both, one line per object.

The recognition is in rospider_gazebo/shape_detect.py, tested on synthetic
depth frames in test/test_shape_detect.py.

`plane_distance` is in millimetres here, as upstream has it for this demo --
scripts/prevent_falling.py takes metres, which is upstream's inconsistency,
not a typo. Run once with `debug:=true` to measure it.

Upstream's arm is not driven here: its main loop only ever displays, and
picking things up in the simulation is scripts/pick_and_place.py's job.

    ros2 launch rospider_gazebo pick_place.launch.py
    ros2 launch rospider_gazebo vision_demo.launch.py demo:=object_volume
"""

import cv2
import numpy as np
from rospider_gazebo import shape_detect, vision_demo
from rospider_gazebo.vision_demo import VisionDemo, depth_color_map

#: Upstream's goto_default(), servo ids 19-24.
LOOK_POSE = ((19, 500), (20, 500), (21, 130), (22, 130), (23, 500), (24, 700))

#: Recognition ROI as [y_min, y_max, x_min, x_max], upstream's values.
ROI = [50, 350, 150, 500]

#: Millimetres that saturate the depth colour map, as upstream scales it.
DEPTH_CEILING = 350

CALIBRATION_FRAMES = 50


class ObjectVolumeNode(VisionDemo):

    window = 'Object Classification (ROI Mode)'

    def __init__(self):
        super().__init__('object_volume', depth=True,
                         camera_info='/depth_cam/depth/camera_info',
                         servos=True)
        self.debug = bool(self.param('debug', False))
        self.plane_distance = float(self.param('plane_distance', 350.0))
        self.roi = [int(v) for v in self.param('roi', ROI)]
        self.samples = []

    def on_start(self):
        self.servos.set_servo_position(1.0, LOOK_POSE)
        self.get_logger().info(
            'calibrating the floor plane' if self.debug else
            f'floor plane at {self.plane_distance} mm')

    def process(self, frame):
        if self.depth_mm is None or self.intrinsics is None:
            self.get_logger().info('waiting for depth and camera_info',
                                   throttle_duration_sec=2.0)
            return frame
        depth_mm = self.depth_mm
        if depth_mm.shape[:2] != frame.shape[:2]:
            frame = cv2.resize(frame, (depth_mm.shape[1], depth_mm.shape[0]))

        near = shape_detect.stable_distance(depth_mm, self.roi,
                                            self.plane_distance)
        view = depth_color_map(depth_mm, DEPTH_CEILING)
        if self.debug:
            self._calibrate(near)
            return self._side_by_side(view, frame)

        objects = shape_detect.recognise(depth_mm, frame, self.intrinsics,
                                         self.plane_distance, near, self.roi)
        self._dim_outside_roi(view)
        for row, obj in enumerate(objects):
            x, y, _w, _h = obj.box
            cv2.putText(view, obj.name, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.drawContours(view, [np.int32(cv2.boxPoints(obj.rect))], -1,
                             (0, 0, 255), 2)
            cv2.putText(view,
                        f'{obj.name}, Vol: {obj.volume:.2f}cm3, '
                        f'H: {obj.height:.1f}mm',
                        (20, view.shape[0] - 20 - row * 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return self._side_by_side(view, frame)

    def _dim_outside_roi(self, view):
        """Darken everything the recogniser is not looking at."""
        outside = np.ones(view.shape[:2], dtype=bool)
        outside[self.roi[0]:self.roi[1], self.roi[2]:self.roi[3]] = False
        view[outside] = (view[outside] * 0.3).astype(np.uint8)

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

    def _side_by_side(self, view, frame):
        cv2.rectangle(view, (self.roi[2], self.roi[0]),
                      (self.roi[3], self.roi[1]), (255, 255, 255), 2)
        cv2.rectangle(frame, (self.roi[2], self.roi[0]),
                      (self.roi[3], self.roi[1]), (255, 255, 0), 2)
        return np.concatenate([view, frame], axis=1)


def main():
    vision_demo.main(ObjectVolumeNode)


if __name__ == '__main__':
    main()
