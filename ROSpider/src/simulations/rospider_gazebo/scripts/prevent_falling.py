#!/usr/bin/env python3
"""Walk without falling off the table -- the prevent-falling window.

A port of example/rgbd_example/include/prevent_falling_node.py. Three small
patches of the depth image look at the floor ahead of the robot; while all
three read the expected distance it walks forward, and as soon as one of them
reads something else -- a table edge, a step -- it turns away. The window is
the depth frame as a JET colour map with the three probes marked, which is
what upstream shows.

The policy itself is in rospider_gazebo/depth_probe.py (FallPolicy), tested
without a simulator in test/test_depth_probe.py.

Calibrate first: `debug:=true` averages the three probes over 50 frames, logs
the floor distance it measured and carries on using it. Upstream also writes
that number into config/plane_distance.yaml; here it is logged for you to pass
back as `plane_distance:=<metres>`, because the sim's floor distance depends on
the arm pose the world was started with.

    ros2 launch rospider_gazebo gazebo.launch.py
    ros2 launch rospider_gazebo vision_demo.launch.py demo:=prevent_falling debug:=true
"""

import cv2
from rospider_gazebo import depth_probe, vision_demo
from rospider_gazebo.depth_probe import FALL_ROIS, FallPolicy
from rospider_gazebo.vision_demo import VisionDemo, banner, depth_color_map

#: Upstream's start pose, servo ids 19-24: camera down at the floor ahead.
LOOK_POSE = ((19, 500), (20, 700), (21, 85), (22, 150), (23, 500), (24, 700))

#: Frames the debug pass averages before it settles on a floor distance.
CALIBRATION_FRAMES = 50


class PreventFallingNode(VisionDemo):

    def __init__(self):
        super().__init__('prevent_falling', depth_frames=True,
                         cmd_vel=True, servos=True)
        self.debug = bool(self.param('debug', False))
        self.policy = FallPolicy(
            float(self.param('plane_distance', 0.35)),
            edge_tolerance=float(self.param('edge_tolerance', 0.04)),
            turn_speed=float(self.param('turn_speed', 0.2)),
            forward_speed=float(self.param('forward_speed', 0.05)))
        self.samples = []

    def on_start(self):
        self.servos.set_servo_position(1.0, LOOK_POSE)
        if self.debug:
            self.get_logger().info(
                f'calibrating: hold the robot over open floor for '
                f'{CALIBRATION_FRAMES} frames')
        else:
            self.get_logger().info(
                f'floor at {self.policy.plane_distance} m; walking')

    def process(self, depth_mm):
        distances = [depth_probe.roi_distance(depth_mm, roi)
                     for roi in FALL_ROIS.values()]
        view = depth_color_map(depth_mm)
        for roi in FALL_ROIS.values():
            cv2.circle(view, depth_probe.roi_center(roi), 10, (0, 0, 0), -1)

        if self.debug:
            return self._calibrate(view, distances)

        command = self.policy.update(
            distances, self.get_clock().now().nanoseconds * 1e-9)
        if command is not None:
            self.drive(*command)
        banner(view, ' '.join(f'{d:.3f}' for d in distances), scale=0.7,
               color=(255, 255, 255))
        banner(view, 'TURNING' if self.policy.turning else 'FORWARD',
               position=(10, 140), scale=0.8,
               color=(0, 200, 255) if self.policy.turning else (0, 255, 0))
        return view

    def _calibrate(self, view, distances):
        """Average the probes, then switch to walking with what was measured."""
        if all(d > 0 for d in distances):
            self.samples.append(sum(distances) / len(distances))
        banner(view, f'CALIBRATING {len(self.samples)}/{CALIBRATION_FRAMES}',
               scale=0.7, color=(0, 200, 255))
        if len(self.samples) >= CALIBRATION_FRAMES:
            self.policy.plane_distance = round(
                sum(self.samples) / len(self.samples), 3)
            self.debug = False
            self.samples = []
            self.get_logger().info(
                f'floor is {self.policy.plane_distance} m away -- pass '
                f'plane_distance:={self.policy.plane_distance} next time to '
                'skip this')
        return view

    def on_stop(self):
        self.stop()


def main():
    vision_demo.main(PreventFallingNode)


if __name__ == '__main__':
    main()
