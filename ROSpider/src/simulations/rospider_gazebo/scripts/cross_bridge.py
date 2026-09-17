#!/usr/bin/env python3
"""Creep across a narrow bridge -- the cross-bridge window.

A port of example/rgbd_example/include/cross_bridge_node.py. Five patches of
the depth image watch the floor: the outer pair tell the robot it is over the
bridge with air to either side, the inner three steer it back towards the
middle. The window is the depth frame as a JET colour map with the five probes
marked.

The policy is in rospider_gazebo/depth_probe.py (BridgePolicy), tested in
test/test_depth_probe.py. Two upstream pieces are left out there and reported
in the window instead: the NARROW_POSE / DEFAULT_POSE body changes, which go
to a step_controller the simulation does not have, and the 1.5 s sleeps
upstream runs inside its policy, which would stall this loop.

Calibrate with `debug:=true` the same way scripts/prevent_falling.py does, and
build a bridge in the world first -- the stock rospider_room has none.

    ros2 launch rospider_gazebo gazebo.launch.py
    ros2 launch rospider_gazebo vision_demo.launch.py demo:=cross_bridge debug:=true
"""

import cv2
from rospider_gazebo import depth_probe, vision_demo
from rospider_gazebo.depth_probe import BRIDGE_ROIS, BridgePolicy
from rospider_gazebo.vision_demo import VisionDemo, banner, depth_color_map

#: Upstream's start pose, servo ids 19-24.
LOOK_POSE = ((19, 500), (20, 700), (21, 85), (22, 150), (23, 500), (24, 700))

CALIBRATION_FRAMES = 50


class CrossBridgeNode(VisionDemo):

    def __init__(self):
        super().__init__('cross_bridge', depth_frames=True,
                         cmd_vel=True, servos=True)
        self.debug = bool(self.param('debug', False))
        self.policy = BridgePolicy(
            float(self.param('plane_distance', 0.35)),
            edge_tolerance=float(self.param('edge_tolerance', 0.04)),
            steer_tolerance=float(self.param('steer_tolerance', 0.06)),
            forward_speed=float(self.param('forward_speed', 0.02)))
        self.samples = []

    def on_start(self):
        self.servos.set_servo_position(1.0, LOOK_POSE)
        self.get_logger().info(
            'calibrating over the floor' if self.debug else
            f'floor at {self.policy.plane_distance} m; looking for the bridge')

    def process(self, depth_mm):
        distances = {name: depth_probe.roi_distance(depth_mm, roi)
                     for name, roi in BRIDGE_ROIS.items()}
        view = depth_color_map(depth_mm)
        for roi in BRIDGE_ROIS.values():
            cv2.circle(view, depth_probe.roi_center(roi), 10, (0, 0, 0), -1)

        if self.debug:
            return self._calibrate(view, distances['center'])

        linear, angular = self.policy.update(
            distances['left_1'], distances['left'], distances['center'],
            distances['right'], distances['right_1'])
        self.drive(linear, angular)
        banner(view, ' '.join(f'{distances[n]:.3f}' for n in BRIDGE_ROIS),
               scale=0.6, color=(255, 255, 255))
        banner(view,
               f'{"ON BRIDGE" if self.policy.on_bridge else "SEARCHING"}  '
               f'{self.policy.wanted_pose}',
               position=(10, 140), scale=0.7,
               color=(0, 200, 255) if self.policy.on_bridge else (0, 255, 0))
        cv2.putText(view, f'v {linear:+.3f} m/s   w {angular:+.2f} rad/s',
                    (10, view.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 2)
        return view

    def _calibrate(self, view, center_distance):
        """Upstream calibrates off the centre probe alone; so does this."""
        if center_distance > 0:
            self.samples.append(center_distance)
        banner(view, f'CALIBRATING {len(self.samples)}/{CALIBRATION_FRAMES}',
               scale=0.7, color=(0, 200, 255))
        if len(self.samples) >= CALIBRATION_FRAMES:
            self.policy.plane_distance = round(
                sum(self.samples) / len(self.samples), 3)
            self.debug = False
            self.samples = []
            self.get_logger().info(
                f'floor is {self.policy.plane_distance} m away -- pass '
                f'plane_distance:={self.policy.plane_distance} next time')
        return view

    def on_stop(self):
        self.stop()


def main():
    vision_demo.main(CrossBridgeNode)


if __name__ == '__main__':
    main()
