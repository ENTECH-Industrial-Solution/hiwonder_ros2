#!/usr/bin/env python3
"""Draw in the air and have the shape named -- the finger-trajectory window.

A port of example/mediapipe_example/include/finger_trajectory.py. Hold up one
index finger to start drawing, move it to trace a shape, then open all five
fingers: the path is closed, cleaned up and named Triangle, Square, Circle or
Star in a second window. Space clears the current path.

Nothing moves the robot -- upstream's node does not either.

The shape recogniser is rospider_gazebo/shape_detect.trajectory_shape(),
tested on drawn paths in test/test_shape_detect.py.

    ros2 launch rospider_gazebo vision_demo.launch.py demo:=finger_trajectory source:=webcam
"""

import time

import cv2
import numpy as np
from rospider_gazebo import gestures, shape_detect, vision_demo
from rospider_gazebo.vision_demo import VisionDemo, banner

try:
    import mediapipe as mp
except ImportError as exc:                                  # pragma: no cover
    raise SystemExit('this demo needs mediapipe: pip install mediapipe') from exc

#: Frames of "one finger up" before drawing starts, as upstream counts them.
START_FRAMES = 20
#: Pixels the fingertip must travel before another point is recorded.
POINT_SPACING = 5
#: Seconds without a hand before the path is abandoned.
LOST_TIMEOUT = 2.0

TRACK_WINDOW = 'track'


def draw_points(image, points, thickness=4, color=(255, 0, 0)):
    """Join the recorded fingertip positions up, upstream's draw_points()."""
    points = np.array(points).astype(np.int64)
    for start, end in zip(points, points[1:]):
        cv2.line(image, tuple(start), tuple(end), color, thickness)


class FingerTrajectoryNode(VisionDemo):

    def __init__(self):
        super().__init__('finger_trajectory', flip=True)
        self.drawing = mp.solutions.drawing_utils
        self.detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=float(self.param('detection_confidence',
                                                      0.6)),
            min_tracking_confidence=float(self.param('tracking_confidence',
                                                     0.05)))
        self.tracking = False
        self.points = []
        self.start_count = 0
        self.last_seen = time.monotonic()
        self.shape = None

    def on_start(self):
        self.get_logger().info(
            'hold up one index finger to start drawing, open all five to '
            'finish, Space to clear')

    def on_key(self, key):
        if key == ord(' '):
            self.points = []
            return True
        return False

    def process(self, frame):
        results = self.detector.process(cv2.cvtColor(frame,
                                                     cv2.COLOR_BGR2RGB))
        height, width = frame.shape[:2]

        if not results.multi_hand_landmarks:
            if self.tracking and time.monotonic() - self.last_seen > LOST_TIMEOUT:
                self.tracking = False
                self.points = []
            return self._caption(frame)

        self.last_seen = time.monotonic()
        for hand in results.multi_hand_landmarks:
            self.drawing.draw_landmarks(frame, hand,
                                        mp.solutions.hands.HAND_CONNECTIONS)
            landmarks = gestures.landmarks_to_pixels((width, height),
                                                     hand.landmark)
            gesture = gestures.hand_gesture(landmarks)
            fingertip = landmarks[8].tolist()

            if not self.tracking:
                self.start_count = self.start_count + 1 if gesture == 'one' else 0
                if self.start_count > START_FRAMES:
                    self.tracking = True
                    self.start_count = 0
                    self.points = []
                    self.shape = None
            elif gesture == 'five':
                self.tracking = False
                self._finish()
            elif (not self.points
                  or gestures.distance(self.points[-1], fingertip)
                  > POINT_SPACING):
                self.points.append(fingertip)

        if len(self.points) > 1:
            draw_points(frame, self.points)
        return self._caption(frame)

    def _finish(self):
        """Name the drawn shape and show it in its own window."""
        name, view = shape_detect.trajectory_shape(self.points)
        self.shape = name
        self.get_logger().info(f'drawn shape: {name or "unrecognised"}')
        if self.show:
            cv2.imshow(TRACK_WINDOW, view)

    def _caption(self, frame):
        if self.tracking:
            return banner(frame, f'DRAWING ({len(self.points)})', scale=0.9)
        if self.shape:
            return banner(frame, self.shape.upper())
        return banner(frame, 'SHOW ONE FINGER', scale=0.8,
                      color=(200, 200, 200))


def main():
    vision_demo.main(FingerTrajectoryNode)


if __name__ == '__main__':
    main()
