#!/usr/bin/env python3
"""Draw the hand skeleton -- the hand-detection window.

A port of example/mediapipe_example/include/hand_detect.py: mirror the frame,
find up to two hands, draw their 21 landmarks and the bones between them.
Nothing moves the robot.

Upstream uses MediaPipe's Tasks API and ships the hand_landmarker.task model
file alongside the node. This uses mp.solutions.hands instead, which downloads
nothing and needs no model file, and names the gesture each hand is making
while it is at it (rospider_gazebo/gestures.py).

The simulated world has nobody in it, so point this at the PC's own camera:

    ros2 launch rospider_gazebo vision_demo.launch.py demo:=hand_detect source:=webcam
"""

import cv2
from rospider_gazebo import gestures, vision_demo
from rospider_gazebo.vision_demo import VisionDemo, banner

try:
    import mediapipe as mp
except ImportError as exc:                                  # pragma: no cover
    raise SystemExit('this demo needs mediapipe: pip install mediapipe') from exc


class HandDetectNode(VisionDemo):

    def __init__(self):
        super().__init__('hand_detect', flip=True)
        self.drawing = mp.solutions.drawing_utils
        self.detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=int(self.param('max_hands', 2)),
            min_detection_confidence=float(self.param('detection_confidence',
                                                      0.4)),
            min_tracking_confidence=float(self.param('tracking_confidence',
                                                     0.4)))

    def process(self, frame):
        # MediaPipe wants RGB; the frame off the camera is BGR.
        results = self.detector.process(cv2.cvtColor(frame,
                                                     cv2.COLOR_BGR2RGB))
        if not results.multi_hand_landmarks:
            return banner(frame, 'NO HAND', color=(200, 200, 200))

        height, width = frame.shape[:2]
        names = []
        for hand in results.multi_hand_landmarks:
            self.drawing.draw_landmarks(frame, hand,
                                        mp.solutions.hands.HAND_CONNECTIONS)
            landmarks = gestures.landmarks_to_pixels((width, height),
                                                     hand.landmark)
            names.append(gestures.hand_gesture(landmarks))
        return banner(frame, '  '.join(n.upper() for n in names))


def main():
    vision_demo.main(HandDetectNode)


if __name__ == '__main__':
    main()
