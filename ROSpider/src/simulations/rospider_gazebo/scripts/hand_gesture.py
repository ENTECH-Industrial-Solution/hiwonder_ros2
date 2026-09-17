#!/usr/bin/env python3
"""Order the robot about with your fingers -- the gesture-control window.

A port of example/mediapipe_example/include/hand_gesture.py. Hold one of four
gestures steady for HOLD_FRAMES and the robot performs that gesture's move;
the window captions the gesture and greys out while a move is running, as
upstream's does.

The moves are the one real difference. Upstream plays action groups from
/home/ubuntu/software/actionset_editor/ActionGroups -- recorded servo
sequences that exist only on the robot image -- so each gesture drives
/controller/cmd_vel here instead, through the simulated gait. The pairing is
upstream's, one move per gesture:

    gun          attack           -> a short lunge forward
    hand_heart   twist_l          -> turn left on the spot
    OK           wave             -> turn right on the spot
    fist         forward_flutter  -> back off

The gesture classifier itself is upstream's, in rospider_gazebo/gestures.py.

Nobody is in the simulated world, so use the PC's camera for the hand while
the robot walks in Gazebo:

    ros2 launch rospider_gazebo gazebo.launch.py
    ros2 launch rospider_gazebo vision_demo.launch.py demo:=hand_gesture source:=webcam
"""

import threading
import time

import cv2
from rospider_gazebo import gestures, vision_demo
from rospider_gazebo.vision_demo import VisionDemo, banner

try:
    import mediapipe as mp
except ImportError as exc:                                  # pragma: no cover
    raise SystemExit('this demo needs mediapipe: pip install mediapipe') from exc

#: gesture -> (linear x m/s, angular z rad/s, seconds).
MOVES = {
    'gun': (0.05, 0.0, 2.0),
    'hand_heart': (0.0, 0.4, 2.0),
    'OK': (0.0, -0.4, 2.0),
    'fist': (-0.05, 0.0, 2.0),
}

#: Frames a gesture must hold before it fires, as upstream counts them.
HOLD_FRAMES = 20


class HandGestureNode(VisionDemo):

    def __init__(self):
        super().__init__('hand_gesture', flip=True, cmd_vel=True)
        self.drawing = mp.solutions.drawing_utils
        self.detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=float(self.param('detection_confidence',
                                                      0.4)),
            min_tracking_confidence=float(self.param('tracking_confidence',
                                                     0.4)))
        self.gesture = None          # the move the worker should run
        self.last_gesture = 'none'
        self.count = 0
        self.busy = False

    def on_start(self):
        threading.Thread(target=self._move_loop, daemon=True).start()
        self.get_logger().info(f'hold a gesture -- {", ".join(MOVES)} -- for '
                               f'{HOLD_FRAMES} frames to trigger its move')

    def _move_loop(self):
        """Run a gesture's move off the display thread, which cannot sleep."""
        while self.running:
            gesture = self.gesture
            if gesture is None:
                time.sleep(0.01)
                continue
            self.busy = True
            linear, angular, seconds = MOVES[gesture]
            self.get_logger().info(f'{gesture}: {linear:+.2f} m/s '
                                   f'{angular:+.2f} rad/s for {seconds}s')
            until = time.monotonic() + seconds
            while self.running and time.monotonic() < until:
                self.drive(linear, angular)     # sim_gait times cmd_vel out
                time.sleep(0.1)
            self.stop()
            self.gesture = None
            self.busy = False

    def process(self, frame):
        results = self.detector.process(cv2.cvtColor(frame,
                                                     cv2.COLOR_BGR2RGB))
        if self.busy:
            return banner(frame, 'RUNNING', color=(0, 200, 255))
        if not results.multi_hand_landmarks:
            self.count = 0
            self.last_gesture = 'none'
            return banner(frame, 'NO HAND', color=(200, 200, 200))

        height, width = frame.shape[:2]
        gesture = 'none'
        for hand in results.multi_hand_landmarks:
            self.drawing.draw_landmarks(frame, hand,
                                        mp.solutions.hands.HAND_CONNECTIONS)
            gesture = gestures.hand_gesture(
                gestures.landmarks_to_pixels((width, height), hand.landmark))

        if gesture in MOVES and gesture == self.last_gesture:
            self.count += 1
        else:
            self.count = 0
        self.last_gesture = gesture

        if self.count > HOLD_FRAMES:
            self.count = 0
            self.gesture = gesture

        banner(frame, gesture.upper())
        if gesture in MOVES:
            cv2.rectangle(frame, (10, 120),
                          (10 + int(300 * min(self.count, HOLD_FRAMES)
                                    / HOLD_FRAMES), 132),
                          (255, 255, 0), -1)
        return frame

    def on_stop(self):
        self.gesture = None
        self.stop()


def main():
    vision_demo.main(HandGestureNode)


if __name__ == '__main__':
    main()
