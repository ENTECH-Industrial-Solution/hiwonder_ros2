#!/usr/bin/env python3
"""Make the robot copy your arms -- the body-control window.

A port of example/mediapipe_example/include/pose_control.py. Raise both hands
over your head to arm it, hold both arms straight out to the sides to start
copying, and the robot's two front legs follow your arms: shoulder angle to
the hip servo, elbow angle to the knee, and how far your elbow sits from your
shoulder in the picture to the leg's yaw. Cross your raised arms to stop.

Upstream drives servo ids 5/3/1 and 6/4/2, which are the left-front and
right-front legs (driver/servo_controller/config/servo_controller.yaml), so
"arms" really are the front legs. Every pulse, limit and threshold below is
upstream's; ServoClient turns the pulses into the radians the simulated
leg_controller wants.

Two things are different here. The buzzer beeps that mark each state change
upstream have no counterpart in Gazebo, so the window's caption carries the
state instead. And scripts/sim_gait.py commands the same leg controller
whenever /controller/cmd_vel is non-zero -- do not drive the robot while it is
imitating, or the two will fight over the front legs.

Nobody is in the simulated world, so use the PC's camera:

    ros2 launch rospider_gazebo gazebo.launch.py
    ros2 launch rospider_gazebo vision_demo.launch.py demo:=pose_control source:=webcam
"""

import enum
import time

import cv2
from rospider_gazebo import gestures, vision_demo
from rospider_gazebo.pid import set_range
from rospider_gazebo.vision_demo import VisionDemo

try:
    import mediapipe as mp
except ImportError as exc:                                  # pragma: no cover
    raise SystemExit('this demo needs mediapipe: pip install mediapipe') from exc

#: Upstream shows the frame resized to this (pose_control.py display_size).
DISPLAY_SIZE = (int(640 * 8 / 4), int(400 * 8 / 4))

PULSE_PER_DEGREE = 1000 / 240

#: Neutral pulse of each mirrored servo, upstream's *_DEFAULT constants.
NEUTRAL = {
    'right_1': 340, 'right_2': 425, 'right_3': 235,
    'left_1': 660, 'left_2': 575, 'left_3': 775,
}

#: Travel each mirrored servo is allowed, upstream's SERVO_LIMITS.
LIMITS = {
    'right_1': (300, 600), 'right_2': (100, 600), 'right_3': (100, 500),
    'left_1': (400, 700), 'left_2': (400, 900), 'left_3': (500, 900),
}

#: Servo ids: right_* is the left-front leg (coxa 5, femur 3, tibla 1),
#: left_* the right-front one (coxa 6, femur 4, tibla 2) -- mirrored, because
#: the robot faces the person.
SERVO_IDS = {
    'right_1': 5, 'right_2': 3, 'right_3': 1,
    'left_1': 6, 'left_2': 4, 'left_3': 2,
}

#: Frames of the "arms out" pose before copying starts.
FLAT_FRAMES = 2
#: Seconds after a stop before the raised-hands pose can arm it again.
REARM_DELAY = 5.0


class State(enum.Enum):
    NULL = 'NULL'
    ARMED = 'ARMED'
    IMITATION = 'IMITATION'


class PoseControlNode(VisionDemo):

    window = 'image'

    def __init__(self):
        super().__init__('pose_control', flip=True, servos=True)
        self.drawing = mp.solutions.drawing_utils
        self.pose = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=int(self.param('model_complexity', 1)),
            min_detection_confidence=float(self.param('detection_confidence',
                                                      0.6)),
            min_tracking_confidence=float(self.param('tracking_confidence',
                                                     0.7)))
        self.state = State.NULL
        self.timestamp = 0.0
        self.flat_count = 0
        self.right_reach = 1.0
        self.left_reach = 1.0

    def on_start(self):
        self._reset_legs()
        self.get_logger().info(
            'raise both hands over your head to arm, then hold both arms '
            'straight out to start copying; cross your raised arms to stop')

    def _reset_legs(self):
        self.servos.set_servo_position(
            1.0, tuple((SERVO_IDS[name], NEUTRAL[name]) for name in NEUTRAL))

    def process(self, frame):
        results = self.pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        if not results.pose_landmarks:
            return cv2.resize(frame, DISPLAY_SIZE)
        self.drawing.draw_landmarks(frame, results.pose_landmarks,
                                    mp.solutions.pose.POSE_CONNECTIONS)
        height, width = frame.shape[:2]
        landmarks = gestures.landmarks_to_pixels(
            (width, height), results.pose_landmarks.landmark)
        # Upstream's mp_pose_landmarks(): a blue dot on every landmark, then
        # the two shoulders and the right elbow in their own colours.
        for cx, cy in landmarks:
            cv2.circle(frame, (int(cx), int(cy)), 5, (255, 0, 0), cv2.FILLED)
        for index, color in ((11, (255, 255, 0)), (12, (0, 255, 255)),
                             (14, (0, 255, 0))):
            cv2.circle(frame, tuple(int(v) for v in landmarks[index]), 5,
                       color, cv2.FILLED)

        if self.state is State.NULL:
            if (time.monotonic() - self.timestamp > REARM_DELAY
                    and gestures.is_pentagon(landmarks)):
                self.state = State.ARMED
                self.timestamp = time.monotonic()
                self.get_logger().info('armed -- now hold both arms out')
        elif self.state is State.ARMED:
            if gestures.is_flat(landmarks, 30):
                self.flat_count += 1
                if self.flat_count > FLAT_FRAMES:
                    self._activate(landmarks)
            else:
                self.flat_count = 0
        elif gestures.is_cross(landmarks):
            self._stop_imitating()
        else:
            self._mirror(landmarks)

        return cv2.resize(frame, DISPLAY_SIZE)

    def _activate(self, landmarks):
        """Record the arm lengths this person's yaw mapping is scaled by."""
        self.state = State.IMITATION
        self.flat_count = 0
        self.right_reach = max(gestures.distance(landmarks[13], landmarks[11]),
                               1.0)
        self.left_reach = max(gestures.distance(landmarks[12], landmarks[14]),
                              1.0)
        self.get_logger().info('imitating')

    def _stop_imitating(self):
        self.state = State.NULL
        self.timestamp = time.monotonic()
        self._reset_legs()
        self.get_logger().info('stopped imitating')

    def _mirror(self, landmarks):
        """Upstream's adjust_servos(), pulse for pulse."""
        left_2 = gestures.get_angle(landmarks[11], landmarks[12], landmarks[14])
        left_3 = gestures.get_angle(landmarks[12], landmarks[14], landmarks[16])
        right_2 = gestures.get_angle(landmarks[12], landmarks[11], landmarks[13])
        right_3 = gestures.get_angle(landmarks[11], landmarks[13], landmarks[15])

        right_reach = gestures.distance(landmarks[13], landmarks[11])
        left_reach = gestures.distance(landmarks[12], landmarks[14])
        # An arm pointing at the camera is foreshortened, so the elbow sits
        # closer to the shoulder in the picture; that shortening becomes the
        # leg's yaw.
        left_1 = set_range(90.0 - left_reach / self.left_reach * 90.0, 0, 120)
        right_1 = set_range(90.0 - right_reach / self.right_reach * 90.0,
                            0, 120)

        pulses = {
            'right_1': NEUTRAL['right_1'] + PULSE_PER_DEGREE * right_1,
            'right_2': NEUTRAL['right_2'] + PULSE_PER_DEGREE * right_2,
            'right_3': NEUTRAL['right_3'] + PULSE_PER_DEGREE * right_3,
            'left_1': NEUTRAL['left_1'] - PULSE_PER_DEGREE * left_1,
            'left_2': NEUTRAL['left_2'] + PULSE_PER_DEGREE * left_2,
            'left_3': NEUTRAL['left_3'] + PULSE_PER_DEGREE * left_3,
        }
        self.servos.set_servo_position(
            0.1, tuple((SERVO_IDS[name],
                        int(set_range(pulse, *LIMITS[name])))
                       for name, pulse in pulses.items()))

    def on_stop(self):
        self._reset_legs()


def main():
    vision_demo.main(PoseControlNode)


if __name__ == '__main__':
    main()
