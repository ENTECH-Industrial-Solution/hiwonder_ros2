#!/usr/bin/env python3
"""Keep a face in the middle of the picture -- the face-tracking window.

A port of example/mediapipe_example/include/face_track.py. The nearest face to
the centre of the frame is picked, and two PIDs pan and tilt the camera to
bring it back to the middle. Five consecutive detections are needed before
tracking starts, and the count runs back down when the face is lost, exactly
as upstream does it, so one stray detection cannot throw the camera.

Upstream turns the tilt into a wrist height through the arm_kinematics IK
service -- an aarch64-only library this workspace cannot load. The camera
rides on link4, so tilting is what joint4 already does: here the pan PID drives
servo 19 (joint1) and the tilt PID drives servo 22 (joint4) directly, and the
arm starts from the pose that levels the camera (the same one
`gazebo.launch.py arm_pose:=horizontal` sets).

The simulated world has nobody in it, so use the PC's camera:

    ros2 launch rospider_gazebo gazebo.launch.py
    ros2 launch rospider_gazebo vision_demo.launch.py demo:=face_track source:=webcam
"""

import cv2
from rospider_gazebo import gestures, vision_demo
from rospider_gazebo.pid import PID, set_range
from rospider_gazebo.vision_demo import VisionDemo, banner

try:
    import mediapipe as mp
except ImportError as exc:                                  # pragma: no cover
    raise SystemExit('this demo needs mediapipe: pip install mediapipe') from exc

#: Start pose, servo ids 19-24: the arm shape of the robot's init action with
#: joint4 levelling the camera (joint2 0.628, joint3 -1.927, joint4 -0.286 rad).
LEVEL_POSE = ((19, 500), (20, 650), (21, 40), (22, 432), (23, 500), (24, 500))

PAN_PULSE = (200, 800)      # servo 19 travel, upstream's limits
TILT_PULSE = (332, 532)     # servo 22 travel, +/- 100 counts (~24 deg)

#: Detections in a row before the camera starts following, and the ceiling
#: the counter is held at so a lost face decays in a bounded time.
LOCK_FRAMES = 5
LOCK_CEILING = 20

BOX_COLOR = (0, 255, 0)
POINT_COLOR = (0, 0, 255)


def face_boxes(results, width, height):
    """[(x1, y1, x2, y2)], [[(x, y), ...]] in pixels (sdk.common.mp_face_location)."""
    boxes, keypoints = [], []
    for detection in results.detections or []:
        box = detection.location_data.relative_bounding_box
        x1, y1 = max(box.xmin * width, 0), max(box.ymin * height, 0)
        boxes.append((x1, y1,
                      min(x1 + box.width * width, width),
                      min(y1 + box.height * height, height)))
        keypoints.append([(p.x * width, p.y * height)
                          for p in detection.location_data.relative_keypoints])
    return boxes, keypoints


class FaceTrackNode(VisionDemo):

    def __init__(self):
        super().__init__('face_track', servos=True)
        self.detector = mp.solutions.face_detection.FaceDetection(
            min_detection_confidence=float(self.param('detection_confidence',
                                                      0.3)))
        self.pan = float(self.param('pan_pulse', 500))
        self.tilt = float(self.param('tilt_pulse', LEVEL_POSE[3][1]))
        self.pid_pan = PID(float(self.param('pan_gain', 0.055)), 0.0, 0.0)
        self.pid_tilt = PID(float(self.param('tilt_gain', 0.05)), 0.0, 0.0)
        self.locked = 0

    def on_start(self):
        self.servos.set_servo_position(1.5, LEVEL_POSE)
        self.get_logger().info('looking for a face')

    def process(self, frame):
        results = self.detector.process(cv2.cvtColor(frame,
                                                     cv2.COLOR_BGR2RGB))
        height, width = frame.shape[:2]
        boxes, keypoints = face_boxes(results, width, height)

        if not boxes:
            if self.locked > 0:
                self.locked -= 1
            else:
                self.pid_pan.clear()
                self.pid_tilt.clear()
            return banner(frame, 'NO FACE', color=(200, 200, 200))

        self.locked = min(self.locked + 1, LOCK_CEILING)
        for box, points in zip(boxes, keypoints):
            cv2.rectangle(frame, (int(box[0]), int(box[1])),
                          (int(box[2]), int(box[3])), BOX_COLOR, 2)
            for x, y in points:
                cv2.circle(frame, (int(x), int(y)), 2, POINT_COLOR, 2)

        # The face closest to the centre of the frame wins, as upstream picks it.
        centre = min((gestures.box_center(b) for b in boxes),
                     key=lambda c: gestures.distance(c, (width / 2,
                                                         height / 2)))
        if self.locked < LOCK_FRAMES:
            return banner(frame, f'LOCKING {self.locked}/{LOCK_FRAMES}',
                          scale=0.8, color=(0, 200, 255))

        # Both PIDs see setpoint - measurement, so a face left of centre
        # gives a positive pan step (servo 19 up = turn left) and a face
        # above centre a positive tilt step (servo 22 up = look up).
        self.pid_pan.SetPoint = width / 2.0
        self.pid_pan.update(centre[0])
        self.pan = set_range(self.pan + self.pid_pan.output, *PAN_PULSE)

        self.pid_tilt.SetPoint = height / 2.0
        self.pid_tilt.update(centre[1])
        self.tilt = set_range(self.tilt + self.pid_tilt.output, *TILT_PULSE)

        self.servos.set_servo_position(0.05, ((19, int(self.pan)),
                                              (22, int(self.tilt))))
        cv2.circle(frame, (int(centre[0]), int(centre[1])), 5, (0, 255, 255), -1)
        return banner(frame, f'PAN {int(self.pan)}  TILT {int(self.tilt)}',
                      scale=0.8)

    def on_stop(self):
        self.servos.set_servo_position(1.0, LEVEL_POSE)


def main():
    vision_demo.main(FaceTrackNode)


if __name__ == '__main__':
    main()
