"""Click on a thing, then follow it by colour -- app/app/object_tracking.py's
maths (ColorPicker and ObjectTracker), without the node round it.

A click hands ColorPicker a pixel; it averages the 2x2 patch under that
pixel over `repeat` frames and answers with the mean LAB and BGR colour.
lab_band() turns that colour and the app's `threshold` into an inRange
band, find_blob() finds the biggest patch of that band, and FollowControl
is the pair of PIDs that turn the patch's position into a body twist.

Every constant is upstream's. Frames are BGR, as cv_bridge hands them out
here; upstream works in RGB, so its LAB conversion is COLOR_RGB2LAB and
this one COLOR_BGR2LAB -- same LAB either way.

cv2 and numpy only, no ROS, so test/test_color_picker.py can run it on
synthetic frames.
"""

import math

import cv2
import numpy as np

from rospider_gazebo.pid import PID, set_range

#: Frames a click is averaged over before tracking starts.
PICK_REPEAT = 10
#: Upstream tracks on a 320x200 downscale of the camera frame.
PROCESS_SIZE = (320, 200)
#: px^2 at PROCESS_SIZE; smaller contours are noise.
MIN_AREA = 40
#: Where in the (640x480) frame the tracker wants the target to sit.
STOP_POINT = (320, 300)
#: px at PROCESS_SIZE: a patch this close to the last one is the same object.
SAME_OBJECT_PX = 100


class ColorPicker:
    """The colour under a click, averaged over a few frames."""

    def __init__(self, x, y, repeat=PICK_REPEAT):
        self.x, self.y = int(x), int(y)
        self.repeat = repeat
        self.count = 0
        #: Running means so far, for drawing the pick ring in that colour.
        self.lab = None
        self.bgr = None
        self._lab = []
        self._bgr = []

    def sample(self, frame):
        """Add one frame. Returns ((l, a, b), (b, g, r)) once `repeat`
        frames are in, None before that."""
        h, w = frame.shape[:2]
        # Upstream reads image[y-1:y+1, x-1:x+1]; keep the click one pixel
        # off the edge so that patch is never empty.
        x = min(max(self.x, 1), w - 1)
        y = min(max(self.y, 1), h - 1)
        patch = frame[y - 1:y + 1, x - 1:x + 1]
        self._bgr.append(patch.reshape(-1, 3).mean(axis=0))
        self._lab.append(cv2.cvtColor(patch, cv2.COLOR_BGR2LAB)
                         .reshape(-1, 3).mean(axis=0))
        del self._bgr[:-self.repeat], self._lab[:-self.repeat]
        self.count = min(self.count + 1, self.repeat)
        self.lab = tuple(int(v) for v in np.mean(self._lab, axis=0))
        self.bgr = tuple(int(v) for v in np.mean(self._bgr, axis=0))
        if self.count < self.repeat:
            return None
        return self.lab, self.bgr


def lab_band(lab, threshold):
    """(lower, upper) LAB bounds round `lab`: upstream's +-50*threshold on
    a and b, twice that on L, clipped to 0..255."""
    span = 50 * threshold
    lower = (lab[0] - 2 * span, lab[1] - span, lab[2] - span)
    upper = (lab[0] + 2 * span, lab[1] + span, lab[2] + span)
    clip = lambda v: int(min(max(v, 0), 255))          # noqa: E731
    return tuple(clip(v) for v in lower), tuple(clip(v) for v in upper)


def find_blob(frame, lower, upper, last=None):
    """(x, y, r) of the patch inside the band, in frame pixels, or None.

    Upstream's pipeline: downscale, LAB, blur, inRange, open. With no
    `last`, the largest patch; given the last answer, the patch nearest to
    it when one is within SAME_OBJECT_PX, so a bigger patch of the same
    colour elsewhere (a poster on the wall) does not steal the target.
    Upstream wrote that rule but never set its `last_color_circle`, so on
    the robot the largest patch always wins.
    """
    h, w = frame.shape[:2]
    small = cv2.resize(frame, PROCESS_SIZE)
    lab = cv2.GaussianBlur(cv2.cvtColor(small, cv2.COLOR_BGR2LAB), (5, 5), 5)
    mask = cv2.inRange(lab, lower, upper)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.dilate(cv2.erode(mask, kernel), kernel)
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_NONE)[-2]
    contours = [c for c in contours if math.fabs(cv2.contourArea(c)) > MIN_AREA]
    if not contours:
        return None
    scale_x, scale_y = w / PROCESS_SIZE[0], h / PROCESS_SIZE[1]
    circles = [cv2.minEnclosingCircle(c) for c in contours]
    chosen = None
    if last is not None:
        lx, ly = last[0] / scale_x, last[1] / scale_y
        nearest = min(circles, key=lambda c: math.hypot(c[0][0] - lx, c[0][1] - ly))
        if math.hypot(nearest[0][0] - lx, nearest[0][1] - ly) < SAME_OBJECT_PX:
            chosen = nearest
    if chosen is None:
        chosen = max(circles, key=lambda c: c[1])
    (x, y), r = chosen
    return x * scale_x, y * scale_y, r * scale_x


class FollowControl:
    """Upstream ObjectTracker's two PIDs: the target's y sets the forward
    speed (higher in the frame = farther away), its x the turn rate.

    `max_speed` (m/s) and `max_turn` (rad/s) are upstream's output limits,
    0.05 and 0.1, settable for the Color Tracking exercise; the final
    approach keeps upstream's 0.01 cap, or `max_speed` when that is lower."""

    def __init__(self, stop_point=STOP_POINT, max_speed=0.05, max_turn=0.1):
        self.x_stop, self.y_stop = stop_point
        self.max_speed = max_speed
        self.max_turn = max_turn
        self.pid_yaw = PID(0.009, 0.0, 0.001)
        self.pid_dist = PID(0.002, 0.0, 0.0)

    def clear(self):
        self.pid_yaw.clear()
        self.pid_dist.clear()

    def update(self, x, y, now=None):
        """(linear_x, angular_z) for a target at frame pixel (x, y)."""
        linear = 0.0
        angular = 0.0
        # The PIDs keep SetPoint 0 and are fed the offset, so the output
        # has the sign of stop - target: a target above the stop point is
        # far away and gives a positive (forward) speed, one to the right
        # a negative (clockwise) turn.
        dy = y - self.y_stop
        if abs(dy) > 60:
            self.pid_dist.update(dy, now)
            linear = set_range(self.pid_dist.output, -self.max_speed, self.max_speed)
        elif abs(dy) > 30:
            self.pid_dist.update(dy, now)
            slow = min(0.01, self.max_speed)
            linear = set_range(self.pid_dist.output, -slow, slow)
        else:
            self.pid_dist.clear()

        dx = x - self.x_stop
        if abs(dx) > 40:
            self.pid_yaw.update(dx, now)
            angular = set_range(self.pid_yaw.output, -self.max_turn, self.max_turn)
        else:
            self.pid_yaw.clear()
        return float(linear), float(angular)
