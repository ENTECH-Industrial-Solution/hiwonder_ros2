"""Follow a line on the floor -- app/app/line_following.py's maths without
the node round it.

find_line() is upstream's LineFollower.__call__: three thin horizontal
strips of the frame (ROIS, weighted so the nearest counts most), the
largest patch of the line's colour in each, and the weighted centre turned
into a deflection angle. LineControl is its PID and the twist it makes;
ObstacleStop is its LiDAR guard, which parks the robot while anything is
within `stop_distance` in the front sector.

Every constant is upstream's, including the aim point at w/1.8 rather than
w/2, which steers the line a little left of centre in the picture. Frames
are BGR here, so the LAB conversion is COLOR_BGR2LAB; same LAB either way.

ObstacleStop takes the scan's angle_min and angle_increment and picks the
front sector by angle. Upstream reads ranges[:n] and ranges[::-1][:n],
which is the front only when index 0 points straight ahead, as it does on
the robot's LiDAR; the simulated one starts at -pi.

cv2 and numpy only, no ROS, so test/test_line_follower.py runs it on
synthetic frames.
"""

import math

import cv2
import numpy as np

from rospider_gazebo.pid import PID, set_range

#: (top, bottom, left, right, weight): fractions of the frame, nearest
#: strip first. The weights sum to 1.
ROIS = ((0.81, 0.83, 0, 1, 0.7), (0.69, 0.71, 0, 1, 0.2), (0.57, 0.59, 0, 1, 0.1))
WEIGHT_SUM = 1.0
#: px^2 in a strip; smaller contours are noise.
MIN_AREA = 30
#: Where across the frame the line is steered to, as a fraction of its width.
AIM_X = 1 / 1.8
#: Sector in front of the robot the LiDAR guard watches, radians, total.
SCAN_SECTOR = math.radians(45)


def find_line(frame, lower, upper, rois=ROIS):
    """(deflection_angle, boxes) for one BGR frame.

    `boxes` is one (corners, centre) per strip that saw the line, in frame
    pixels: the minAreaRect corners as a 4x2 int array and the midpoint of
    its first and third corner, which is what upstream draws. The angle is
    None when no strip saw it.
    """
    h, w = frame.shape[:2]
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    centroid_sum = 0.0
    boxes = []
    for top, bottom, left, right, weight in rois:
        y0, x0 = int(top * h), int(left * w)
        strip = frame[y0:int(bottom * h), x0:int(right * w)]
        lab = cv2.GaussianBlur(cv2.cvtColor(strip, cv2.COLOR_BGR2LAB), (3, 3), 3)
        mask = cv2.inRange(lab, lower, upper)
        mask = cv2.dilate(cv2.erode(mask, kernel), kernel)
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_TC89_L1)[-2]
        contours = [c for c in contours if math.fabs(cv2.contourArea(c)) > MIN_AREA]
        if not contours:
            continue
        largest = max(contours, key=lambda c: math.fabs(cv2.contourArea(c)))
        box = np.intp(cv2.boxPoints(cv2.minAreaRect(largest)))
        box[:, 0] += x0
        box[:, 1] += y0
        centre = ((box[0, 0] + box[2, 0]) / 2, (box[0, 1] + box[2, 1]) / 2)
        boxes.append((box, centre))
        centroid_sum += centre[0] * weight
    if not boxes:
        return None, boxes
    centre_x = centroid_sum / WEIGHT_SUM
    return -math.atan((centre_x - w * AIM_X) / (h / 2.0)), boxes


class LineControl:
    """Upstream's steering: PID(1.1) on the deflection angle, a fifth of
    its output as the turn rate within +-0.35 rad/s, walking at 0.05 m/s
    -- and standing still when the PID has nothing to say. The three
    numbers are parameters (upstream's are the defaults) so the Line
    Following exercise can hand them to the participant."""

    def __init__(self, kp=1.1, speed=0.05, max_turn=0.35):
        self.pid = PID(kp, 0.0, 0.0)
        self.speed = speed
        self.max_turn = max_turn

    def clear(self):
        self.pid.clear()

    def update(self, angle, now=None):
        """(linear_x, angular_z) for a deflection angle in radians."""
        self.pid.update(angle, now)
        output = self.pid.output
        angular = set_range(-output / 5, -self.max_turn, self.max_turn)
        linear = self.speed if output else 0.0
        return float(linear), float(angular)


class ObstacleStop:
    """Upstream's LiDAR guard: stopped while anything within `sector` in
    front is nearer than `stop_distance`, released again after
    `release_after` clear scans in a row. A scan with no finite, non-zero
    reading in the sector changes nothing."""

    def __init__(self, stop_distance=0.4, sector=SCAN_SECTOR, release_after=5):
        self.stop_distance = stop_distance
        self.sector = sector
        self.release_after = release_after
        self.stopped = False
        self.count = 0

    def update(self, ranges, angle_min, angle_increment):
        """Feed one scan; returns whether the robot should be stopped."""
        ranges = np.asarray(ranges, dtype=float)
        angles = angle_min + np.arange(len(ranges)) * angle_increment
        angles = (angles + math.pi) % (2 * math.pi) - math.pi
        front = ranges[np.abs(angles) <= self.sector / 2]
        front = front[np.isfinite(front) & (front > 0)]
        if len(front) == 0:
            return self.stopped
        if front.min() < self.stop_distance:
            self.stopped = True
        else:
            self.count += 1
            if self.count > self.release_after:
                self.count = 0
                self.stopped = False
        return self.stopped
