"""Walk the body up to a point until it sits where the arm can reach.

scripts/track_and_grab.py's walk mode: the block's position comes from the
camera (a pixel plus its depth, put through TF into base_footprint), and
this turns it into a body twist for /controller/cmd_vel, the way
scripts/mission.py's dock creeps to a pose -- a P-controller per axis.

The hexapod walks in any direction, so the three axes run together: the
body slides towards the spot from which the block would sit at the
pedestal distance straight ahead *as the body is turned now*, while it
turns to face the block. With the block off to one side that first move
is backwards and sideways, away from whatever is in front -- which
matters, because the robot starts 1 cm from the pick pedestal and cannot
so much as turn in place there without climbing it.

Pure Python, so test/test_approach.py can drive it point by point.
"""

import math


def clamp(value, limit):
    return max(-limit, min(limit, value))


class Approach:
    """step(x, y) -> (linear_x, linear_y, angular_z, done) for the block at
    (x, y) metres in base_footprint.

    target_x   where the block must end up in front of the body -- the
               pick pedestal's spot, 0.235 m, where pick_and_place's look
               pose and the arm's reach are calibrated.
    tolerance  how close, on each axis, counts as there. The bearing then
               takes care of itself: atan(tolerance / target_x) is under
               10 degrees.
    """

    def __init__(self, target_x=0.235, tolerance=0.04, gain=0.5, turn_gain=1.5,
                 max_linear=0.10, min_linear=0.03, max_angular=0.4):
        self.target_x = target_x
        self.tolerance = tolerance
        self.gain = gain
        self.turn_gain = turn_gain
        self.max_linear = max_linear
        self.min_linear = min_linear
        self.max_angular = max_angular

    def _linear(self, error):
        if abs(error) < self.tolerance:
            return 0.0
        speed = clamp(self.gain * error, self.max_linear)
        return math.copysign(max(abs(speed), self.min_linear), speed)

    def step(self, x, y):
        ex, ey = x - self.target_x, y
        if abs(ex) < self.tolerance and abs(ey) < self.tolerance:
            return 0.0, 0.0, 0.0, True
        bearing = math.atan2(y, x)
        return (self._linear(ex), self._linear(ey),
                clamp(self.turn_gain * bearing, self.max_angular), False)
