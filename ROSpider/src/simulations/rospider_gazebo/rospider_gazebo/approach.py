"""Walk the body up to a point until it sits where the arm can reach.

scripts/track_and_grab.py's walk mode: the block's position comes from the
camera (a pixel plus its depth, put through TF into base_footprint), and
this turns it into a body twist for /controller/cmd_vel -- a P-controller
per axis.

The body faces the block first and then walks straight in: the further the
block is off to the side, the less the body walks, so far off-axis it turns
in place, and once it faces the block it walks at full speed with a small
sideways trim. It stops only with the block dead ahead at the pedestal
distance, so the body stands square on to it and pick_and_place's look
pose, which faces straight ahead, finds it in the middle of the frame.
Turning in place needs room: the robot starts 1 cm from the pick pedestal
and climbs it if it turns there, which is why track_and_grab backs off
before it walks.

Pure Python, so test/test_approach.py can drive it point by point.
"""

import math


def clamp(value, limit):
    return max(-limit, min(limit, value))


class Approach:
    """step(x, y) -> (linear_x, linear_y, angular_z, done) for the block at
    (x, y) metres in base_footprint.

    target_x           where the block must end up in front of the body --
                       the pick pedestal's spot, 0.235 m, where
                       pick_and_place's look pose and the arm's reach are
                       calibrated.
    tolerance          how close to target_x counts as there.
    lateral_tolerance  how far off the centre line the block may still be:
                       1.5 cm at target_x is under 4 degrees of heading.
    turn_first_rad     the bearing from which the body only turns; below
                       it the walk speeds up as the bearing closes.
    """

    def __init__(self, target_x=0.235, tolerance=0.04, lateral_tolerance=0.015,
                 turn_first_rad=math.radians(15.0), gain=0.5, turn_gain=1.5,
                 max_linear=0.10, min_linear=0.03, max_angular=0.4):
        self.target_x = target_x
        self.tolerance = tolerance
        self.lateral_tolerance = lateral_tolerance
        self.turn_first_rad = turn_first_rad
        self.gain = gain
        self.turn_gain = turn_gain
        self.max_linear = max_linear
        self.min_linear = min_linear
        self.max_angular = max_angular

    def _linear(self, error, deadband):
        if abs(error) < deadband:
            return 0.0
        speed = clamp(self.gain * error, self.max_linear)
        return math.copysign(max(abs(speed), self.min_linear), speed)

    def step(self, x, y):
        ex = x - self.target_x
        if abs(ex) < self.tolerance and abs(y) < self.lateral_tolerance:
            return 0.0, 0.0, 0.0, True
        bearing = math.atan2(y, x)
        facing = max(0.0, 1.0 - abs(bearing) / self.turn_first_rad)
        return (facing * self._linear(ex, self.tolerance),
                facing * self._linear(y, self.lateral_tolerance),
                clamp(self.turn_gain * bearing, self.max_angular), False)
