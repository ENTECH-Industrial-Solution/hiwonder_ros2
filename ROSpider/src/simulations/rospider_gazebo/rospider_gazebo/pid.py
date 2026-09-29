"""The PID the tracking demos use, and the two range helpers beside it.

A port of driver/sdk/sdk/pid.py and two helpers from driver/sdk/sdk/common.py,
kept here so a demo in this package needs nothing from the robot-only sdk
package. Gains carried over from an upstream demo mean the same thing as they
did there, because the arithmetic is the same.

Deliberately stdlib-only, so test/test_pid.py runs without ROS.
"""

import time


class PID:
    """Ivmech-style PID, as upstream ships it.

    Differences from sdk/pid.py, both bugs rather than behaviour worth
    keeping:

      * D acts on the error's derivative (delta_error / delta_time). Upstream
        forgets to multiply by Kd there, then multiplies by Kd again in the
        sum, which is the same thing -- except that its DTerm is left holding
        the raw derivative. Here DTerm holds Kd * derivative and the sum adds
        it directly, so the output is identical and DTerm reads honestly.
      * `clear()` no longer resets SetPoint. Upstream's demos set SetPoint
        once and call clear() whenever the target is lost, which silently
        moves the set point to 0; every one of them then writes SetPoint
        again on the next frame, so nothing depended on the reset.
    """

    def __init__(self, P=0.2, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.SetPoint = 0.0
        self.sample_time = 0.0
        self.windup_guard = 20.0
        self.current_time = time.time()
        self.last_time = self.current_time
        self.clear()

    def clear(self):
        """Forget the accumulated error, keeping the gains and the set point."""
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.output = 0.0

    def update(self, feedback_value, current_time=None):
        """Recompute `output` from a new measurement.

        `current_time` is for tests; left out, the clock is read here.
        """
        error = self.SetPoint - feedback_value
        self.current_time = time.time() if current_time is None else current_time
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if delta_time < self.sample_time:
            return self.output

        self.PTerm = self.Kp * error
        self.ITerm += error * delta_time
        self.ITerm = min(max(self.ITerm, -self.windup_guard), self.windup_guard)
        self.DTerm = self.Kd * delta_error / delta_time if delta_time > 0 else 0.0

        self.last_time = self.current_time
        self.last_error = error
        self.output = self.PTerm + self.Ki * self.ITerm + self.DTerm
        return self.output


def set_range(x, x_min, x_max):
    """x clamped to [x_min, x_max] (sdk.common.set_range)."""
    return min(max(x, x_min), x_max)


def val_map(x, in_min, in_max, out_min, out_max):
    """Linear remap of x from one range to another (sdk.common.val_map)."""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
