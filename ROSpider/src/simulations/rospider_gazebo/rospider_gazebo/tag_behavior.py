"""What the robot does about the tags it can see, without ROS.

scripts/apriltag_detect.py calls update() once per camera frame with the
tags solved in that frame and publishes whatever comes back. Keeping the
rules here means the sign conventions, the deadbands and the release
sequence are pinned by test/test_tag_behavior.py instead of by driving the
simulated robot into a wall.

Sightings are (tag_id, tvec) with tvec in the camera's optical frame: x
right, y down, z forward, metres. So "the tag is to the right" is x > 0 and
the robot must turn right, which is a NEGATIVE angular.z in ROS.

Twist output is (linear_x, angular_z); None means "publish nothing", which
is how teleop or Nav2 gets /controller/cmd_vel back once no tag is in view.
"""

import math
from dataclasses import dataclass

from rospider_gazebo import tag_settings


@dataclass
class Decision:
    twist: tuple | None     # (linear_x, angular_z), or None to stay silent
    place: int | None       # tag id whose station ~/place must target now
    status: str             # one line for the GUI and the log


def _clamp(value, limit):
    return max(-limit, min(limit, value))


class TagBehavior:

    def __init__(self, control, behaviors, enabled=False):
        self.enabled = enabled
        self.configure(control, behaviors)
        self._reset()

    def configure(self, control=None, behaviors=None):
        """Replace the gains and/or the per-tag rows; keeps the current
        target so a slider tweak mid-approach does not restart it."""
        if control is not None:
            self.control = dict(control)
        if behaviors is not None:
            self.behaviors = {tag_settings.tag_id(key): dict(row)
                              for key, row in behaviors.items()}

    def action(self, tag_id):
        row = self.behaviors.get(int(tag_id))
        return row['action'] if row else 'none'

    def _reset(self):
        self.target = None      # tag id being acted on
        self.phase = 'idle'     # idle | tracking | reached
        self.last_seen = None   # `now` of the last frame the target was in
        self.released = True    # the post-loss zero twist has been sent
        # Tags that already fired ~/place, each with the `now` it was last
        # seen. Tracked per id rather than on self.target/self.phase, since
        # a nearer or a `stop` tag can steal the scalar target for a frame
        # while the placed tag never actually left view.
        self.placed = {}

    def update(self, now, sightings):
        if not self.enabled:
            # Switched off mid-drive: one zero so the robot actually stops,
            # then silence. Everything else forgets the target so switching
            # back on starts clean.
            self._reset_keeping_release()
            if not self.released:
                self.released = True
                return Decision((0.0, 0.0), None, 'behaviours disabled')
            return Decision(None, None, 'behaviours disabled')

        # Keep last_seen fresh for every placed tag still in view, then drop
        # the ones that have been gone longer than lost_timeout -- that is
        # the "left view" signal that lets a placed tag be placed again.
        seen_now = {tag_id for tag_id, _tvec in sightings}
        for tag_id in seen_now & self.placed.keys():
            self.placed[tag_id] = now
        self.placed = {tag_id: last_seen
                        for tag_id, last_seen in self.placed.items()
                        if now - last_seen < self.control['lost_timeout']}

        candidates = [(tag_id, tvec) for tag_id, tvec in sightings
                      if self.action(tag_id) != 'none']
        stops = [c for c in candidates if self.action(c[0]) == 'stop']
        pool = stops or candidates
        if not pool:
            return self._lost(now)
        # Planar range, not raw z: a remembered tag can sit behind the
        # camera (z < 0), and raw z would let it beat a nearer, visible
        # tag just because a negative number compares less than a
        # positive one.
        tag_id, tvec = min(pool,
                           key=lambda c: math.hypot(float(c[1][0]),
                                                    float(c[1][2])))
        x, z = float(tvec[0]), float(tvec[2])

        if tag_id != self.target:
            self.target, self.phase = tag_id, 'tracking'
        self.last_seen = now
        self.released = False
        action = self.action(tag_id)

        if action == 'stop':
            self.phase = 'reached'
            return Decision((0.0, 0.0), None, f'tag {tag_id}: stop')
        if tag_id in self.placed:
            # The arm is (or was) moving: hold still, and do not call
            # ~/place again until the tag has left and come back. This is
            # keyed on the tag id, not self.target/self.phase, so a tag that
            # briefly loses the scalar target to a nearer tag (or a `stop`
            # tag, which always wins the pool) still holds instead of
            # re-firing when it becomes nearest again.
            return Decision((0.0, 0.0), None,
                            f'tag {tag_id}: placed; hold until it leaves view')

        c = self.control
        standoff = float(self.behaviors[tag_id]['standoff'])
        bearing = math.atan2(x, z)
        if abs(bearing) > c['turn_first_rad']:
            # Far off-axis, or behind the camera (a remembered tag can be):
            # face it before driving. A P law on x alone would happily back
            # the robot through a station that is behind it.
            self.phase = 'tracking'
            angular = -math.copysign(c['max_angular'], bearing)
            return Decision((0.0, angular), None,
                            f'tag {tag_id}: turning toward it '
                            f'(bearing {math.degrees(bearing):+.0f} deg)')
        range_error = z - standoff
        angular = (0.0 if abs(x) < c['yaw_deadband']
                   else _clamp(-c['kp_yaw'] * x, c['max_angular']))
        linear = (0.0 if abs(range_error) < c['dist_deadband']
                  else _clamp(c['kp_dist'] * range_error, c['max_linear']))

        if angular == 0.0 and linear == 0.0:
            arrived = self.phase != 'reached'
            self.phase = 'reached'
            if action == 'place' and arrived:
                self.placed[tag_id] = now
                return Decision((0.0, 0.0), tag_id,
                                f'tag {tag_id}: reached; placing')
            return Decision((0.0, 0.0), None,
                            f'tag {tag_id}: reached (z {z:.2f} m)')

        self.phase = 'tracking'
        return Decision((linear, angular), None,
                        f'tag {tag_id}: {action} z {z:.2f} x {x:+.2f}')

    def _reset_keeping_release(self):
        released = self.released
        self._reset()
        self.released = released

    def _lost(self, now):
        """No actionable tag this frame.

        Hold zero through lost_timeout (a tag flickering out for a frame
        must not hand cmd_vel to someone else), then publish zero exactly
        once more and go silent.
        """
        if self.target is not None and self.last_seen is not None \
                and now - self.last_seen < self.control['lost_timeout']:
            return Decision((0.0, 0.0), None,
                            f'tag {self.target} lost; holding')
        self._reset_keeping_release()
        if not self.released:
            self.released = True
            return Decision((0.0, 0.0), None, 'tag lost; cmd_vel released')
        return Decision(None, None, 'no tag')
