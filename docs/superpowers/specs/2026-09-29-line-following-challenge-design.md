# Line Following challenge ("tune the follower until it laps the loop in time")

Date: 2026-09-29
Status: implemented; values measured in the sim (see "Measured")
Chosen by: Claude, on the user's instruction to pick the remaining workshop exercises overnight;
the user reviews it in the morning. Same pattern as the other exercises.

## Purpose

Workshop topic 6.9 "Autonomous Line Following" has 10 minutes. The sim already has the loop
(`worlds/line_track.sdf`, `rospider_gazebo/line_track.py`) and upstream's follower
(`line_following.py`: click the line's colour, three image strips, PID on the deflection
angle). Participants tune the follower's numbers in one file until the robot laps the loop
without leaving the line and in time. One lap with upstream's values takes about 145 s, so a
check fits the slot.

What they should take away: the steering gain decides whether the robot follows a bend at
all, and speed trades lap time against control.

## The follower's numbers

`LineControl` (`rospider_gazebo/line_follower.py`) took upstream's constants; it now takes
`kp`, `speed` and `max_turn` (upstream's 1.1, 0.05 m/s and 0.35 rad/s are the defaults), and
`line_following.py` reads them as parameters, which `vision_demo.launch.py` passes through.
Nothing changes for the demo.

## The participant's file: `config/line_challenge.yaml`

`/**: ros__parameters:` with four keys (the three above and `threshold`, the colour band
width the node already had), validated like the other exercises (`challenge_params.merge`:
unknown keys, blanks and wrong types stop the launch with a Thai message).
`line_challenge.launch.py` passes them to `line_following.launch.py` (loop world, robot on the
line, the `image` window). `config/line_challenge_solved.yaml` = upstream's values.

## The checker: `check_line.py` / `rospider_gazebo/line_check.py`

Picks the line's colour through the node's own `~/set_target_color` (the point where the line
runs in the picture at the spawn pose, 0.45 / 0.82 of the frame), switches following on, and
feeds odometry (the sim's odometry is the world pose) to `LapTracker`: the nearest point of the
loop, the signed progress along it, the distance from the line. The run ends when the lap is
done, the robot is more than 0.2 m off the line, it gains less than 1% of the loop in 20 s, or
after 1.6 x the lap limit; then the robot is stopped (`~/set_running false`, also on Ctrl+C). It
refuses to start unless the robot is within 0.15 m of the spawn point (relaunch first).

| Level | Passes when |
|---|---|
| 1 Following (1/4 lap) | a quarter of the loop covered on the line |
| 2 The whole lap | the whole loop covered on the line |
| 3 In time | the lap in at most `LAP_LIMIT` sim seconds |

Hints name what the robot did (left the line at N% / stopped at N% / slow), never a key.

## Measured

Fresh launch per run, `check_line.py`, sim seconds:

| File | Result |
|---|---|
| answer key (upstream's values) | 145 s pass; with `threshold` 0.1 / 0.05: 145 s / 144 s pass |
| starting file (`kp` 0.3, `speed` 0.03) | stopped at 10% of the lap: level 1 (2 runs) |
| `kp` 0.3 only | stopped at 6%: level 1 (2 runs) |
| `speed` 0.03 only | lap in 237 s: level 3 |
| `max_turn` 0.1 only | lap in 148 s: pass |
| `max_turn` 0.05 / 0.03 only | stopped at 8% / 6%: level 1 |

No single value fails only level 2 (the first bend comes at ~6% of the loop, so any steering
shortfall shows at level 1), so the file has two mistakes (`kp` -> level 1, `speed` -> level 3)
and ships `max_turn` 0.35 and `threshold` 0.5 right; level 2 is a gate. `LAP_LIMIT` 200 s sits
55 s above the answer key and 37 s below the slow run.

## As built: fixes from the final review (2026-09-29)

- A run that is still on the line when its time runs out gets the "slow" hint at level 1 or 2
  (it had the "turn harder" hint, pointing at the right value).
- `check_line.py` gives up when no new `/odom` arrives for 10 wall seconds or after
  5 x `RUN_LIMIT` wall seconds (a closed or paused sim no longer hangs it), and resends
  `~/set_target_color` / `~/set_running` when a reply is lost.
- The challenge launches the follower with `start: false`: a click in the window only picks a
  colour, and the robot waits for the checker, so every check starts on the spawn point.

Deferred minors: a lap driven the wrong way round is credited; negative / NaN values are not
refused; the checker cannot tell whether the plain demo (upstream values) is running instead of
the challenge; the start check is position-only (a yawed robot picks floor); a Ctrl+C before the
node exists tracebacks; `nav_msgs` is not declared in package.xml.
