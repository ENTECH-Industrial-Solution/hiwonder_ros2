# Color Tracking challenge ("walk up to the red ball, not the orange one")

Date: 2026-09-29
Status: implemented; values measured in the sim
Chosen by: Claude, on the user's instruction to build the remaining workshop exercises ("ทำส่วนที่เหลือ")
after they had played the first five. Same pattern as the other exercises.

## Purpose

Workshop topic 6.6 "Color Tracking" has 10 minutes. The sim's `object_tracking` window is the phone
app's node: a click samples a colour, a LAB band round it (`threshold`) finds the biggest patch, and
two PIDs walk the robot until the patch sits on the yellow stop dot. Participants fix three of four
values so the robot walks to the right ball, stops where it should, and does it in time. One check
takes about 30 s.

What they should take away: the band width decides what counts as "the same colour"; image-based
following stops when the target reaches a reference pixel (lower in the picture = closer); the speed
cap bounds the approach time.

## Scene: `worlds/track_challenge.sdf`

Bare floor and sun (the room's posters carry every colour). A red ball (r 0.04) at (1.3, 0.45),
1.4 m ahead-left, and a bigger, nearer orange ball (r 0.05) at (0.9, -0.3). Measured LAB (OpenCV):
red (64, 167, 152), orange (97, 137, 166), the gripper fingers at the bottom of the picture dark
neutral grey. Upstream's `threshold` 0.5 takes in the red ball only; 0.6-0.78 also the orange one
(the bigger patch, so it is chosen); from ~0.8 the gripper.

## Node changes

`FollowControl` takes `max_speed` and `max_turn` (upstream's 0.05 m/s and 0.1 rad/s; the final
approach keeps upstream's 0.01 cap, or `max_speed` when that is lower). `object_tracking.py` reads
them and `stop_y` (the stop dot's row, upstream's 300) as parameters; `vision_demo.launch.py` passes
them through.

## The participant's file: `config/track_challenge.yaml`

`/**: ros__parameters:` with four keys, validated by `challenge_params.merge`, passed by
`track_challenge.launch.py` to `object_tracking.launch.py` with `start: false` (a click only picks a
colour; the robot waits for the checker).

| Key | Starting | Solved | Measured symptom |
|---|---|---|---|
| `threshold` | 0.7 | 0.5 | the robot turns to the orange ball (level 1) |
| `stop_y` | 200 | 400 (370-420 pass) | stops 1.01 m from the red ball (level 2) |
| `max_speed` | 0.01 | 0.05 | the right spot after 91 s (level 3) |
| `max_turn` | 0.1 | (right) | 0.02 / 0.01 still reached the ball, in 23 / 53 s |

## The checker: `check_track.py` / `rospider_gazebo/track_check.py`

Refuses unless the robot is within 0.1 m and 5 degrees of the spawn (the ball's pixel is only right
from there); sends `~/set_target_color` at the red ball's pixel (143, 128), waits 3 s of sim time
for the picker's 10 frames, switches following on (resent if the reply is lost) and feeds odometry
to `tag_check.TagRun` scored against the two balls (TagRun now takes `targets`/`target` and also
reports `closer` and `turned`), until the robot has moved and stood still 5 s, never moved in 20 s,
or 120 s passed; then switches it off. Gives up after 10 wall seconds without `/odom`.

| Level | Passes when |
|---|---|
| 1 To the red ball | the ball the robot ends up facing is the red one, and it is >= 0.2 m closer to it |
| 2 Stopped in front | stopped 0.43-0.52 m from the red ball, facing it within 15 degrees |
| 3 In time | stopped within 40 s (answer key 21 s) |

Level-1 hints: did nothing (no walk, turned < 10 degrees: at the spawn the balls sit at nearly the
same bearing, so the facing alone says nothing), turned or walked to the orange ball, or did not get
closer (the gripper case: the robot backs off).

## Measured

Fresh launch per run, sim seconds:

| File | Result |
|---|---|
| starting file | turns to the orange ball: level 1 (twice) |
| `threshold` fixed | stops 1.01 m away: level 2 |
| `stop_y` fixed too | right spot after 91 s: level 3 |
| answer key | 21 s: pass (twice) |
| `threshold` 1.0 | backs off (-0.06 m): level 1, gripper hint |
| `stop_y` 300 / 350 / 380 / 400 / 420 / 440 | 0.64 / 0.54 / 0.49 / 0.46 / 0.44 / 0.42 m, all ~21 s |
