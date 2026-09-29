# 3D Object Grasping challenge ("fetch the blue block and put it on the yellow pad")

Date: 2026-09-29
Status: implemented; values measured in the sim
Chosen by: Claude, on the user's instruction to build the remaining workshop exercises ("ทำส่วนที่เหลือ")
after they had played the first five. Same pattern as the other exercises.

## Purpose

Workshop topic 8.7 "3D Vision: Object Grasping". The sim's `track_and_grab` window finds a coloured
block with the depth camera, can walk the body up to it (`walk`: the block's pixel and depth put
through TF into `base_footprint`, `approach.py` stops with the block `target_x` ahead), hands the
grab to `pick_and_place`, and with `auto_place` puts the block down at `place_point` ("x y z" in
`base_footprint`). Participants fix three values so the robot fetches the blue block from a pedestal
off to its side and puts it on a pad. One check takes about 1.5 minutes of wall time.

What they should take away: the arm only reaches so far, so the body must walk and stop at the
distance the arm is calibrated for; a place point is a 3D point in the robot's own frame (x ahead,
y left, z up).

## Scene

`track_and_grab.launch.py layout:=spread` (green block in front, red and blue each on a pedestal
0.8 m to either side) plus a visual-only 0.12 m yellow pad on the floor at (-0.23, -0.58), where the
answer key puts the blue block down: the robot walks to (-0.07, -0.52) facing -75 degrees, and
`place_point` (0.02, -0.16, 0.035) lands there (measured -0.234/-0.584 and -0.226/-0.575).

## The participant's file: `config/grasp_challenge.yaml`

`/**: ros__parameters:` with three keys, validated by `challenge_params.merge` and
`grasp_check.parse_point` (three numbers), passed to `track_and_grab.launch.py` as launch arguments.

| Key | Starting | Solved | Measured symptom |
|---|---|---|---|
| `walk` | false | true | the robot never sets off: the block is out of reach and out of view (level 1) |
| `target_x` | 0.40 | 0.235 (0.20-0.27 pass) | walks up, stops 0.46 m off and never picks (level 2); 0.30: the grasp misses |
| `place_point` | '0.02 0.16 0.035' | '0.02 -0.16 0.035' | the block lands on the robot's left, 0.35 m from the pad (level 3) |

## The checker: `check_grasp.py` / `rospider_gazebo/grasp_check.py`

Waits for track_and_grab, pick_and_place's `~/state` and `/odom`; refuses unless the robot is within
0.1 m of the spawn and the blue block within 2 cm of its spawn (`gz model -p`); orders `~/pick blue`
(the window's blue button, resent if the reply is lost -- a second order while busy is refused
harmlessly); follows the states and the odometry until the block has been put down (CARRY, then
IDLE/DONE), no pick has started 120 s after the order or the robot has stood still (no step, no
3 degree turn) 20 s without one, or no new pick state for 60 s; switches the job off, waits 2 s and
reads where the block came to rest. Gives up after 10 wall seconds without `/odom`.

| Level | Passes when |
|---|---|
| 1 Walked up to the blue block | the robot ends within 0.6 m of the block, facing it within 20 degrees |
| 2 Picked it up | pick_and_place reached CARRY |
| 3 On the pad | the block rests on the floor within 5 cm of the pad's centre |

Hints: did not set off (the block is beyond the arm from where it stands); stopped somewhere else;
walked up but never picked (how far does the arm reach, how far off does the robot stop); tried and
missed (the arm looks for the block at the distance it is used to); the block's distance from the
pad and on which side of the robot pad and block are (x ahead, y left).

## Measured

Fresh launch per run:

| File | Result |
|---|---|
| starting file | never sets off: level 1 |
| `walk` fixed | stops 0.46 m off, no pick: level 2 (twice) |
| `target_x` fixed too | block on the robot's left, 0.35 m from the pad: level 3 |
| answer key | pass (three times; ~52 s sim, ~106 s wall per check) |
| `target_x` 0.20 / 0.27 | pass |
| `target_x` 0.30 | the grasp misses: level 2 |
