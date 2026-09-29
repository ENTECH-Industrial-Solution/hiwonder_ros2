# 3D Shape Recognition challenge ("find the ball with the depth camera")

Date: 2026-09-29
Status: implemented; values measured in the sim
Chosen by: Claude, on the user's instruction to build the remaining workshop exercises ("ทำส่วนที่เหลือ")
after they had played the first five. Same pattern as the other exercises.

## Purpose

Workshop topic 8.8 "3D Vision: Shape Recognition" has 10 minutes. The sim's `object_classification`
window is upstream's depth recognition: everything nearer than the floor plane inside a region of
interest (ROI) is an object, named by its outline's corners and its depth spread (sphere / cuboid /
cylinder), and the nearest object of a wanted shape is the target. Participants fix its three values
so the robot picks out the ball. One check takes a few seconds; the robot never moves.

What they should take away: a depth camera separates objects from the floor only when it knows how
far the floor is; the ROI must hold the objects but not the robot's own legs; the shape filter is
what turns "everything seen" into "the thing I want".

## Scene: `worlds/shape_challenge.sdf`

Bare floor, sun, three visual-only 3 cm solids in the view of object_classification's look pose
(camera ~0.28 m above the floor, looking almost straight down): a blue cylinder straight below, a green
cuboid beside it, and the red sphere at the top edge of the picture. 3 cm on purpose: the floor then
lies within 40 mm of the nearest top (upstream keeps only what is within 40 mm of the nearest
surface), so a floor distance set too far turns the whole floor into one object. The cylinder sits
where the camera looks straight down at it: seen at an angle its side shows, its depth spread passes
2 mm and it is named a sphere.

## Node change

`object_classification.py` publishes each frame's result on `~/objects` (std_msgs/String, JSON):
`plane_distance`, `roi`, `shapes`, `near` (nearest depth in the ROI), `median` (ROI median depth),
`objects` (kind, box, depth) and `target` (index or null). `vision_demo.launch.py` gained
`params_file`, an extra parameters file for list values.

## The participant's file: `config/shape_challenge.yaml`

`/**: ros__parameters:` with three keys, validated by `challenge_params.merge` (now also lists,
element by element; an empty list is refused because rclpy cannot declare it) and
`shape_check.validate` (ROI inside 640x480 with min < max, known shape names).

| Key | Starting | Solved | Measured symptom |
|---|---|---|---|
| `plane_distance` | 350 (upstream's) | 280 (275-280 pass) | the whole floor is one object (level 1) |
| `roi` | [120, 350, 150, 500] | [0, 350, 150, 500] | the sphere is not seen (level 2) |
| `shapes` | all three | ['sphere'] | the target is the cylinder (level 3) |

## The checker: `check_shape.py` / `rospider_gazebo/shape_check.py`

Waits for reports and `/odom`, refuses unless the robot is at the spawn (within 5 cm and 5 degrees:
the expected boxes are pixels of that view), takes the report 3 s after the first (the arm has
reached its look pose), prints the settings in use, and scores it. A recognised object is one of
`EXPECTED` (the answer key's boxes) only when its whole box lies inside that box + 20 px; a box
covering half the ROI is the floor.

| Level | Passes when |
|---|---|
| 1 Objects apart from the floor | at least one expected object found and no floor blob |
| 2 All three, named right | every expected object found with its own kind, nothing else seen |
| 3 The ball chosen | the target is the object matched to the sphere |

Hints describe what the robot saw: the floor as one object (with the ROI's median depth), nothing
at all (with the nearest and median depth), only something that is not an object (the legs), a
missing object and where it is in the picture, a misnamed one, extra objects (a floor strip or a
leg), and which object was chosen.

## Measured

Fresh launch per run, `check_shape.py` (the scene is static, results are identical run to run):

| File | Result |
|---|---|
| starting file | the floor as one object: level 1 |
| `plane_distance` 280 | sphere not seen: level 2 |
| + `roi` from row 0 | cylinder chosen: level 3 |
| answer key | pass (twice) |
| answer key with `plane_distance` 275 | pass |
| 270 | sphere named cylinder: level 2 |
| 285 | a floor strip at the bottom of the ROI: level 2 |
| 290 | the floor as one object: level 1 |
| 240 | nothing seen: level 1 |
| ROI the whole frame | only a leg (150 mm) counts: level 1 |

The passing band for `plane_distance` is narrow (275-284 mm) because the camera is tipped a little
and the floor lies 274-285 mm across the ROI; the level-1 hint gives the ROI's median depth
(277-280), which lands in it.
