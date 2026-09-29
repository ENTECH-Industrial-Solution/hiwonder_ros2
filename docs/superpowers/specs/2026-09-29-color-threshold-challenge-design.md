# Color Threshold challenge ("tune the LAB bands until the checker sees only the cubes")

Date: 2026-09-29
Status: implemented; values measured in the sim
Chosen by: Claude, on the user's instruction to pick the remaining workshop exercises overnight
("โจทย์นายเลือกเองก่อนเลย"); the user reviews it in the morning.
Follows the pattern of the SLAM / Nav2 / V-SLAM exercises (wrong values in a file, an automatic
checker scoring in three levels, hints that name symptoms, not parameters).

## Purpose

Workshop topic 6.1 "Color Threshold Adjustment" has 10 minutes. Participants use the sim's
LAB_Tool window (`lab_tool.launch.py`, the same layout as Hiwonder's LAB_Tool) to fix three LAB
bands, then run one command that scores what the camera sees. No driving: the check takes one
camera frame, so it fits the time.

What they should take away: a band that is too narrow finds nothing, a band that is too wide
catches look-alikes (orange for red), and the lightness range decides whether an object in shadow
is still found.

## Scene: `worlds/color_challenge.sdf`

A grey floor, no walls (posters would carry every colour), the sun as in the room world. The
robot spawns at the origin with the arm in its `init` pose, so the camera looks at the floor
0.25-0.5 m ahead. Visual-only 6 cm cubes:

| Object | Position (x, y) | Role |
|---|---|---|
| red cube | 0.35, 0.12 | target |
| green cube | 0.45, 0.06 | target |
| blue cube | 0.28, 0.00 | target (lit) |
| blue cube | 0.45, -0.06 | target in the shadow of a grey block at (0.53, -0.075) |
| orange cube | 0.35, -0.12 | decoy for red |

Measured LAB (OpenCV 8-bit, after the detector's 3x3 blur): red A 171-191 B 155-168; orange
A 137-143 B 166-190; green A 67-90 B 157-176; blue lit L 106 A 167 B 53, blue in shadow L 50
A 152 B 82; floor, block and gripper A = B = 128.

## The participant's file: `config/color_challenge.yaml`

`color_detect`'s own shape (`color_detect: ros__parameters:` with `min`/`max` per colour), so
LAB_Tool opens it directly (`config:=`), and its Save writes `~/.ros/color_challenge_tuned.json`,
which overlays the file exactly as `color_detect` does. Solved = `config/color_detect.yaml`'s bands.

| Colour | Starting band | Solved band | Symptom |
|---|---|---|---|
| red | A min 135 | A min 150 | the orange cube is caught too (level 2) |
| green | A max 60 | A max 110 | the green cube is not found (level 1) |
| blue | L min 80 | L min 0 | the blue cube in shadow is not found (level 3) |

## The checker: `check_color.py` / `rospider_gazebo/color_check.py`

Takes the 8th camera frame (the first frames of a new subscriber are black), reads the bands the
way `color_detect` does (file, then the tuned JSON), and runs `color_detect`'s own mask pipeline
(the shared `find_blobs`). Each target has a pixel box measured from the answer-key frame, with a
margin.

| Level | Passes when |
|---|---|
| 1 Every colour found | red, green and the lit blue cube each have a blob of their colour inside their box |
| 2 No look-alikes | no blob of any colour lies outside that colour's boxes |
| 3 Also in shadow | the blue cube in shadow has a blue blob |

Hints: level 1 "the mask shows nothing on the cube — is the band too narrow?"; level 2 "the mask
of a colour also lights up another object — which one? the band is too wide"; level 3 "the cube in
shadow is darker: does the band reach dark enough?". A frame that is black or taken with the
robot moved is reported as such.

## Launch: `color_challenge.launch.py`

`gazebo.launch.py world:=color_challenge` plus, after the sim is up, `color_detect` with
`tune:=true` on the participant's file and `tuned_path` `~/.ros/color_challenge_tuned.json`. To
start over: `git checkout` the file and delete the JSON (the launch prints both).

## Testing

`test/test_color_check.py` scores a frame recorded from the sim (`test/data/color_challenge.png`)
with the starting, partly fixed and solved bands, and synthetic frames (black, empty floor);
hints name no parameter. In the sim: the answer key passes and the starting file fails level 1
with the live checker.

## As built: fixes from the final review (2026-09-29)

- A target is found only when a blob's whole bounding rect lies inside its box; the centroid test
  let a band that whitened 82% of the frame pass every level. A blob covering the box is reported
  as "too wide" at level 1, not "too narrow".
- `color_detect` publishes its live settings (latched `/color_detect/settings`, JSON) at every
  change; `check_color.py` scores those, so slider moves not yet saved are what gets checked,
  and falls back to the files with a note when no window is open.
- The frame is refused unless the answer-key bands pass on it (robot or arm moved, other world,
  other camera size), instead of being scored as a band mistake.
- Colours other than red, green and blue (added in the window) are ignored and named; a scored
  colour missing from the Color list is named as such.

Deferred minors: model names `blue_target`/`blue_shade` in the SDF are swapped relative to their
roles; a broken tuned JSON makes the checker refuse where color_detect ignores it; `params:=` with a
custom path prints a check command without `--params`; no "empty floor" synthetic test.
