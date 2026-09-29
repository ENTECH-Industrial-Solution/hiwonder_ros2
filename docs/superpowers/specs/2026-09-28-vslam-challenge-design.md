# V-SLAM challenges ("map the room with the camera", then "navigate on it")

Date: 2026-09-28
Status: implemented; values measured with the shipped files (see "As built" at the end)
Follows: `2026-09-28-slam-challenge-design.md` and `2026-09-28-nav-challenge-design.md`
(same room, same pattern)

## Purpose

Two workshop exercises for the RTAB-VSLAM slot, the camera-only counterparts
of the SLAM and Nav2 exercises:

1. **Mapping.** Participants map the exercise room with RTAB-Map using only
   the RGB-D camera. Three values in one YAML file are deliberately wrong; an
   automatic checker scores the saved map in three levels.
2. **Navigation.** Participants navigate the same course as the Nav2
   exercise on *their own* V-SLAM map. Two values are wrong; the Nav2
   exercise's checker scores the run.

What they should take away, each measured in the sim: the camera only maps
what it looks at, the grid only reaches as far as the depth range allows,
and a loose loop-closure check makes the robot "recognise" a place it has
never been, which bends the map.

Constraints, as before: participants edit YAML only; checking is automatic;
hints describe the symptom, never the parameter name; Thai learner text.

## Findings that shape the design (measured)

- **Repeated wall texture broke loop closure.** In `worlds/slam_challenge.sdf`
  seven inner walls show the same `partition.png`, on both faces. With
  Hiwonder's values RTAB-Map accepted 4-5 wrong loop closures per run (1.2 m
  off: room A matched onto the corridor), and the map came out bent (76-84%
  coverage against the LiDAR map). Parameters could not fix it reliably:
  `RGBD/OptimizeMaxError` 1.0 gave 0 wrong closures once and 4 the next time.
  With a distinct texture on every face, Hiwonder's values gave 0 wrong
  closures in both runs (13 and 6 correct), 99% coverage, straight walls.
- **`map_saver_cli` sometimes saves a scrap.** In two of four mapping runs
  it saved a ~3 x 2.5 m grid although the robot had covered the room; both
  were in the repeated-texture room, not seen since. `rtabmap-export --map`
  rebuilds the full grid from the database every time.
- **Localization rewrites the map.** On shutdown, `rtabmap` in localization
  mode writes its 2D grid back into the `.db`. That grid carries ghost walls
  (camera poses lag the arm's TF; localization jumps), and every later run
  loads it as the map: the global costmap then had a phantom wall below
  room A and lines across the corridor, and the answer key failed G2 with
  "no path" in 3 of 4 runs. Running localization on a temporary copy of the
  `.db`, with `map_always_update` off, the answer key passed 2 of 2
  (159 s, 176 s).
- **Relocalization from another start pose failed.** Spawned 0.8 m from the
  mapping start, RTAB-Map applied the last pose saved in the database (the
  start) and never corrected it (its best visual candidate had 14 features,
  under the minimum of 20). Both exercises therefore start at the spawn
  point used for mapping; relocalization is out of scope.
- **Lost lifecycle replies hit Nav2 too.** In ~1 of 12 launches
  `smoother_server` "failed to send response to change_state" and
  `lifecycle_manager_navigation` waited forever. A lost goal response was
  also seen once ("Failed to send goal response (timeout)").

## Room change: `worlds/slam_challenge.sdf`

Every inner wall (partition north/south, stub, maze 1-4) keeps its one
collision box; its visual is split into two 0.025 m boxes, one per face, each
with its own poster texture (14 new files, `worlds/textures/face_*.png`,
drawn by a new `tools/make_face_textures.py` with fixed seeds, in the style
of the existing posters: shapes plus a label). LiDAR ignores visuals, so the
room's geometry, `maps/slam_challenge.*` and both existing exercises are
unchanged. The file's header comment records why.

## Exercise 1: mapping

### What the participant does

1. `ros2 launch rospider_gazebo vslam_challenge.launch.py map:=<name>`
   starts the exercise room and RTAB-Map (camera only, new map at
   `maps/vslam/<name>.db`), with RViz.
2. Edit `config/vslam_challenge.yaml` in place and relaunch.
3. Drive with teleop through both rooms and back to the start (the loop
   closure needs a revisit), then close the launch (Ctrl+C): RTAB-Map writes
   the database on shutdown.
4. `ros2 run rospider_gazebo check_vslam.py <name>` scores the map.

### The participant's config: `config/vslam_challenge.yaml`

Same shape as the other exercises (`/**: ros__parameters:`), each key with a
Thai comment. RTAB-Map keys keep their real names (the workshop teaches
them); `camera_view` is a friendly key.

| Key | Starting | Solved | Symptom (measured, answer key otherwise) |
|---|---|---|---|
| `camera_view` | `floor` | `ahead` | walls mostly missing: 34% of the reference's walls found, coverage still 94% |
| `Grid/RangeMax` | `"0.5"` | `"5.0"` | almost no map: 0.5 m² known (and no `/map` at all) |
| `Vis/MinInliers` | `"5"` | `"20"` | wrong loop closures (1-2 per run in 3 runs), walls 1.6-2.2x as thick |
| `Grid/CellSize` | `"0.05"` | (right) | — |
| `Rtabmap/DetectionRate` | `"1"` | (right) | — |

Solved = Hiwonder's values (`config/vslam.yaml`); the starting file's
combination of all three mistakes is measured during implementation.

`rospider_gazebo/vslam_params.py` (tested) merges the file: `camera_view`
becomes the `arm_pose` launch argument (`floor` → `init`, `ahead` →
`horizontal`); every other key must already exist under `rtabmap:` in
`config/vslam.yaml` and is written there as a string (RTAB-Map declares all
its parameters as strings; a number typed without quotes is accepted and
converted). Unknown keys, blank values and a `camera_view` other than the
two words stop the launch with a Thai message naming the key (reusing
`challenge_params`' loader and error type). `config/vslam_challenge_solved.yaml`
holds the answer key.

### The checker: `check_vslam.py` / `rospider_gazebo/vslam_check.py`

1. Finds `<name>` like `navigation.launch.py`'s `map:=` does, in the
   `maps/vslam/` subfolder (`.db`).
2. Refuses to read a database that a running `rtabmap` still has open
   (checked in `/proc`) and says to close the launch first.
3. Runs `rtabmap-export --map` into a temporary folder for the 2D grid, and
   reads the `Node` and `Link` tables with `sqlite3` for loop closures.
4. Scores against `maps/vslam_challenge.{pgm,yaml}`: the grid exported from
   the answer-key database (small, committed; the `.db` itself is ~135 MB
   and git-ignored).

| Level | Passes when | Starting mistake it catches |
|---|---|---|
| 1 A map | known area ≥ 1 m² | `Grid/RangeMax` (0.5 m²) |
| 2 The room and its walls | ≥ 90% of the reference's free cells known, and ≥ 60% of its wall cells matched by an occupied cell within 2 cells | `camera_view` (34%; answer key 100%) |
| 3 A straight map | ≥ 1 loop closure, none wrong, and occupied area ≤ 1.2x the reference's | `Vis/MinInliers` (1-2 wrong, 1.6-2.2x) |

A **wrong loop closure** is a global or local closure link (types 1, 2)
whose transform differs from the odometry between its two nodes by more
than 0.1 m or 5°. This uses the fact that sim odometry is exact (the node
poses in the database are odometry poses); the module docstring says so.
The wall threshold is 60%, not higher, because the loose-closure mistake
already drops it to 75-77% and must fail level 3, not level 2. As in the
SLAM checker, a level that cannot be judged yet says which level to pass
first, and coverage/area helpers are reused from `slam_check.py`.

Hints (symptoms only): level 1 "the map only shows what is right next to
the robot"; level 2 "RViz shows the floor but hardly any walls — what is the
camera looking at?"; level 3, no closures: "did you drive back to where you
started?"; wrong closures: "the robot thought it was somewhere it had not
been — walls are doubled or bent".

## Exercise 2: navigation on the participant's map

### What the participant does

1. `ros2 launch rospider_gazebo vslam_nav_challenge.launch.py map:=<name>`
   loads `maps/vslam/<name>.db` from exercise 1 (a clear Thai error if it
   does not exist, pointing to exercise 1). Instructors make the answer-key
   map once with `vslam_challenge.launch.py params:=<solved file>` and the
   route in the docs.
2. Edit `config/vslam_nav_challenge.yaml` in place and relaunch.
3. `ros2 run rospider_gazebo check_nav.py`: the Nav2 exercise's checker and
   course, unchanged except for the resend fix below.

### The participant's config: `config/vslam_nav_challenge.yaml`

| Key | Starting | Solved | Symptom (measured) |
|---|---|---|---|
| `camera_view` | `floor` | `ahead` | G1 aborts after ~58 s, the controller sees obstacles everywhere (2 runs, one on a clean map) |
| `max_vel_x` | `0.05` | `0.15` | course 379 s, over the 300 s limit |
| `robot_radius` | `0.3` | `0.15` | to measure on the V-SLAM map (its walls are thinner than the LiDAR map's); replaced if it does not block the doorway |
| `xy_goal_tolerance` | `0.05` | (right) | — |
| `inflation_radius` | `0.3` | (right) | — |
| `max_vel_theta` | `0.25` | (right) | — |

Answer key: 156-176 s over three runs. `camera_view` is handled as in
exercise 1; the Nav2 keys go through `nav_params.apply` onto
`config/vslam_nav2_params.yaml` after `nav_params.challenge_base` (RPP,
cost scaling, smoother acceleration), exactly as the Nav2 exercise does.

### Launch changes (in `vslam.launch.py`'s localization mode, so plain use benefits too)

- The database is copied to a temporary file and RTAB-Map localizes on the
  copy; the participant's `.db` is never modified.
- `map_always_update` is false in localization mode (it stays true for
  mapping, where `map_saver_cli` needs it).
- Nav2 is started with `autostart: false` and brought up by
  `scripts/lifecycle_activate.py`, extended to take several node names and
  bring them up in order (the list in `nav2_bringup/navigation_launch.py`).

### `check_nav.py` fix (shared with the Nav2 exercise)

When Nav2 accepts a goal but its response is lost, the checker waited 10 s
and reported `rejected` although the robot might already be driving. It now
resends the goal (Nav2 replaces the first) up to three times before
reporting `rejected`. Same for the planner request.

## Files

Added under `ROSpider/src/simulations/rospider_gazebo/`:

| File | Role |
|---|---|
| `tools/make_face_textures.py`, `worlds/textures/face_*.png` | per-face wall posters |
| `config/vslam_challenge.yaml`, `config/vslam_challenge_solved.yaml` | mapping exercise |
| `config/vslam_nav_challenge.yaml`, `config/vslam_nav_challenge_solved.yaml` | navigation exercise |
| `rospider_gazebo/vslam_params.py` | participant file → RTAB-Map params + `arm_pose` |
| `rospider_gazebo/vslam_check.py` | grid metrics, closure check, levels, Thai report (pure, tested) |
| `scripts/check_vslam.py` | checker CLI (export, sqlite, report) |
| `launch/vslam_challenge.launch.py`, `launch/vslam_nav_challenge.launch.py` | the two exercises |
| `maps/vslam_challenge.{pgm,yaml}` | reference grid |
| `test/test_vslam_params.py`, `test/test_vslam_check.py` | unit tests |

Changed: `worlds/slam_challenge.sdf` (face visuals), `launch/vslam.launch.py`
(a `vslam_params` argument for the RTAB-Map file, `arm_pose` pass-through, localization on a copy,
`map_always_update`, Nav2 activation), `scripts/lifecycle_activate.py` and
`rospider_gazebo/lifecycle.py` (several nodes), `scripts/check_nav.py`
(resend), `CMakeLists.txt`, `package.xml` (`rtabmap` for `rtabmap-export`),
`ROSpider/SIMULATION.md` (a short "โจทย์" block in the V-SLAM section),
`CLAUDE.md`.

## Error handling

- Participant files: as in the other exercises.
- `check_vslam.py`: missing database (hint: was the launch closed, was the
  name the same), database still open, `rtabmap-export` missing or failing
  — each a Thai message and exit 1, no traceback.
- `vslam_nav_challenge.launch.py` without a map: Thai message naming
  exercise 1.

## Testing

Unit tests: `vslam_params` (every key lands under `rtabmap:` as a string,
numbers converted, `camera_view` mapping, unknown/blank refused, shipped
files merge); `vslam_check` (synthetic grids for each level; a closure link
that disagrees with odometry is counted wrong, one that agrees is not;
hints name no parameter); `lifecycle` (several nodes in order); the
`check_nav` resend logic with a fake client.

In the sim, with the shipped files and fresh launches: the answer key
passes both exercises; the starting files fail level 1; fixing one value
at a time passes the levels in order; the table values above are
re-measured with the committed room and files, and any wrong value that
does not produce its symptom is replaced and reported rather than bending
thresholds.

## Out of scope

- Relocalization from an unknown start pose (measured unreliable here).
- Making the database small enough to commit.
- Applying the Nav2 activation fix to `navigation.launch.py` (LiDAR Nav2);
  noted as a follow-up.


## As built (measured with the shipped files, 2026-09-29)

Changes from the design above, each a ledgered ruling:

- **Levels of the mapping checker.** With all three mistakes the camera at the floor still maps
  the floor within 0.5 m (7.7 m2), so the designed level 1 passed and fixing the camera then fell
  back to level 1. Level 1 is now "a map with its walls": known >= 1 m2 **and** >= 60% of the
  reference's walls that lie next to floor the learner mapped (walls of rooms never entered do
  not count). Level 2 is coverage >= 90% alone. Level 3 is unchanged.
- **Wrong closure threshold** 0.1 m / 5 deg -> 0.3 m / 10 deg: the answer key's own closures reach
  0.12 m / 2.9 deg (depth registration noise); aliasing closures are 1 m and more.
- **Level-3 mistake.** `Vis/MinInliers` 5 failed level 3 in 4 of 5 runs (kept, like the SLAM
  exercise's accepted partial fix); "3" passed 2 of 2; `RGBD/OptimizeMaxError` off alone passed,
  and together with `Vis/MinInliers` 5 bent the map past level 1.
- **Level 3 needs a return to the start** (final review): a closure whose older node lies within
  0.5 m of the first node and >= 30 nodes earlier; mid-route closures in room B and those made
  standing at the spawn point no longer count. The answer-key maps had 1-3 such closures.
- **Level 1 also reports wrong closures** when there are any (the floor camera made 9-14).
- **Localization grid.** Besides the copy and `map_always_update` false, the copy's cached 2D grid
  is rebuilt with `rtabmap-export --map --save_in_db`: the answer-key database's cached grid was a
  60x49-cell scrap, and every goal was "outside bounds".
- **Navigation mistakes.** `camera_view` ships `ahead` (a distractor): at the floor it stopped the
  robot at G1 only at 0.15 m/s, not at the starting 0.05 m/s. Two mistakes remain.

| Mapping run (tools/drive_route.py) | Result |
|---|---|
| answer key x3 | pass (17.1-17.3 m2, walls 94-100%, coverage 98-100%, 25-31 closures, 1.0-1.1x) |
| starting file | level 1 (walls 22%, 14 wrong closures) |
| camera fixed, range 0.5 | level 1 (0.5 m2) |
| camera at the floor, range fixed | level 1 (walls 27%) |
| only `Vis/MinInliers` 5 wrong | level 3 in 4 of 5 runs |
| answer key, room A only | level 2 (coverage 52%, walls 98%) |

| Navigation run (answer-key map, check_nav.py) | Result |
|---|---|
| starting file (radius 0.3, speed 0.05) | G1 41 s, no path to G2: level 2 |
| radius 0.15, speed 0.05 | 41 + 178 s, over 300 s in G3: level 3 |
| answer key x2 | 218 s, 203 s: pass |
