# SLAM Mapping challenge ("fix the config, pass the levels")

Date: 2026-09-28
Status: approved in chat, awaiting spec review

## Purpose

The workshop's morning sim tutorial gives SLAM Mapping 20 minutes. After the
demo, participants get a 10-15 minute exercise that tests whether they
understand what the main slam_toolbox parameters do. It is the first of a
set of per-topic exercises (one per row of the schedule); the others come
later, each with its own spec.

Constraints:

- Not too hard: participants edit numbers in one YAML file, no code.
- In the sim, odometry has no drift, so loop-closure parameters have no
  visible effect. The exercise uses only parameters whose effect shows
  immediately in RViz: topic, range, resolution.
- Checking is automatic, so instructors do not have to inspect every screen.

## What the participant does

1. `ros2 launch rospider_gazebo slam_challenge.launch.py`. It reads
   `config/slam_challenge.yaml` and logs its source-tree path; the participant
   edits that file in place and relaunches (this needs
   `colcon build --symlink-install`). `git checkout` on the file restarts the
   exercise. This was the user's choice over a copy in `~/.ros`, accepting
   that a finished exercise leaves the repo file edited. `params:=<path>`
   uses another file and copies the starting config there if it is missing.
2. Drive with teleop through both rooms, then save the map with
   `map_saver_cli`.
3. `ros2 run rospider_gazebo check_slam.py <name>` (a map name in
   `ROSpider/maps/`, or a path to its `.yaml`) reports each level as pass or
   fail, with a hint that describes the symptom, not the parameter. A missing
   file gets its own hint (was it saved, is the name right), and `maps/<name>`
   without `.yaml` is accepted, since that is the form map_saver_cli takes.

## The room: `worlds/slam_challenge.sdf`

- 5 x 4 m inside the walls: x from -1.0 to 4.0, y from -2.0 to 2.0.
- A partition at x = 1.0 splits it into room A (spawn side, 2 m wide) and
  room B (3 m). The doorway is the gap y from 0.3 to 0.8 (0.5 m), wide enough
  for the robot to walk through.
- One 0.4 m square box in room A at (-0.3, 1.3) and a 1 m wall stub in room B
  at x = 2.8 from y 1.0 to 2.0. Walls reuse the existing `worlds/textures/`.
- Room B is a course (the user's sketch, added after the first version): maze
  walls from (1.0, 0.9) at the doorway's jamb to (2.0, 0.9), along x = 2.0 to
  y = -1.2, diagonally to (2.8, 0.0) and on to the stub. A cylinder at
  (3.13, -1.13), r 0.2, can be passed on either side (gaps 0.67-0.68 m; r 0.25
  left 0.58 m on one side, where Nav2's RPP stopped and aborted).
  Measured with the maze (coverage / wall ratio): answer key 100% / 0.9;
  range 0.5 70-73%; 0.25 m cells 2.5-2.7x (doorway also closed in one run); room A only 56%.
- The rooms must not look alike. A first version with two mirror-image
  2.5 m rooms and a centred doorway made loop closure match room B onto
  room A and shift the map 2.47 m (measured).
- The doorway check samples x = 1.0, y from 0.45 to 0.65.
- The robot spawns at world (0, 0), yaw 0, so the SLAM `map` frame equals
  world coordinates. The checker relies on this for the doorway position.

## The participant's config: `config/slam_challenge.yaml`

It holds six parameters, each with a Thai comment saying what it means.
Three are wrong and three are already correct (distractors). Every other
parameter comes from Hiwonder's `slam/config/slam.yaml`.

| Parameter | Starting value | Correct value | Symptom |
|---|---|---|---|
| `scan_topic` | `scan_raw` | `scan` | RViz stays empty; no map can be saved |
| `max_laser_range` | `0.5` | `12.0` | map only appears right around the robot |
| `resolution` | `0.25` | `0.05` | blocky map; the doorway closes |
| `map_update_interval` | `2.0` | (already correct) | — |
| `minimum_travel_distance` | `0.03` | (already correct) | — |
| `minimum_travel_heading` | `0.3` | (already correct) | — |

`config/slam_challenge_solved.yaml` holds the correct values. Instructors use
it to regenerate the reference map.

## Levels and how the checker measures them

The two maps can differ in resolution and origin, so they are compared in
metres: every reference cell centre is looked up in the participant's map
through that map's own `origin` and `resolution`. Cell states come from the
map YAML's `occupied_thresh`/`free_thresh`/`negate` (map_saver trinary PGM).

1. **Map exists** — the file loads and has at least 1 m² of known cells
   (free or occupied). On failure the hint is: no map; which topic is
   slam_toolbox listening to? Try `ros2 topic list`.
2. **Coverage** — of the reference map's free cells, at least 90% are known
   (free or occupied) in the participant's map. Known, not free: at 0.25 m
   cells the wall-side cells turn occupied, and that is level 3's symptom, not
   level 2's. The hint gives the measured percentage and says the
   map only sees close to the robot.
3. **Detail** — both of the following must hold:
   - every sample point on the doorway's centre line, 0.15 m or more from
     either jamb, is free (the recorded reference's jamb cells already reach
     0.05 m into the gap);
   - the participant's occupied area is at most 1.2x the reference's occupied
     area, measured within the reference map's bounds.

   Level 3 is checked only once level 2 passes: on a partial map an unseen
   doorway reads as blocked and unseen walls make the ratio small.

   The hint says the doorway is closed in the map and that the walls are Nx
   thicker than they really are.

The checker reports every level on every run; a level that cannot be judged
yet says which level to pass first. The doorway segment and all thresholds are constants in
`slam_check.py`, next to a comment tying them to the world file. The
thresholds come from sim runs on a scripted route through both rooms
(coverage / wall ratio): answer key 100% / 1.0; answer key without entering
room B 85-86%; starting range 0.5 about 20%; range fixed but 0.25 m cells
100% / 2.9 with 18 of 31 doorway samples blocked. A starting range of 1.0
was tried first and dropped: a thorough route still reached 93%.

## Files

Added under `ROSpider/src/simulations/rospider_gazebo/`:

| File | Role |
|---|---|
| `worlds/slam_challenge.sdf` | the exercise room |
| `config/slam_challenge.yaml` | the starting config (3 wrong values) |
| `config/slam_challenge_solved.yaml` | the answer key |
| `launch/slam_challenge.launch.py` | reads `config/slam_challenge.yaml` (or `params:=`, copied from it if missing); merges Hiwonder's `slam.yaml` with the participant's file (the participant's values win) into a temp file; includes `slam.launch.py` with `world` and `params_file` |
| `rospider_gazebo/slam_check.py` | pure Python + numpy: map loading (PGM reader, no cv2), alignment, the three measurements |
| `rospider_gazebo/challenge_params.py` | pure Python: copies the starting config, loads and validates the participant's file, merges it over Hiwonder's |
| `scripts/check_slam.py` | the CLI, installed for `ros2 run`: loads both maps and prints the levels in Thai; exit code 0 only if all pass |
| `maps/slam_challenge.{yaml,pgm}` | the reference map, saved from a sim run with the solved config |
| `test/test_slam_check.py`, `test/test_challenge_params.py` | unit tests |

Changed:

- `launch/slam.launch.py` — a new `params_file` argument, defaulting to
  Hiwonder's `slam.yaml`, so existing use is unchanged. nav2's
  `lifecycle_manager` is replaced by `scripts/lifecycle_activate.py`
  (`rospider_gazebo/lifecycle.py`, tested): the configure reply is sometimes
  lost on Jazzy, and the manager then waits forever with slam_toolbox
  inactive and no `/map` (seen in 1 of the first 2 runs). The activator
  re-reads the node's state instead of trusting replies.
- `slam_challenge.launch.py` sets `do_loop_closing: false` between Hiwonder's
  file and the participant's. Sim odometry has no drift, so loop closure only
  adds false corrections (0.19 m measured on the route; off: within 0.034 m).
- `CMakeLists.txt` — installs `scripts/check_slam.py`, registers both tests.
- `ROSpider/SIMULATION.md` — a short "โจทย์" subsection under SLAM Mapping,
  about 10 lines.

## Error handling

- Missing or unreadable map file: level 1 fails with its hint, and the other
  two levels are reported as not checked. No traceback.
- A malformed participant file (YAML syntax error): the launch
  stops with a message naming the file and line, rather than slam_toolbox
  failing later with an unrelated error.
- A misspelled key in the participant's file (slam_toolbox would silently
  ignore it) stops the launch, naming the key.
- Every value must have the type of Hiwonder's (rclcpp aborts on a mismatch):
  a whole number for a float (`max_laser_range: 12`) is converted; a blank
  value, a number for a topic name, or a non-number for a number stops the
  launch with a message naming the key.
- The launch logs which params file is in use on every run, so a stale edited
  file is never a silent surprise.

## Testing

Unit tests (`test_slam_check.py`, synthetic grids):

- an exact copy of the reference passes all levels;
- a map that knows only half the room fails level 2 and passes level 1;
- a copy downsampled to 0.25 m cells fails level 3 (doorway and thickness);
- a copy with a shifted origin but the same content still passes, which
  tests the alignment;
- a PGM write/read round trip.

In the sim, with a scripted `cmd_vel` route so runs repeat:

- the starting config shows each symptom in the table and fails the expected
  levels;
- fixing one value at a time passes the levels in order;
- the solved config passes all three levels; this run also produces the
  reference map;
- time the full exercise to confirm it fits in 10-15 minutes.

## Out of scope

- Odometry noise and loop-closure tuning (too hard for 20 minutes).
- Exercises for the other workshop topics, which get their own specs.
- Removing the leftover pastel-cube grasp joints from the URDF (unrelated
  cleanup).
