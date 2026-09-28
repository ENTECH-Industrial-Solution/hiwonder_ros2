# Nav2 challenge ("fix the Nav2 config, run the course")

Date: 2026-09-28
Status: implemented; values measured in the sim
Follows: `2026-09-28-slam-challenge-design.md` (same room, same pattern)

## Purpose

The second workshop exercise, after the SLAM one. Participants load a map of
the exercise room into Nav2 and must make the robot run a fixed course
through both rooms smoothly. Three Nav2 values are deliberately wrong; an
automatic checker drives the course and scores it in three levels. Target
time: 10-15 minutes, like the SLAM exercise (the schedule gives SLAM Mapping
and RTAB-VSLAM 20 minutes each; which slot this uses is the instructor's call).

Constraints:

- Not too hard: participants edit six values in one YAML file, no code.
- Checking is automatic: the checker sends the goals itself; the participant
  does not click anything.
- Hints describe the symptom, never the parameter name.

## What the participant does

1. `ros2 launch rospider_gazebo nav_challenge.launch.py` starts the exercise
   room (`worlds/slam_challenge.sdf`), the reference map
   (`maps/slam_challenge.yaml`, installed with the package), Nav2 and RViz.
   `map:=<name>` uses the participant's own map from the SLAM exercise instead
   (looked up like `navigation.launch.py`'s `map:=`).
2. Edit `config/nav_challenge.yaml` in place (the file in the source tree, as
   the SLAM exercise does; it needs `colcon build --symlink-install`) and
   relaunch.
3. `ros2 run rospider_gazebo check_nav.py` runs the course and prints each
   level in Thai. It first reminds the participant to relaunch before
   checking, so the robot starts at the spawn point and times compare.

## The participant's config: `config/nav_challenge.yaml`

Same file shape as `config/slam_challenge.yaml` (`/**: ros__parameters:`),
six friendly keys, each with a Thai comment. Three are wrong; three are
already right (distractors). The values below were measured in the sim.

| Key | Starting value | Solved value | Symptom (measured) |
|---|---|---|---|
| `xy_goal_tolerance` | `0.002` | `0.05` | at 0.15 m/s G1 never finishes (timeout); at the starting 0.05 m/s it settles after ~100 s of circling |
| `robot_radius` | `0.3` | `0.15` | no free cell in the doorway; the planner reports no path to G2 at once |
| `max_vel_x` | `0.05` | `0.15` | in the maze the course passes the 300 s limit during G3 (443 s when let run; answer key 171-216 s) |
| `inflation_radius` | `0.3` | (already right) | — |
| `yaw_goal_tolerance` | `0.1` | (already right) | — |
| `max_vel_theta` | `0.25` | (already right) | — |

Measured, not assumed (see the implementation ledger):

- **Controller.** Hiwonder's DWB could not run the answer key: it stood still in
  front of the doorway and again when a goal needed a U-turn. The challenge runs
  Regulated Pure Pursuit instead (the old mini game's measured block), with
  `cost_scaling_factor` 3.0 so NavFn keeps the path centred in the doorway.
- **Doorway.** With DWB, `inflation_radius` equal to `robot_radius` left no cost
  gradient; NavFn (which only avoids lethal cells) planned through the inscribed
  band and DWB refused to follow. With RPP and `robot_radius` 0.3 the planner
  finds no path at all, which the checker reports immediately.
- **Speed.** The starting value is Hiwonder's real-robot 0.05, not 0.03: at 0.03
  the G2 leg alone would outlast the per-goal timeout and fail level 2 for
  slowness. Each goal has its own timeout: G1 120 s (a circling robot is
  reported within two minutes), G2 and G3 300 s (G2 at 0.05 takes 210 s in the
  maze). The checker stops as soon as the running total passes the limit.
- **Levels are not strictly one parameter each.** While the speed is still 0.05
  the tolerance mistake sometimes settles after ~100 s (level 1 passes) and
  sometimes times out at 120 s (both measured); at 0.15 it always timed out.
  The course cannot pass without all three fixed.

`config/nav_challenge_solved.yaml` holds the solved values for instructors.

### Where each key lands (`rospider_gazebo/nav_params.py`, after the RPP swap)

Nav2's parameters are nested per node, and several meanings live in more than
one place; editing only one of them would change nothing visible. Each
friendly key is written to every place it belongs:

| Key | Paths in `config/nav2_params.yaml` |
|---|---|
| `xy_goal_tolerance` | `controller_server.general_goal_checker.xy_goal_tolerance` (RPP has no copy) |
| `yaw_goal_tolerance` | `controller_server.general_goal_checker.yaw_goal_tolerance` |
| `robot_radius` | `local_costmap.local_costmap.robot_radius`, `global_costmap.global_costmap.robot_radius` |
| `inflation_radius` | the `inflation_layer.inflation_radius` of both costmaps |
| `max_vel_x` | `controller_server.FollowPath.desired_linear_vel`, `velocity_smoother.max_velocity[0]` |
| `max_vel_theta` | `controller_server.FollowPath.rotate_to_heading_angular_vel`, `velocity_smoother.max_velocity[2]` |

(all under each node's `ros__parameters`). Validation reuses
`challenge_params`: unknown keys, blank values and wrong types stop the
launch with a Thai message naming the key; whole numbers become floats.

## The course and the levels (`rospider_gazebo/nav_check.py`)

The checker sends three `NavigateToPose` goals in the `map` frame, one after
another, each with its own timeout (G1 120 s, G2 and G3 300 s):

1. G1, a corner of room A: (-0.4, -1.3)
2. G2, the top-left corner of room B, through the doorway, the corridor and
   past the cylinder on either side: (3.5, 1.5)
3. G3, back to the start: (0, 0)

| Level | Passes when |
|---|---|
| 1 Reach a goal | G1 ends SUCCEEDED within its timeout |
| 2 Through the doorway | G2 ends SUCCEEDED within its timeout |
| 3 The whole course in time | all three succeed and their total time is at most the limit (300 s) |

Before each goal the checker asks the planner for a path
(`ComputePathToPose`, 15 s): a failure is reported at once as "no path",
instead of waiting out Nav2's recovery retries, and gets its own hint. The
checker stops at the first failed goal (a robot circling at G1 would circle
at every goal) and measures time on the sim clock, so a slower PC is not
penalised.

The time limit is set from the solved config's measured course time plus a
margin, and recorded next to the constant with the measurements. A level
that cannot be judged yet reports which level to pass first (as in the SLAM
checker). The pure part (course constants, level logic, Thai report) is in
`nav_check.py`; `scripts/check_nav.py` only talks to the action server and
records each goal's final status and duration.

## Files

Added under `ROSpider/src/simulations/rospider_gazebo/`:

| File | Role |
|---|---|
| `config/nav_challenge.yaml` | the starting config (3 wrong values) |
| `config/nav_challenge_solved.yaml` | the answer key |
| `rospider_gazebo/nav_params.py` | friendly key -> nested paths; writes the merged Nav2 params to a temp file |
| `launch/nav_challenge.launch.py` | includes `navigation.launch.py` with `world:=slam_challenge`, the reference map (or `map:=`), and the merged params |
| `rospider_gazebo/nav_check.py` | course, level logic, Thai report (pure, tested) |
| `scripts/check_nav.py` | the checker CLI, installed for `ros2 run` |
| `test/test_nav_params.py`, `test/test_nav_check.py` | unit tests |

Changed: `CMakeLists.txt` (install the script, register the tests),
`ROSpider/SIMULATION.md` (a short "โจทย์" block under Mapping/Navigation),
`CLAUDE.md` (one bullet).

## Error handling

- Participant file: as in the SLAM exercise (unknown key, blank value, wrong
  type, YAML syntax error each stop the launch with a message naming the
  file and key or line).
- Checker started without Nav2 running: it waits up to 30 s for the action
  server, then says so in Thai and exits 1, no traceback.
- A goal rejected by Nav2 counts as a failed goal for its level.
- Ctrl+C during the course cancels the current goal before exiting, so the
  robot does not keep driving.

## Testing

Unit tests:

- `nav_params`: a key reaches every path in its table (checked against the
  real `config/nav2_params.yaml`); `max_velocity` list entries are replaced
  by index, not the whole list; unknown keys, blank values and wrong types
  are refused; the shipped starting and solved files merge cleanly.
- `nav_check`: all SUCCEEDED within the limit passes every level; G1 ABORTED
  fails level 1 and skips the rest; G2 ABORTED fails level 2; all SUCCEEDED
  but over the limit fails level 3; hints name no parameter.

In the sim, with fresh launches:

- the starting config shows the symptoms in the table and fails level 1;
- fixing one value at a time passes the levels in order;
- the solved config passes all three; its course time sets the level-3 limit;
- time the course with the solved config and with `max_vel_x` at Hiwonder's
  0.05, to check the 10-15 minute budget.

If a wrong value does not produce its symptom in the sim, it is replaced by
another value from the same file and the user is told, rather than bending
the thresholds.

## Out of scope

- Tuning the DWB critics, AMCL or the planner choice.
- Obstacles that appear during the run (the costmaps would need live
  sensors in the scoring).
- Exercises for the other workshop topics.
