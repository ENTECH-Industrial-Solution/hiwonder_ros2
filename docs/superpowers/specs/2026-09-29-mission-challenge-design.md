# Final mission ("fetch the blue block from room B and bring it home")

Date: 2026-09-29
Status: implemented; values measured in the sim (see "As built")
Asked for by: the user, after the ten single-topic exercises ("ภารกิจปิดท้าย"). Decisions taken with
them: 30 minutes at the end of the workshop day; three stages (Nav2 → pick → place); the participant
writes a mission file (YAML), not code.

## Purpose

The closing exercise of the one-day workshop. The ten exercises each tune one subsystem; this one
chains four of them in the maze room the participants already know:

1. Nav2 crosses from room A to room B.
2. AprilTag tracking brings the robot to the right station.
3. The depth camera and the arm pick the block on that station.
4. Nav2 brings the robot back, and the arm puts the block down on a pad.

What the participant does is write the order of the steps and their values. For that they must read
coordinates off the map, pick the station by its tag number, and think about where a "place" point
relative to the robot lands in the room.

Success for the workshop: a participant who finished the single exercises (or not: see "Tuning")
completes the mission within 30 minutes of trying, each check taking about 4-5 minutes.

## The brief (the participant's view)

> บล็อกสีน้ำเงินวางอยู่บนแท่นหน้าป้าย AprilTag หมายเลข 3 ในห้อง B
> ให้หุ่นไปหยิบมา แล้ววางบนแผ่นสีเหลืองในห้อง A (กลางแผ่นอยู่ที่ x = 0.40, y = 0.55 บนแผนที่)
> ในห้อง B มีแท่นอีกแท่น (ป้ายหมายเลข 2) ที่มีบล็อกสีแดง อย่าหยิบผิด

## Scene: `worlds/mission_challenge.sdf`

`worlds/slam_challenge.sdf` (walls, per-face posters, box, pillar, unchanged) plus two tag stations
(`stations.station_sdf`) embedded at build time by `tools/make_mission_world.py`, as
`make_apriltag_world.py` does. The yellow pad is also embedded: visual only, 0.12 m square.

| Thing | Pose (world = map frame, robot spawns at the origin) | Why there |
|---|---|---|
| station, tag 2, red block | (3.75, 1.0), facing -x | against the east wall of room B, clear of the Nav2 course |
| station, tag 3, blue block | (3.75, -0.3), facing -x | 1.3 m from the other one, so pick-here in front of one cannot reach the other |
| yellow pad | (0.40, 0.55) | room A, just inside the doorway, so the way back is short |

- The blocks are spawned at run time (they are dynamic) by the launch file, as
  `track_and_grab.launch.py` does, with the model names the grasp welds expect: `pick_cube_red` and
  `pick_cube_blue`. The welds are one `DetachableJoint` per model name
  (`urdf/rospider_gazebo.urdf.xacro`), which is why the two blocks are two colours.
- Each block sits on its station's pedestal 0.035 m in front of the pedestal centre, where the robot
  that `go_to_tag` parks has it at the pick spot. The exact offset comes from the "Measure first"
  step below.
- The stations are not in the reference map `maps/slam_challenge.*`. Their pedestals are below the
  LiDAR plane, and their posts show up as small obstacles that Nav2's local costmap handles. Both
  stand clear of every path the mission needs.

## Step types: the mission file

`config/mission_challenge.yaml`, edited in place like the other exercises. It has two top-level
keys, `speed` and `steps`.

**`speed`** is the walking speed for every `go_to`, in m/s, allowed from 0.05 to 0.20. The gait
caps near 0.23 m/s.

- It is a free knob, not scored (the user's choice). The draft ships 0.15, the answer key's Nav2
  speed. A participant may raise it to make a check faster, or lower it if the robot overshoots
  while carrying the block.
- Before each `go_to` the runner sets it live on Nav2 with `SetParameters`, so the file needs no
  relaunch:
  - `controller_server`: `FollowPath.desired_linear_vel`
  - `velocity_smoother`: `max_velocity[0]` (the same two places `nav_params.py` writes `max_vel_x`)
- It does not touch `go_to_tag`, whose approach speed stays at the answer key's.

**`steps`** is a list of one-key mappings:

| Step | Value | Done by | Succeeds when | Gives up after (sim s) |
|---|---|---|---|---|
| `go_to` | `[x, y, yaw_deg]` in the map frame | Nav2 `NavigateToPose` (planned first, `no_path` reported at once) | the goal succeeds | 240 |
| `go_to_tag` | tag id (int) | `apriltag_track` with that `target_tag`, switched on, then off | the robot walked and stood still 3 s with the tag in view | 60 |
| `pick` | `red`, `green` or `blue` | `track_and_grab` in pick-here mode, place by button (holds the block) | `pick_and_place` reaches `CARRY` | 60 |
| `place` | `'x y z'` metres in `base_footprint` (x ahead, y left, z up) | `/pick_and_place/place` with that point | `pick_and_place` is back to `IDLE`/`DONE` | 40 |

`rospider_gazebo/mission_plan.py` (pure, tested) turns the file into a list of steps, or refuses it
with a Thai message naming the step's number:

- an unknown step name
- a step that is not a one-key mapping
- a value of the wrong type or length
- a colour that is not one of the three
- a `place` that is not three numbers
- an empty list
- a missing `speed`, or one that is not a number from 0.05 to 0.20

It checks nothing about whether the plan makes sense: that is the exercise.

### Starting file: a colleague's draft with three mistakes

```yaml
speed: 0.15                      # right; a free knob
steps:
  - go_to: [0.6, 1.5, 0]         # wrong: still room A (level 1)
  - go_to_tag: 2                 # wrong: the red block's station (level 2)
  - pick: blue
  - go_to: [0.0, 0.0, 0]         # wrong: the spawn, not the pad (level 3)
  - place: '0.20 0.0 0.035'
```

`config/mission_challenge_solved.yaml` is the answer key. It is the same shape with:

- a room-B waypoint in front of the stations
- `go_to_tag: 3`
- a return goal 0.20 m short of the pad centre, facing it, so the `place` point 0.20 m ahead lands on
  the pad

## Tuning: answer-key values, not the participant's

Every subsystem underneath runs with the single exercises' answer-key values:

- Nav2: `nav_challenge_solved.yaml`, through `nav_params.py`
- colour bands: `color_detect.yaml`
- `apriltag_track`: speed and turn limits from `apriltag_challenge_solved.yaml`; `stop_distance`
  set by this exercise (next section)
- `pick_and_place`: its own config

A participant who did not finish an earlier exercise can still do the mission. The mission tests
putting the parts together, not tuning them again.

## Node changes

- **`apriltag_track.py`**
  - While not following, it publishes nothing on `/controller/cmd_vel`: one zero twist when
    switched off, then silence. Today it sends a stop every frame, which would fight Nav2 and
    track_and_grab.
  - `target_tag` becomes settable at run time (a parameter callback), so the runner can switch
    stations without relaunching.
- **`track_and_grab.py`**: no change. Its services already cover the step (`~/set_walk`,
  `~/set_auto_place`, `~/pick`); `~/set_running false` brings the camera down to the look pose
  before a pick.
- **`pick_and_place.py`**: no change. `~/place` already takes a point.

## The runner and checker: `check_mission.py`

One command both runs the mission and scores it (`ros2 run rospider_gazebo check_mission.py`).

1. Validates the file (`mission_plan`) and prints the plan as numbered Thai lines.
2. Waits for the stack: Nav2 active (`nav_check`'s lifecycle wait), the services of `apriltag_track`,
   `track_and_grab` and `pick_and_place`, and `/odom`.
3. Refuses unless the robot is within 0.1 m of the spawn and both blocks sit on their pedestals
   (`gz model -p`, `grasp_check.parse_gz_pose`).
4. Runs the steps in order, printing each as it starts and ends: `ขั้น 2/5 go_to_tag 2 ... ผ่าน (14 วินาที)`.
   - It stops at the first failed step, with the reason in Thai: `no_path`, a timeout, a tag never
     seen, a pick refused, and so on.
   - Every subsystem it switched on is switched off again (on Ctrl+C too).
5. Records facts along the way:
   - whether the robot's pose ever entered room B
   - which block was lifted after each `pick`
   - where the blocks came to rest at the end
6. Scores with `rospider_gazebo/mission_check.py` (pure, tested).

| Level | Passes when | The starting file fails it because |
|---|---|---|
| 1 ไปถึงห้อง B | the robot was in room B (x > 2.1 in the map frame) at some point | the first goal is in room A; the next step then sees no tag |
| 2 ถือบล็อกสีน้ำเงิน | the blue block was lifted (z > 0.15 m) after a `pick` | the robot parks at the red block's station and pick-here cannot reach blue |
| 3 บล็อกสีน้ำเงินอยู่บนแผ่นเหลือง | the blue block rests on the floor within 0.05 m of the pad centre | the block is put down next to the spawn |

Hints describe what happened, never a key, and name the failed step:

- **level 1:** where the robot ended, and "ห้อง B อยู่ทางไหนของแผนที่"
- **level 2:** which tag the robot parked at, or that the tag was never seen, or which colour it
  lifted
- **level 3:** how far the block is from the pad and on which side of it, and "place นับจากตัวหุ่น
  ตอนนั้นหันไปทางไหน"

## Launch: `mission_challenge.launch.py`

Starts, in the order and with the delays the existing launches use:

- `navigation.launch.py` with the mission world, map `slam_challenge`, and the Nav2 parameters
  merged as `nav_challenge.launch.py` does with the solved file (RViz included)
- `color_detect`, `apriltag_detect`, `apriltag_track` (`start: false`)
- `pick_and_place` (`auto_start: false`, no scene of its own)
- the `track_and_grab` window (`walk: false`, `auto_place: false`)
- the two block spawns

`map:=` takes the participant's own map from the SLAM exercise (a name in `ROSpider/maps` or a
path, resolved like `nav_challenge.launch.py`'s). The default is the reference map
`maps/slam_challenge`: the user chose own maps as an optional harder variant, so that participants
without a good map can still do the mission. A SLAM map starts at the spawn, so every coordinate in
the mission (waypoints, room B, pad) means the same on it. The checker prints which map Nav2 serves
(`/map_server` `yaml_filename`). When a `go_to` fails with `no_path` on a map that is not the
reference, the failure says to check that the map covers room B.

The participant's file is read by the checker, not by the launch, so editing it needs no relaunch.
A relaunch is still needed between checks, to put the robot and the blocks back.

## Measure first (the implementation plan's first task)

Before the scene is fixed, measure in the sim (and record in "As built"):

- **Station park.**
  - The `stop_distance` that parks the robot so the block on the pedestal sits where pick-here
    grabs it: 0.20-0.27 m ahead of `base_footprint`, measured in the grasping exercise.
  - The pedestal offset that goes with it.
  - Accept when pick-here lifts the block in 3 of 3 runs from a `go_to_tag` park.
- **The carried block and Nav2.** With the block held in `CARRY`, Nav2 must still plan and drive.
  The LiDAR ignores robot visuals but not the held block, which sits 0.18 m ahead and 0.22 m up.
  If the LiDAR marks it, give the blocks' visuals the robot's `visibility_flags`, so the LiDAR's mask
  skips them as it skips the legs.
- **Live speed.** Setting `desired_linear_vel` and the smoother's `max_velocity` between goals
  takes effect on the next goal. Check with the robot's odometry at 0.10 and at 0.20. The answer
  key passes 3 of 3 runs at 0.20 too, or the allowed maximum comes down to the fastest speed that
  does.
- **`cmd_vel` silence.** With every node idle, nothing publishes on `/controller/cmd_vel` for 10 s
  (`ros2 topic hz`).
- **Load.** The whole stack runs at a real-time factor that keeps a check under ~5 minutes of wall
  time. If it does not, the first thing to drop is RViz.
- **Coordinates.**
  - Stations, pad and room-B waypoint are reachable, and Nav2's paths stay clear of the posts.
  - The answer key passes 3 of 3 runs.
  - Each single mistake, fixed in the order level 1 → 3, moves the result one level.

## Files

| File | What |
|---|---|
| `tools/make_mission_world.py`, `worlds/mission_challenge.sdf` | the scene (generated from `slam_challenge.sdf` + stations + pad) |
| `models/tag_station_3/`, `worlds/textures/tag_3.png` | station 3 (already generated, uncommitted until now) |
| `config/mission_challenge.yaml`, `config/mission_challenge_solved.yaml` | draft and answer key |
| `launch/mission_challenge.launch.py` | the stack |
| `rospider_gazebo/mission_plan.py`, `test/test_mission_plan.py` | file → steps, refusals |
| `rospider_gazebo/mission_check.py`, `test/test_mission_check.py` | facts → levels, Thai report |
| `scripts/check_mission.py` | runner + checker |
| `scripts/apriltag_track.py` | quiet when off; live `target_tag` |
| `ROSpider/SIMULATION.md`, `CLAUDE.md` | a "ภารกิจปิดท้าย" section; a bullet |

## Testing

- **Unit (pytest, pure):**
  - `mission_plan`: every refusal (`speed` out of range included), and the shipped files parse.
  - `mission_check`: every level outcome from recorded facts, and hints that name no key.
- **Live, recorded in "As built":**
  - the answer key three times
  - the starting file
  - the file after each fix, in order
  - the `cmd_vel` silence check
- **Own map:** the answer key passes once on a map recorded with the SLAM exercise's answer key.
- **Existing suite:** `apriltag_track`'s change must leave the AprilTag exercise's measured results
  as they are. Re-run its answer key once.

## Out of scope

- SLAM and V-SLAM: mapping takes too long for a 30-minute slot. The mission uses the reference map.
- Loops, conditions or retries in the mission file: a list of steps is enough for three stages.
- A time limit level. `speed` is a free knob (the user's choice); scoring time would make the
  draft's checks slower (8-10 minutes at a slow start) in a 30-minute slot.

## As built (2026-09-29)

Changes from the design above, each measured first:

- **Scene: stations on room B's north wall, blocks on pick pedestals of their own.**
  - Stations stand at (3.2, 1.8) (tag 2, red) and (3.8, 1.8) (tag 3, blue), facing -y, 0.6 m apart.
  - Each block stands on a `models/pick_pedestal` 0.35 m in front of its station, 0.035 m past the
    pedestal's centre (the pick_place scene's layout).
  - On the station's own pedestal the park was right (block 0.237 m ahead), but track_and_grab
    never picked. Its free-standing test saw the pedestal top just behind the block and took the
    block for a poster patch.
  - The extra pedestal needs room the east wall did not leave: the robot's rear would reach the stub
    at x = 2.8.
- **`STOP_DISTANCE` 0.55.** The park left the block 0.245-0.249 m ahead, and pick-here lifted it in
  3 of 3 runs (CARRY after 11-12 s).
- **The pick step backs off 0.2 m after CARRY** (0.1 m/s for 2 s, like track_and_grab's undock).
  The robot stands 0.13 m from the pedestal, and Nav2's first turn in place would drag a foot over
  it. The step table in the mission file says so.
- **`LIFTED_Z` 0.13, not 0.15.** At CARRY the block measured 0.153-0.158 m high: the lift, before
  the carry pose.
- **The launch releases the grasp welds itself.** A `gz topic` detach burst runs 10 x 0.5 s from
  t = 7 s. With Nav2 loading, pick_and_place's start-up detach stopped before the blocks' weld
  subscribers existed, and the arm's first pose swung both blocks 3 m through the air.
- **The carried block does not blind the LiDAR.** The return goal succeeded while holding it
  (56 s), so no `visibility_flags` were needed.
- **Live speed works.** Both `FollowPath.desired_linear_vel` and the smoother's `max_velocity[0]`
  accept `SetParameters`. Peak odometry speed was 0.100 m/s at 0.10 and 0.201 m/s at 0.20.
- **Idle silence.** With every node idle, `/controller/cmd_vel` carries nothing, now that
  `apriltag_track` is quiet when off.
- **Load.** The real-time factor is ~0.45-0.55, with or without RViz (RViz stays on).
- **The Nav2 client moved to `rospider_gazebo/nav_client.py`.** `check_nav.py`'s answer key still
  passes: 25 + 104 + 80 = 208 s.

### Measured runs

| File | Result |
|---|---|
| answer key, 3 runs | pass; steps ~55 + 14 + 17 + 59 + 7 s sim; 392 / 404 / 379 s wall |
| draft | stops at step 2 (`มองไม่เห็นป้ายหมายเลข 2`): level 1; 110 s wall |
| + room-B waypoint | stops at step 3 (pick refused): level 2, naming tag 2 and its red block |
| + `go_to_tag: 3` | level 3: the block 0.64 m from the pad, "the pad to the robot's left, the block ahead" |
| answer key at `speed: 0.20` | pass; 348 s wall |
| answer key, `map:=` a renamed copy of the reference | prints that map; pass |
| Ctrl+C during step 1 | `หยุดตรวจแล้ว`; the robot stands still; `/controller/cmd_vel` silent |

A check takes 6-7 minutes of wall time at a real-time factor of ~0.45, not the 4-5 the design
hoped for. That leaves about four tries in 30 minutes; `speed: 0.20` saves about a minute.
