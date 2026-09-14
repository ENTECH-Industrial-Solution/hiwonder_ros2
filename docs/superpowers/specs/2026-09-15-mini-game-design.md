# Mini game: pick, carry through a hall, place by AprilTag

Scope: `ROSpider/src/simulations/rospider_gazebo`. The afternoon of the
one-day workshop: teams launch one file, tune a few values, and the robot
runs a pick-carry-place mission that the judges score.

## Goal

`ros2 launch rospider_gazebo mini_game.launch.py detector:=color|yolo`
starts a long-hall arena, Nav2 on a shipped 2D map, the chosen detector,
`apriltag_detect`, `pick_and_place` and a `mission` node that executes a
YAML step list: survey the stations, pick each pastel cube, drive through
the obstacles, place it on the station whose marker colour matches. A
scoring tool reads the final cube poses from Gazebo.

Everything the teams edit is YAML: `config/missions/basic.yaml` (steps,
waypoints, standoff), `config/color_detect.yaml` / the HSV tuner, or a
YOLO model they trained (`tools/capture_dataset.py` + `tools/train_yolo.py`).

## Non-goals

- No new perception. Detection, tag memory, `approach`, pick and place are
  the existing nodes; the mission only sequences them.
- No hardware path in this round.
- No multi-robot; each team runs its own simulator.

## Objects: three pastel cubes

`pink`, `yellow`, `sky` -- low saturation (S about 0.35) so the shipped
HSV bands (S >= 80..120) do not see them at all; teams must widen S and
then keep the light-grey floor out. Models `models/pick_cube_<name>`,
5 cm, same mass/friction as the RGB cubes. Each needs its own
`DetachableJoint` in `urdf/rospider_gazebo.urdf.xacro` and its two grasp
topics in `config/gz_bridge.yaml` (one plugin per child model; the plugin
tolerates the model being absent, so the RGB demo is unaffected).
`pick_and_place` gets `colors: [pink, yellow, sky]` from the mini-game
launch. The same three RGB triples colour the station markers.

## Stations with a colour marker

`tags.station_sdf(tag_id, marker_rgba=None)` (moved out of
`tools/make_tag_textures.py` so a launch file can call it) optionally adds a
0.20 x 0.20 m colour panel above the tag board on a taller post. The marker
says which cube belongs here; it is vertical, not on the floor, so a level
camera can read it from metres away. Colours are assigned to stations at
launch (`seed:=`), and the launch spawns the station from the generated SDF
text (`ros_gz_sim create -string`), so no per-colour model files exist.
`marker_rgba=None` produces byte-for-byte the committed `tag_station_*`
models, which `test_apriltag.py` already pins.

## Arena `worlds/arena.sdf`

A 5 x 1.6 m hall, 1 m walls with the poster textures (V-SLAM needs them).
`home` at one end with the pick pedestal and the three cubes in front of
it, clear of the robot by >= 0.4 m; the three stations across the far end
wall, 0.5 m apart, facing the hall. Obstacles: a box blocking the straight
line mid-hall, a 0.8 m gap between two pillars, a slanted panel near the
end. Everything is box geometry (no meshes). `maps/arena.yaml/.pgm` is
built with `slam.launch.py world:=arena` and committed; teams may rebuild
it with SLAM or V-SLAM but the game does not depend on that.
`config/nav2_arena_params.yaml` = `nav2_params.yaml` with `robot_radius`
raised (0.12) and speed 0.10 m/s if the gait holds at that speed
(measured in this round; otherwise 0.05 stays).

## Mission node `scripts/mission.py`

Reads `config/missions/<name>.yaml`:

```yaml
mission:
  waypoints:            # x, y, yaw in map
    home: [0.0, 0.0, 0.0]
    pick_table: [0.35, 0.0, 0.0]
    survey: [1.5, 0.0, 0.0]
  standoff: 0.30        # approach distance for place, metres from the base
  on_fail: skip         # skip | retry | stop
  steps:
    - survey: survey    # goto, then turn to read every station's tag +
                        # marker colour into a table tag -> colour
    - goto: pick_table
    - pick: pink
    - deliver: by_marker   # goto the station whose marker is the held
                           # colour (waypoint derived from the remembered
                           # tag pose), approach_tag, place
    - goto: pick_table
    - pick: yellow
    - deliver: by_marker
    - goto: pick_table
    - pick: sky
    - deliver: by_marker
    - goto: home
```

Blocks and what they call:

| block | underneath |
|---|---|
| `goto: <waypoint>` | Nav2 `NavigateToPose`; waits for the result |
| `survey: <waypoint>` | `goto`, then rotate in place through the stations (three yaw targets); for each frame reads `/apriltag_detect/apriltag_info` + `/yolo/object_detect` from `color_detect` (marker colours) and pairs a tag with the marker box nearest to it in the image; stores `tag -> colour` and asks `apriltag_detect` for the remembered pose (`/apriltag_detect/remembered_tags`, a small new publisher: id + pose in `memory_frame`) |
| `pick: <colour>` | `/pick_and_place/pick`, waits for state `CARRY` (`/pick_and_place/state`, a small new publisher) |
| `deliver: by_marker` or `deliver: <tag id>` | goal = tag pose moved `standoff + 0.3` m along the tag normal, facing the tag; `goto` it; `ros2 param set` on `apriltag_detect` (`behaviors.tagN.action: place`, `standoff`, `behaviors_enabled: true`); wait for the place service response in `/apriltag_detect/status` (new publisher: the status string the GUI shows) then disable behaviours |
| `place: here` | `/pick_and_place/place` with an empty string (floor) |
| `say: text` | log |

`on_fail: skip` moves to the next block on a failed Nav2 goal or a refused
service; `retry` tries once more; `stop` ends the mission. Every block logs
`[k/n] <block> ... ok/failed (t s)` and the node publishes elapsed time on
`/mission/elapsed` for the judges.

Two small additions to existing nodes make this possible without the
mission peeking into GUI state: `apriltag_detect` publishes
`~/remembered_tags` (id + pose in `memory_frame`, latched, on every change)
and `~/status`; `pick_and_place` publishes `~/state` (latched).

## Launch `mini_game.launch.py`

Arguments: `detector` (color|yolo), `map` (default `arena`), `mission`
(default `basic`), `seed` (int; marker colour assignment), `tune`, `rviz`
(default true), `gui` (default true), `auto_start` (default true: the
mission starts 5 s after Nav2 is active; false waits for
`/mission/start`).

Composition: `gazebo.launch.py world:=arena arm_pose:=horizontal` -> spawn
pedestal + cubes + stations (generated SDF with colours from `seed`) ->
`navigation.launch.py`-style Nav2 with `nav2_arena_params.yaml` and the
map -> detector (`color_detect` with `pick_place`-style tune wiring, or
`yolo_detect`) -> `apriltag_detect` (`scene` stations from the seed) ->
`pick_and_place` with `colors: [pink, yellow, sky]` -> `mission`.

## Scoring `tools/score.py`

Reads each cube's world pose from `gz model -m pick_cube_<c> -p` and the
station layout for the seed: cube within 0.12 m of the top of the station
whose marker is its colour = 10 points, on any other station = 3, on the
floor = 0; prints the table and the mission's elapsed time from
`/mission/elapsed`.

## Testing

- Unit (plain pytest): `station_sdf` marker variant parses and keeps the
  default byte-identical; mission YAML parsing and block validation; the
  goal-from-tag-pose maths (tag pose -> Nav2 goal facing the tag);
  survey's tag/marker pairing on synthetic boxes.
- Simulator: a full `basic` mission with `detector:=color` after tuning the
  pastel bands, and with `detector:=yolo` after `capture_dataset.py
  --samples 20` + `train_yolo.py` on the pastel cubes; record times.

## What the simulator changed (2026-09-15)

Recorded after the first full runs; the code and SIMULATION.md section 10
carry the details.

- Nav2: DWB found no valid trajectory in the 0.8 m gaps and sat through
  spin/backup recoveries; the arena uses Regulated Pure Pursuit, robot_radius
  0.15 (0.10 clipped a partition end and tipped the robot), inflation 0.18,
  0.15 m/s commanded (~0.10 walked), goal tolerance 0.15, AMCL's omni motion
  model (strafing), a static layer in the local costmap and a 300 s
  initial_transform_timeout.
- The pick pedestal is below the LiDAR plane, so it is painted into
  `maps/arena.pgm`; the pillars and the walls are painted too.
- `pick_table` stands 0.25 m behind the spawn pose and `pick` docks 0.5 m to
  an exact map pose (3-axis P-controller on TF), undocking by odometry.
  Nav2's tolerance is wider than the arm's reach window, and planning from
  beside the pedestal made RPP report "collision ahead".
- Game tags are 0.30 m (`stations.station_sdf(..., tag_size)`), the marker
  panels are emissive, and the taller post sits behind the board. The
  survey point is (3.0, 0.3) and the survey pans left/centre/right.
- The arena pedestal is 0.16 m deep (0.08 lost a cube off the back).
- `pick` while still holding a cube drops it on the floor first.
- `stations.py` holds the geometry and SDF with no cv2 import; a launch
  file importing cv2 crashed the Gazebo GUI through Qt's plugin path.
- `tools/capture_dataset.py --objects pastel --world arena --dock 0.25`;
  `models/yolo/pastel.pt` trained from 17 images (mAP50 0.915).
