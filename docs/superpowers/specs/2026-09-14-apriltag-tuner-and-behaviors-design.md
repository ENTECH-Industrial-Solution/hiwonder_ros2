# AprilTag tuner GUI and per-tag behaviours (simulation)

Scope: `ROSpider/src/simulations/rospider_gazebo`. First of two rounds; the
second (a YOLO capture/label/train GUI) is designed separately after this one
has been tried.

## Goal

`ros2 launch rospider_gazebo pick_place.launch.py tune:=true` opens a Tk
window for `apriltag_detect` in which the user can

1. tune the aruco detector live and see the effect on the overlay, and
2. assign a behaviour to each tag id -- `none`, `approach`, `place`, `stop` --
   and its parameters, switch behaviours on, watch the robot execute them,

and save the result to a JSON file that overrides `config/apriltag.yaml` on
the next launch, exactly as `color_detect` already does with its HSV bounds.

## Non-goals

- No change to `color_detect.py`; its OpenCV-trackbar tuner stays as it is.
- No new behaviours beyond the four above (no pick, no turn).
- No hardware path. `/controller/cmd_vel` and `/pick_and_place/place` are the
  only outputs, and both exist on the real robot, but nothing here is tested
  on it.

## Files

| File | Change |
|---|---|
| `rospider_gazebo/tkview.py` | new: Tk helpers shared with the later YOLO GUI |
| `rospider_gazebo/tag_settings.py` | new: defaults, YAML/JSON merge, YAML rendering |
| `rospider_gazebo/tag_behavior.py` | new: behaviour maths, no ROS |
| `rospider_gazebo/tags.py` | `detect_tags(gray, params=None)`; `detector_parameters(dict)` |
| `scripts/apriltag_detect.py` | detector params, tuned JSON, behaviour execution, Tk tuner |
| `config/apriltag.yaml` | new blocks `detector`, `control`, `behaviors` |
| `launch/pick_place.launch.py` | pass `tune` to `apriltag_detect` too |
| `package.xml` | `<exec_depend>python3-tk</exec_depend>` |
| `test/test_apriltag.py` | detector params round-trip |
| `test/test_tag_settings.py`, `test/test_tag_behavior.py` | new |
| `CMakeLists.txt` | register the new test |
| `SIMULATION.md` | section 8 gains "จูนและกำหนดพฤติกรรม" |
| `CLAUDE.md` | one bullet under the sim section |

## Detector parameters

`tags.detector_parameters(values)` builds a `cv2.aruco.DetectorParameters`
from a dict; `detect_tags(gray, params)` uses it (default: stock parameters,
so existing tests and callers are unchanged). Exposed keys, all under
`detector:` in the YAML and as flat node parameters `detector.<key>`:

| key | aruco field | default | GUI range |
|---|---|---|---|
| `adaptive_thresh_win_size_min` | adaptiveThreshWinSizeMin | 3 | 3..50 |
| `adaptive_thresh_win_size_max` | adaptiveThreshWinSizeMax | 23 | 3..100 |
| `adaptive_thresh_win_size_step` | adaptiveThreshWinSizeStep | 10 | 1..50 |
| `adaptive_thresh_constant` | adaptiveThreshConstant | 7.0 | 0..30 |
| `min_marker_perimeter_rate` | minMarkerPerimeterRate | 0.03 | 0.005..0.2 |
| `polygonal_approx_accuracy_rate` | polygonalApproxAccuracyRate | 0.03 | 0.01..0.2 |
| `corner_refinement` | cornerRefinementMethod | `none` | `none` / `subpix` |

Odd window sizes are enforced (aruco requires them): even values are rounded
up. `max_reproj_error_px` stays where it is (top level) and gets a slider.

## Behaviours

### Config

```yaml
apriltag_detect:
  ros__parameters:
    behaviors_enabled: false          # GUI checkbox; never true by default
    control:
      max_linear: 0.05                # m/s, same ceiling Nav2 uses here
      max_angular: 0.3                # rad/s
      kp_yaw: 1.5                     # rad/s per metre of lateral offset
      kp_dist: 0.4                    # m/s per metre of range error
      yaw_deadband: 0.02              # m of lateral offset
      dist_deadband: 0.03             # m
      lost_timeout: 0.5               # s without the tag before releasing
      place_height: 0.055             # m above the pedestal top, = drop_slots
      turn_first_rad: 0.35            # bearing beyond which the robot turns in place
    behaviors:
      tag0: {action: none, standoff: 0.4}
```

`behaviors` keys are `tag<id>`; `standoff` is the range from
`base_footprint` to the tag, along the robot's heading, at which
`approach`/`place` stop. Same nested-parameter shape as
`pick_place.yaml`'s `drop_slots` (ROS 2 cannot hold a list of dicts).

Tuned JSON (`tuned_path`, default `~/.ros/apriltag_tuned.json`) holds
`{"detector": {...}, "max_reproj_error_px": ..., "control": {...},
"behaviors": {...}}` and overrides the YAML key by key, using the same
merge/precedence code shape as `color_detect._merge_settings`.
`behaviors_enabled` is deliberately NOT saved: every launch starts disabled.

### `TagBehavior` (rospider_gazebo/tag_behavior.py)

Pure Python, unit-tested. State: per-tag action table, control gains, and the
active target (`tag_id`, `phase`, `last_seen`).

`update(now, sightings) -> Decision` where `sightings` is
`[(tag_id, tvec)]` for the current frame (optical frame: x right, y down,
z forward) and `Decision` has

- `twist`: `(linear_x, angular_z)` or `None` (nothing to publish),
- `place`: a tag id when `~/place` must be called now, else `None`,
- `status`: a short string for the GUI.

Rules:

- Candidates are sightings whose tag has an action other than `none`;
  the nearest by planar range (`hypot(x, z)`) wins, not the smallest z -- a
  remembered tag can sit behind the camera (z < 0), and raw z would let it
  beat a nearer, visible tag just because a negative number compares less
  than a positive one. A `stop` tag beats everything.
- `stop`: twist `(0, 0)` every frame it is seen.
- `approach`: if the tag's bearing `atan2(x, z)` exceeds `turn_first_rad`
  in magnitude (including any tag behind the camera, z <= 0), turn in place
  toward it at `max_angular` with no linear motion -- a P law on x alone
  would back the robot through a station that is behind it. Otherwise
  `angular_z = clamp(-kp_yaw * x, ±max_angular)` unless
  `|x| < yaw_deadband`; `linear_x = clamp(kp_dist * (z - standoff),
  ±max_linear)` unless `|z - standoff| < dist_deadband`. When both are inside
  their deadbands the phase becomes `reached` and the twist is `(0, 0)`.
- `place`: as `approach`; on the frame it first reaches the standoff,
  `place` is set to the tag id once and the id is recorded as placed. A
  placed tag holds `(0, 0)` while in view and cannot fire again until it has
  been out of sight longer than `lost_timeout`. Placed state is per tag id,
  not per current target, so another tag transiently being nearer cannot
  re-arm it.
- No candidate: if a target was active less than `lost_timeout` ago, hold
  `(0, 0)`; once past it, publish `(0, 0)` exactly once more and then return
  `twist=None` so teleop/Nav2 can own `/controller/cmd_vel`.
- Disabled: `update` returns `twist=None`, but still tracks nothing --
  flipping the switch on starts from `idle`.

### Tag memory (node side)

The camera is on the wrist (`link4`), so in `CARRY` the held cube fills the
frame and the tag is never visible while carrying. `place` therefore cannot
rely on a live sighting. The node remembers every tag it sees:

- On each detection with TF available, the tag pose is stored in
  `memory_frame` (parameter, default `odom`; `''` disables memory) as a
  rotation and translation, replacing any earlier entry for that id.
- When behaviours are enabled, every configured `approach`/`place` tag that
  is NOT in this frame's detections but IS in memory is turned into a
  *virtual sighting*: its remembered pose mapped through the current TF
  `<camera frame> <- memory_frame` into camera coordinates. Virtual
  sightings feed `TagBehavior.update` exactly like real ones, and
  `~/place` uses the same mapped pose. `stop` tags are never virtual.
- Real sightings always win over memory for the same id.
- **Sightings are steered in a level frame at `base_footprint`, not in the
  camera frame.** The camera is on the wrist: in `CARRY` it points down and
  sideways, so camera x/z say nothing about the robot's heading and a
  controller working in them declares "reached" at a meaningless spot. The
  node maps every pose (real or virtual) through TF
  `base_footprint <- <camera frame>` and hands `TagBehavior` the tuple
  `(-Y, -Z, X)` of the tag's base_footprint position -- the optical
  convention (x right, z forward) `TagBehavior` and its tests already use,
  but measured from the robot's base with the robot's heading as z.
  `standoff` is therefore the planar range from `base_footprint` to the tag,
  independent of the arm pose. If the TF is unavailable, the behaviour
  still runs on an empty frame (so the hold -> one-zero -> release
  sequence for a lost tag still happens instead of leaving the last twist
  standing), it is just never steered in raw camera coordinates.
- Consequence: with memory on, a placed tag never "leaves view", so `place`
  fires once per enable; untick and re-tick `Enable behaviors` to re-arm.
- The GUI status line appends `[remembered]` when the active target is a
  virtual sighting.

This is also the hook for a later combined Nav2 launch: the memorised pose
of station N, in `map`, is where its Nav2 goal comes from.

### Node side (`apriltag_detect.py`)

- Publishes `Twist` on `/controller/cmd_vel` when `Decision.twist` is set.
- On `Decision.place`: the pedestal top in the tag frame
  (`tags.tag_to_pedestal_top()`) is mapped through the solved tag pose into
  the camera frame, then through TF `base_footprint <- <image frame_id>`
  into `base_footprint`; `control.place_height` is added to z; the point is
  sent as `"x y z"` to `/pick_and_place/place` (async client, non-blocking).
  The response message (`placing red` / `not reachable ...` / `not holding
  anything`) is shown in the GUI status line and logged. No retry: the
  user shortens `standoff` and lets the tag be re-acquired.
- The behaviour runs inside `_process_image` on the same frame the
  detections came from, so there is no separate timer and no second copy of
  the sightings.

## GUI (`tune:=true`)

Tk, main thread; `rclpy.spin` on a daemon thread (same split as
`color_detect.main`). Shared state behind the node's lock; the window polls
the latest overlay frame with `after(50)`.

Layout, one window titled `apriltag_detect tune`:

- Left: the overlay image (`_draw` output), scaled to fit 640 px wide.
- Right, three labelled frames:
  1. **Detector** -- one slider + numeric entry per key above, a `none/subpix`
     radio, and the `max_reproj_error_px` slider. Any change takes effect on
     the next frame.
  2. **Tags** -- one row per id: id label, `action` combobox, `standoff`
     entry. Rows come from `behaviors` in the config, plus every id seen
     since start (added live, defaulting to `none`), plus an "add id" entry +
     button. Removing a row is not offered; set it to `none`.
  3. **Control** -- entries for the nine `control` values, the
     `Enable behaviors` checkbox, and a status label showing
     `Decision.status` and the last `~/place` response.
- Bottom: `Save` (write tuned JSON), `Revert` (reload YAML baseline, redraw
  widgets), `Print YAML` (log the current values as a paste-ready block).
- Closing the window stops the GUI only; detection and behaviours continue
  with the last values, like `color_detect`'s `q`.

The window opens in a **simple view**: the overlay image, **Tags**, the
`Enable behaviors` checkbox, the status label, and the bottom buttons.
**Detector** and **Control** sit behind an **Advanced ▸** toggle button,
collapsed by default -- a ten-minute demo only ever needs the first three
and a puzzled afternoon needs the rest.

`rospider_gazebo/tkview.py` provides: `photo_from_bgr(frame) -> PhotoImage`
(PPM encoding, no PIL), `LabeledScale` (slider + entry bound to one
`tk.DoubleVar`/`IntVar`), and `mainloop_until_shutdown(root)`, which runs
`mainloop` and shuts down cleanly on window close or Ctrl-C. There is no
`run_with_spin`: `apriltag_detect.main()` inlines the spinner thread itself
(starts it, calls `node.run_tuner()`, then joins it), the same shape
`color_detect.main` already used.

## Launch

`pick_place.launch.py` passes `tune` to `apriltag_detect` the same way it
passes it to `color_detect` (`ParameterValue(..., value_type=bool)`). With
`tune:=true` and the default `detector:=color tags:=true`, two tuner windows
open; that is intended -- `tags:=false` suppresses the tag one.

`approach`/`place` need the camera level: the docs say to launch with
`arm_pose:=horizontal` for them.

`pick_place.launch.py` also gains `scene:=false`, which skips spawning the
pick pedestal and cubes. The spawn pose puts the skid box 1 cm from the
pedestal, so from there any forward motion rams it and pitches the body;
`scene:=false` is the tag-only demo where `approach`/`stop` can walk from
the spawn pose.

## Testing

- `test/test_tag_behavior.py` (plain pytest, no ROS): nearest-tag choice,
  stop precedence, approach sign conventions and clamping, deadband ->
  `reached`, place fires exactly once, lost-timeout release sequence
  (hold, zero once, then None), disabled returns None.
- `test/test_apriltag.py`: `detector_parameters` maps every key, rounds
  even window sizes up, rejects unknown keys; `detect_tags` with explicit
  stock params still finds the synthetic tag.
- `test/test_tag_behavior.py` also: turn-in-place when the bearing exceeds
  `turn_first_rad` or the tag is behind the camera.
- Manual, in the simulator: `scene:=false tune:=true arm_pose:=horizontal`,
  set `tag0` to `approach` with standoff 0.4, enable, watch the robot walk
  to the station and stop. Then with the scene: see the tag, `~/pick red`,
  drive around the pedestal with teleop, set `place`, and confirm the cube
  lands on the station pedestal or the status line explains why not.
