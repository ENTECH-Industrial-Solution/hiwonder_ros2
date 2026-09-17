# LAB colour tool and demo-window fidelity

Scope: `ROSpider/src/simulations/rospider_gazebo`. Two related changes so
that what a student sees in the simulation is what they will see on the
real ROSpider: the colour detector moves to Hiwonder's LAB threshold
workflow with a window laid out like `LAB_Tool`, and every ported demo
window is brought back to the exact drawing of its `src/example/` original.

## Goal

1. `color_detect.py tune:=true` opens a window that reads like Hiwonder's
   `LAB_Tool 1.0` (wiki 6.1): processed feed left, raw feed right, one row
   each for L, A and B with a min and a max slider, a colour list with
   Add / Delete / Save, and Quit. The detector itself thresholds in LAB
   with a config shaped like the robot's `lab_config.yaml`.
2. Each of the 15 windows started by `vision_demo.launch.py`, plus the
   `image_result` frames of `color_detect` and `apriltag_detect` that they
   display, draws what the corresponding upstream script draws -- same
   shapes, colours, fonts, positions, window titles -- and nothing more.

## Non-goals

- No change to what the nodes publish: `ObjectsInfo` on
  `/yolo/object_detect`, `ApriltagsInfo`, `image_result` topics and the
  `pick_and_place` / `mission` consumers keep their contracts.
- No `Mono` (camera switch), no Chinese/English toggle, no LAB colour
  wheel picture. `LAB_Tool` has them; the simulation has one camera and
  one language.
- No new demo. `track_and_grab` and the arm half of
  `object_classification` stay unported, as before.
- `yolo_detect` and `apriltag_detect` keep their own Tk tuners; they have
  no upstream window to match.

## A. `color_detect.py`: LAB detection and the LAB_Tool window

### Colour space and config

Thresholding moves from HSV to LAB (`cv2.COLOR_BGR2LAB`, OpenCV's 0-255
scale, the same numbers `LAB_Tool` shows). One band per colour; the HSV
red wrap-around and its second band go away. YAML shape follows
`lab_config.yaml`:

```yaml
red:
  min: [0, 150, 100]
  max: [255, 255, 255]
```

`config/color_detect.yaml`, `config/color_detect_arena.yaml` and
`config/color_detect_arena_solved.yaml` are converted by tuning in the
simulator with the new window, not by arithmetic: Gazebo's lighting decides
the rendered LAB values. `color_detect_arena.yaml` keeps its property that
the shipped bands miss the pastel cubes and the teams must widen them.

`min_area_px`, `kernel_px`, `colors`, `tune`, `tuned_path` stay. The tuned
JSON (`~/.ros/color_detect_tuned.json`) takes the new `min`/`max` shape; a
file in the old `lower`/`upper` shape is ignored with a warning naming it,
so a forgotten HSV file cannot silently shadow the LAB config.

`rospider_gazebo/lab_settings.py` (pure Python, tested) owns the settings:
defaults, YAML/JSON overlay with the existing precedence, the old-shape
check, and rendering the current bands as a paste-ready YAML block --
mirroring `tag_settings.py` / `yolo_settings.py`.

### Detection and `image_result`

Detection is per colour: mask in LAB, morphological open/close with
`kernel_px`, contours above `min_area_px`. The published `ObjectsInfo` is
unchanged. The drawn frame follows `color_detect_node.py`:

- new parameter `detect_type`: `circle` (default -- what
  `color_position.py` and `color_recognition_node.py` request upstream)
  draws `cv2.minEnclosingCircle` with thickness 2; `rect` draws the
  `cv2.minAreaRect` box with `drawContours` thickness 2 plus a filled
  5 px centre dot.
- colours from Hiwonder's `range_rgb`: red `(0, 50, 255)`, green
  `(50, 255, 0)`, blue `(255, 50, 0)`; a colour not in that table draws
  in the BGR of its own band midpoint, as today.
- no colour name text on the frame; no FPS.

### The tune window

Tk, built on `tkview.py`, replacing the OpenCV trackbar window. Layout in
the order `LAB_Tool` shows it:

- Top: two image panes side by side, the mask of the selected colour on
  the left, the raw camera frame on the right. Same scale as the other
  tuners (`photo_from_bgr`).
- Below: three rows labelled `L`, `A`, `B`. Each row is
  `min slider | min value | max slider | max value`, range 0-255, sliders
  as `LabeledScale`. Under the rows, the same explanatory sentence
  `LAB_Tool` prints ("LAB is composed of one lightness channel and two
  color channels ...").
- Right column: `Color list` label over a dropdown of the configured
  colours, then `Add`, `Delete`, `Save`; `Quit` at the bottom.
  - `Add` asks for a name (same `NAME_CHARS` rule as today), seeds it
    wide open (`min [0, 0, 0]`, `max [255, 255, 255]`) and selects it.
  - `Delete` removes the selected colour; the three built-in names can be
    deleted too, as in `LAB_Tool`, since the YAML brings them back.
  - `Save` writes the tuned JSON and logs the YAML block for the config
    file. There is no separate print-YAML key.
  - `Quit` closes the window; the node keeps detecting, as today.
- Slider changes apply live to the running detector, so the mask pane,
  `image_result` and `/yolo/object_detect` all reflect them.

Dropped from the current tuner: the `band` slider, `min_area` and
`kernel` sliders (parameters only), the `n/s/r/y` keys.

## B. Demo windows: exactly upstream

Rule for every script in `scripts/` started by `vision_demo.launch.py`,
and for the `image_result` frames of `color_detect` and `apriltag_detect`:

- **Running normally**, the frame carries only the draw calls the
  upstream script makes, with the same primitives, colours, fonts, sizes,
  positions, text and window titles. Upstream draws on an RGB image and
  converts before `imshow`; the simulation draws on BGR, so colour tuples
  are translated, not copied.
- **FPS** appears only where the upstream script calls `show_fps`.
  `VisionDemo` stops drawing it unconditionally; a demo opts in.
- **Simulation-only states** -- waiting for a topic, floor calibration,
  no depth yet, tracker lost, nothing selected -- are logged through the
  node logger (upstream prints to the terminal) and never drawn. The
  `banner()` helper and every `NO TARGET` / `NO TAG` / `TARGET LOST` /
  `RUNNING` / `CALIBRATING` / velocity / pan-tilt / coordinate readout
  the port added go away.
- **Mirroring and sizing** follow upstream: the MediaPipe demos
  `cv2.flip(image, 1)`; `pose_control` shows the frame resized to its
  `display_size`.
- Behaviour, topics and parameters do not change; only what is drawn.

Per-window targets (upstream file -> what the window shows):

| demo | title | frame |
|---|---|---|
| `color_detect` result | -- | circle or rotated box + centre dot in `range_rgb` |
| `color_position` | `image` | detector frame + yellow 5 px dot + `(x, y)` text 0.6 below it |
| `color_recognition` | `image` | detector frame only; recognised colour logged |
| `apriltag_detect` result | -- | 4 corner dots (2 px), solvePnP axes (3 px lines, filled disc), `idN` centred under the tag, 0.6, yellow |
| `apriltag_position` | `image` | detector frame only; id/x/y/w/d logged |
| `apriltag_track` | `image` | detector frame only |
| `ar_view` | `result` | corner dots + cube: filled green base, white edges, red top (or `.obj` model) |
| `kcf_track` | `result` | `(255, 255, 0)` rectangle thickness 2; `selectROI` on `s` |
| `prevent_falling` | `depth_color_map` | JET map (alpha 0.45) + three black 10 px ROI dots |
| `cross_bridge` | `depth_color_map` | JET map + five black ROI dots |
| `object_volume` | `Object Classification (ROI Mode)` | depth + rgb side by side, white / cyan ROI rectangles, red rotated box, white type text 0.7, info lines bottom-left 0.5 |
| `object_classification` | `depth` | white bbox, shape name `FONT_HERSHEY_COMPLEX` 1.0 two-pass at the box centre, ROI rectangle 1 px on rgb |
| `hand_detect` | `hand_detect` | mirrored frame + landmarks |
| `hand_gesture` | `result_image` | mirrored + landmarks + gesture two-pass at (10, 100) 1.2 / direction 1.0 blue |
| `finger_trajectory` | `image` + `track` | mirrored + landmarks; second window with the drawn track, contour, approx polygon and shape name at (10, 40) |
| `face_track` | `result` | mirrored + MediaPipe face drawing + FPS |
| `pose_control` | `image` | mirrored + pose landmarks + the coloured joint dots, resized to `display_size` |

The implementation plan reads each upstream file in full before editing
its port; the table is the checklist, not the source of truth.

## Files

- `rospider_gazebo/lab_settings.py` (new) + `test/test_lab_settings.py`
- `scripts/color_detect.py`: LAB masks, `detect_type`, Tk window
- `config/color_detect*.yaml`: LAB bands
- `rospider_gazebo/vision_demo.py`: FPS opt-in, `banner()` removed
- `scripts/*.py` for the 15 demos and `apriltag_detect.py`: drawing only
- `SIMULATION.md` sections 7 (colour) and 11 (demos), `CLAUDE.md`

## Testing

- `test_lab_settings.py`: defaults, YAML/JSON precedence, old-shape file
  rejected, YAML rendering round-trips.
- Existing tests keep passing (`detections`, `shape_detect`, `pid`, ...).
- In the simulator: tune the three YAMLs with the new window and confirm
  `pick_place.launch.py` still picks all three cubes and
  `mini_game.launch.py detector:=color` still delivers one cube with the
  arena-solved bands.
- Each non-MediaPipe demo run in Gazebo and its window compared with the
  wiki screenshot; MediaPipe demos run against the PC webcam if one is
  present.
