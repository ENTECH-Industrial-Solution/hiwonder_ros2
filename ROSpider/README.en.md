# ROSpider Simulation

[ไทย](README.md) | English

## 1. Overview

**ROSpider** is Hiwonder's 6-legged spider robot. It has a robot arm with a depth camera at the end, and a LiDAR.

This document covers the **PC simulation** (package `rospider_gazebo`) that Entech added. It is used to teach workshops without a real robot.

- Runs on Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic
- Topic names match the real robot (`/controller/cmd_vel`, `/scan`, `/odom`, `/depth_cam/...`)
- Walking in the sim is simulated: the legs step for show, but the body is moved straight from `cmd_vel`, because Hiwonder's walking code (`kinematics.so`) is an ARM binary that cannot run on a PC

### Install and build (first time)

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Run these three lines first in every terminal you use:

```bash
source /opt/ros/jazzy/setup.bash
source ~/entech_hiwonder_ros2_ws/ROSpider/install/local_setup.bash
export need_compile=True
```

Drive the robot with the keyboard (works with every topic):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/controller/cmd_vel
```

## 2. File layout

```
ROSpider/                  colcon workspace (build from inside this folder)
├── README.md              simulation guide (Thai)
├── README.en.md           ← this file
├── REAL_ROBOT.md          real-robot station guide, afternoon (Thai; English: REAL_ROBOT.en.md)
├── maps/                  maps you make (.yaml/.pgm = 2D, .db = 3D)
└── src/
    ├── driver/ app/ example/ slam/ navigation/ ...   Hiwonder's code (real robot)
    └── simulations/
        ├── rospider_description/   URDF / robot model
        ├── robot_moveit_config/    MoveIt settings for the arm
        └── rospider_gazebo/        ← all of the simulation is here
            ├── launch/             files for ros2 launch (one per topic)
            ├── scripts/            nodes (gait, colour/AprilTag detection, grasping, demos)
            ├── rospider_gazebo/    shared Python code (IK, PID, ...)
            ├── config/             tuning values (.yaml): colour ranges, Nav2, RTAB-Map
            ├── worlds/ models/     Gazebo scenes and objects (cubes, pedestals, AprilTag boards)
            ├── maps/               ready-made maps of the sim rooms
            ├── urdf/ rviz/         the sim's URDF parts and RViz screens
            ├── test/               unit tests
            └── tools/              helper scripts (YOLO dataset/training, textures)
```

## 3. Workshop topics

| Topic | Main command |
|---|---|
| MoveIt2 Simulation | `robot_moveit_config demo.launch.py` |
| Gazebo Simulation | `rospider_gazebo gazebo.launch.py` |
| SLAM Mapping | `rospider_gazebo slam.launch.py` |
| RTAB-VSLAM 3D Mapping | `rospider_gazebo vslam.launch.py` |
| Color Threshold Adjustment | `rospider_gazebo lab_tool.launch.py` |
| Color Tracking | `rospider_gazebo object_tracking.launch.py` |
| AprilTag Tag Tracking | `rospider_gazebo apriltag_track.launch.py` |
| Autonomous Line Following | `rospider_gazebo line_following.launch.py` |
| 3D Vision: Object Grasping | `rospider_gazebo track_and_grab.launch.py` |
| 3D Vision: Shape Recognition | `rospider_gazebo object_classification.launch.py` |

### MoveIt2 Simulation

Plan arm motions with MoveIt in RViz: pick the planning group `arm` or `gripper`, drag the marker to the pose you want, then press **Plan & Execute**.

```bash
ros2 launch robot_moveit_config demo.launch.py      # simulated arm, no Gazebo
ros2 launch rospider_gazebo moveit.launch.py        # arm in Gazebo (the camera on the arm moves with it)
```

### Gazebo Simulation

Opens the robot in a simulated room with all its sensors (LiDAR, depth camera, IMU, odometry); drive it with teleop.

```bash
ros2 launch rospider_gazebo gazebo.launch.py
```

Common options:
- `arm_pose:=horizontal` makes the camera look straight ahead
- `world:=<name>` changes the room; a file name in `worlds/` is enough, e.g. `world:=slam_challenge` (works with every launch that opens Gazebo)
- `gui:=false` doesn't open the Gazebo window

View the camera with `ros2 run rqt_image_view rqt_image_view`.

### SLAM Mapping

Make a 2D map with the LiDAR (slam_toolbox):

1. Start SLAM:
   ```bash
   ros2 launch rospider_gazebo slam.launch.py
   ```
2. Drive the robot around the whole room with teleop
3. Save the map, running from the `ROSpider` folder:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f maps/room1 --ros-args -p use_sim_time:=true
   ```

To navigate with this map, run `ros2 launch rospider_gazebo navigation.launch.py map:=room1` and press **2D Goal Pose** in RViz (`map:=` looks in `ROSpider/maps/` first, then in `maps/` of the folder you run the command from).

**Exercise: fix the settings to pass the levels** (10–15 min)

The exercise has two rooms joined by a doorway. 3 of the 6 SLAM settings provided are wrong. Find them and fix them until all 3 levels pass.

```bash
ros2 launch rospider_gazebo slam_challenge.launch.py
```

1. Edit the exercise file `src/simulations/rospider_gazebo/config/slam_challenge.yaml` (the launch prints its path every time), then close and reopen the launch. The build must use `--symlink-install`, otherwise every edit needs a new `colcon build`
2. Drive through both rooms, then save the map: `ros2 run nav2_map_server map_saver_cli -f maps/<name> --ros-args -p use_sim_time:=true`
3. Check: `ros2 run rospider_gazebo check_slam.py <name>` reports each level with hints

Levels: 1 there is a map · 2 coverage ≥ 90% · 3 the map is detailed (doorway open, walls no thicker than real). To start over: `git checkout -- src/simulations/rospider_gazebo/config/slam_challenge.yaml`

Navigate the exercise room with your saved map: `ros2 launch rospider_gazebo navigation.launch.py world:=slam_challenge map:=<name>`

**Nav2 exercise: make the robot finish the course** (10–15 min)

Same exercise room, with the reference map. 3 of the 6 Nav2 settings are wrong. Fix them until the robot reaches all 3 points (corner of room A → through the doorway and corridor, around the cylinder, to the top-left corner of room B → back to the start) within 300 seconds.

```bash
ros2 launch rospider_gazebo nav_challenge.launch.py      # map:=<name> to use your own map
ros2 run rospider_gazebo check_nav.py                    # the checker sends the goals itself and reports each level
```

Edit `src/simulations/rospider_gazebo/config/nav_challenge.yaml`, then close and reopen the launch before every check. Levels: 1 reach the first goal · 2 through the doorway into room B · 3 finish the course in time (some wrong values only show up once others are fixed; all 3 must be fixed to pass everything)

### RTAB-VSLAM 3D Mapping

Make a 3D map with RTAB-Map.

```bash
ros2 launch rospider_gazebo vslam.launch.py map:=room_vslam           # camera only
ros2 launch rospider_gazebo rtabmap_slam.launch.py map:=room1         # camera + LiDAR
```

1. Drive the robot around the whole room with teleop
2. Press **Ctrl+C** and wait for `Saving database/long-term memory...done!`. The map is saved as a `.db` in `ROSpider/maps/` (V-SLAM's go in `maps/vslam/`)
3. Load the map back to navigate:
   ```bash
   ros2 launch rospider_gazebo vslam.launch.py localization:=true map:=room_vslam
   ```

Caution: running mapping mode with an existing name overwrites that file (navigation mode works on a copy and leaves the original untouched).

**V-SLAM exercise: build a map with the camera** (15–20 min)

Same exercise room (each wall face has its own unique pattern so the camera can recognise places). 3 of the 5 values in the exercise file are wrong.

```bash
ros2 launch rospider_gazebo vslam_challenge.launch.py map:=myvslam
```

1. Edit `src/simulations/rospider_gazebo/config/vslam_challenge.yaml`, then close and reopen the launch
2. Drive through both rooms with teleop, then come back to the start and face the original direction (the robot must see the same view again for a loop closure to happen)
3. Press **Ctrl+C** and wait until it has shut down; the map is saved now
4. Check: `ros2 run rospider_gazebo check_vslam.py myvslam`

Levels: 1 there is a map with walls · 2 coverage ≥ 90% · 3 the map is not bent (recognises the start, no false matches, no doubled walls)

**V-SLAM navigation exercise** (10 min): uses your own map from the exercise above. 2 of the 6 settings are wrong. Same course and checker as the Nav2 exercise.

```bash
ros2 launch rospider_gazebo vslam_nav_challenge.launch.py map:=myvslam
ros2 run rospider_gazebo check_nav.py
```

Edit `src/simulations/rospider_gazebo/config/vslam_nav_challenge.yaml`. The robot must start from the same spot where mapping started (the spawn point).

Instructors: the reference map is made with `vslam_challenge.launch.py map:=vslam_answer params:=<path of config/vslam_challenge_solved.yaml>`, then `python3 src/simulations/rospider_gazebo/tools/drive_route.py` (drives the same route by itself, about 5 minutes). The navigation exercise's answer file is `config/vslam_nav_challenge_solved.yaml`.

### Color Threshold Adjustment

Tune colour ranges in LAB space with the LAB_Tool window, the same one used on the real robot.

```bash
ros2 launch rospider_gazebo depth_camera.launch.py   # terminal 1: start the camera (Gazebo)
ros2 launch rospider_gazebo lab_tool.launch.py       # terminal 2: open the tuning window
```

1. Pick a colour in `Color list`
2. Move the `L` `A` `B` sliders until the mask shows only the object
3. Press **Save**. The values are stored in `~/.ros/color_detect_tuned.json`, and the terminal prints a YAML block to paste into `config/color_detect.yaml`

**Exercise: tune the colours to find only the right cubes** (10 min)

The scene has red, green and blue cubes, a second blue cube in shadow, and an orange cube that looks like red. All 3 colour ranges provided are wrong.

```bash
ros2 launch rospider_gazebo color_challenge.launch.py   # scene + LAB_Tool window
ros2 run rospider_gazebo check_color.py                 # checks the camera image, reports each level
```

Tune in LAB_Tool and run the checker without closing the launch (it uses whatever the window shows at that moment). Don't forget to press **Save** (stored in `~/.ros/color_challenge_tuned.json`, over the exercise file `config/color_challenge.yaml`), or the values are lost when the launch closes. Don't drive the robot during the exercise: the checker refuses if the camera doesn't see the scene from the start pose. Levels: 1 every colour found · 2 the decoy not caught · 3 found even in shadow. To start over: delete `~/.ros/color_challenge_tuned.json`, then close and reopen the launch

### Color Tracking

Click an object in the image and the robot walks after that colour blob.

```bash
ros2 launch rospider_gazebo gazebo.launch.py            # terminal 1
ros2 launch rospider_gazebo object_tracking.launch.py   # terminal 2
```

Left-click an object in the `image` window. The robot takes the colour under the mouse and walks so the object stays on the yellow dot. If the colour range catches too much or too little, adjust it with `threshold:=0.3`.

**Exercise: walk up to the red ball and stop** (10 min)

The scene has a red ball (go to it) and an orange ball (don't). The robot must stop in front of the red ball at 0.43–0.52 m within 40 seconds. 3 of the 4 settings are wrong.

```bash
ros2 launch rospider_gazebo track_challenge.launch.py   # scene + image window (the robot doesn't walk yet)
ros2 run rospider_gazebo check_track.py                 # the checker clicks the red ball's colour and starts the walk itself
```

Edit `src/simulations/rospider_gazebo/config/track_challenge.yaml`, then close and reopen the launch before every check (click in the window to see what the colour catches; the robot won't walk until the checker tells it to). Levels: 1 walk to the red ball · 2 stop right in front of it · 3 in time

### AprilTag Tag Tracking

The robot walks up to the chosen AprilTag and stops at the set distance. The sim room already has tag 1 on the wall.

```bash
ros2 launch rospider_gazebo gazebo.launch.py                                  # terminal 1
ros2 launch rospider_gazebo apriltag_track.launch.py target_tag:=1 stop_distance:=0.35   # terminal 2
```

**Exercise: walk up to tag 2 and stop** (10 min)

The scene has 3 tags. The robot must stop in front of tag 2, within reach of the arm for picking from a pedestal (camera about 0.30–0.40 m from the tag), within 40 seconds. 3 of the 4 settings are wrong.

```bash
ros2 launch rospider_gazebo apriltag_challenge.launch.py   # scene + image window (the robot doesn't walk yet)
ros2 run rospider_gazebo check_tag.py                      # the checker starts the walk itself and reports each level
```

Edit `src/simulations/rospider_gazebo/config/apriltag_challenge.yaml`, then close and reopen the launch before every check. Levels: 1 go to tag 2 · 2 stop at the right distance · 3 in time

### Autonomous Line Following

The robot follows a line on the floor. This launch opens Gazebo itself, in a scene with a black loop, with the robot spawned right on the line.

```bash
ros2 launch rospider_gazebo line_following.launch.py
```

Left-click the line in the `image` window and the robot starts walking. If an obstacle is closer than 0.4 m, it stops and waits. One lap takes about 2 minutes.

**Exercise: make the robot finish a lap in time** (10 min)

2 of the line follower's 4 settings are wrong. Fix them until the robot finishes a lap within 200 seconds.

```bash
ros2 launch rospider_gazebo line_challenge.launch.py   # line scene + image window
ros2 run rospider_gazebo check_line.py                 # the checker picks the line colour and starts the lap itself
```

Edit `src/simulations/rospider_gazebo/config/line_challenge.yaml`, then close and reopen the launch before every check (the robot must start at its spawn point on the line). Levels: 1 follow the line for 1/4 lap · 2 a full lap · 3 the lap in time

### 3D Vision: Object Grasping

The robot tracks a colour block with the camera on its arm, then picks it up. This launch opens everything itself (Gazebo, pedestals, cubes and the pick node).

```bash
ros2 launch rospider_gazebo track_and_grab.launch.py
```

In the control window, press a colour button (red / green / blue) to order a pick. Window options:
- **pick here**: pick the block straight ahead (the green one)
- **walk to it**: walk to the block of that colour first, then pick (the red and blue blocks sit to the sides)
- **place: auto**: put it down beside the robot as soon as it's picked
- **place: by button**: hold it until **Place** is pressed

**Exercise: put the blue block on the yellow pad** (15 min)

The blue block sits on a pedestal to the robot's right, out of the arm's reach. The robot must walk over, pick it up and put it down on the yellow pad on the floor. All 3 settings are wrong.

```bash
ros2 launch rospider_gazebo grasp_challenge.launch.py   # scene + track_and_grab window
ros2 run rospider_gazebo check_grasp.py                 # the checker orders the blue block picked (about 1 min)
```

Edit `src/simulations/rospider_gazebo/config/grasp_challenge.yaml`, then close and reopen the launch before every check (the robot and block must be in their original places). Levels: 1 walk up to the block · 2 pick it up · 3 put it on the yellow pad

### 3D Vision: Shape Recognition

Uses the depth image to tell objects' shapes apart (sphere, cylinder, box) and report their colour. In the sim it only detects and reports; it doesn't sort them into trays.

```bash
ros2 launch rospider_gazebo pick_place.launch.py auto_start:=false    # terminal 1: scene with objects
ros2 launch rospider_gazebo object_classification.launch.py           # terminal 2
```

If nothing is detected, measure the floor distance once with `debug:=true` and put the logged value into `plane_distance:=...` (in millimetres).

**Exercise: make the robot find the ball** (10 min)

On the floor under the camera are a ball (sphere), a box and a cylinder. The robot must separate the objects from the floor, see all 3 and name their shapes correctly, then choose the ball (red frame in the `depth` window). All 3 settings are wrong.

```bash
ros2 launch rospider_gazebo shape_challenge.launch.py   # scene + depth window
ros2 run rospider_gazebo check_shape.py                 # checks what the robot reports, level by level
```

Edit `src/simulations/rospider_gazebo/config/shape_challenge.yaml`, then close and reopen the launch before every check (the robot doesn't need to walk). Levels: 1 objects separated from the floor · 2 all seen and named correctly · 3 the ball chosen

## Final mission (30 min)

Several topics in one mission: Nav2 takes the robot into room B, it follows an AprilTag to stop in front of the right pedestal, picks a block with the depth camera, then brings it back to the yellow pad in room A. Nav2 uses the `config/nav_challenge.yaml` you fixed in the Nav2 exercise, so **pass the Nav2 exercise first**; the other systems are already set up. What you must do is write the right sequence of steps in the mission file (the one provided was drafted by a teammate and has mistakes).

```bash
ros2 launch rospider_gazebo mission_challenge.launch.py   # scene + Nav2 + RViz + track_and_grab window
ros2 run rospider_gazebo check_mission.py                 # runs the mission file step by step and reports each level
```

Edit `src/simulations/rospider_gazebo/config/mission_challenge.yaml` (the task and the available commands are at the top of the file) and check right away. Before each new check, close and reopen the launch so the robot and block return to their places. One check takes about 6–7 minutes. To find coordinates on the map: in RViz pick the **Publish Point** tool and click the map; the coordinates appear in a terminal running `ros2 topic echo /clicked_point`. Levels: 1 reach room B · 2 holding the blue block · 3 the blue block on the yellow pad

Passed and want more of a challenge: use your own map from the SLAM exercise with `mission_challenge.launch.py map:=<map name>`.

Trying it on the real robot in the afternoon: see [REAL_ROBOT.en.md](REAL_ROBOT.en.md) (the instructors' guide to the real-robot station: exercises, pass criteria and the score sheet).

## Common problems

- **The launch fails with `KeyError: 'need_compile'`**: you forgot `export need_compile=True`
- **Gazebo closes at once, or shows `eglInitialize failed`**: a GPU driver problem; try rebooting
- **The robot moves very slowly**: look at the real time factor at the bottom right of Gazebo. Below ~0.5 means the machine can't keep up
- **Nodes can't find each other after many runs**: close everything, then run `rm -f /dev/shm/fastrtps_*`
- **Several people on the same network**: each machine sets `export ROS_DOMAIN_ID=<a unique number>`
