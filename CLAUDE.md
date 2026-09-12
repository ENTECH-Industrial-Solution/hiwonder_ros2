# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Workspace scope

This directory holds Hiwonder robot products, **one colcon workspace per product subdirectory**. Right now there is only `ROSpider/` (an 18-servo hexapod with a camera pan joint, running on Jetson). More products will be added later as sibling directories.

The whole directory is one git repo, `ENTECH-Industrial-Solution/hiwonder_ros2`. `ROSpider/` was imported from Hiwonder's `Jetson_Nano_ROS2` branch with `git subtree`, so the upstream history is kept. To pull Hiwonder updates: `git subtree pull --prefix=ROSpider hiwonder Jetson_Nano_ROS2`, where the remote `hiwonder` is https://github.com/Hiwonder/ROSpider.git.

- Hiwonder products reuse generic package names (`app`, `bringup`, `controller`, `peripherals`, `interfaces`, `sdk`, `example`...) and colcon rejects duplicate names. So products must stay in separate workspaces: run `colcon build` from inside the product directory, never from this root. Never source two products' `install/` in the same shell.
- Don't treat ROSpider-specific details (servo IDs, leg geometry, `rospider_description`) as applying to other products.

Everything below is about ROSpider. Paths are relative to `ROSpider/`.

`README.md` / `README_cn.md` are the upstream Hiwonder README. Official docs: https://docs.hiwonder.com/projects/ROSpider/en/jetson-orin-nano-version/

## Target platform vs. this dev machine

The code targets **Ubuntu 22.04 + ROS 2 Humble + Python 3.10 on a Jetson (aarch64)**, running the official ROSpider system image. The image's user is `ubuntu`, the workspace lives at `/home/ubuntu/ros2_ws`, and `~/.robotrc` sets up the environment.

This machine is different: ROS 2 **Jazzy**, Python 3.12, x86_64, no `~/.robotrc`, and the workspace is at `~/entech_hiwonder_ros2_ws`. So:

- Prebuilt binaries are **aarch64-only**: `driver/kinematics/kinematics/kinematics.so` and `driver/arm_kinematics/arm_kinematics/{forward,inverse}_kinematics.so`. They are Python extension modules with no source, so gait/IK code can't be imported or run here. The build still succeeds, because Python packages aren't imported at build time.
- `xf_mic_asr_offline` links the prebuilt iFlytek libs from `lib/arm64` or `lib/x64`, chosen by `CMAKE_SYSTEM_PROCESSOR`. This is a local patch; upstream hardcoded arm64, which broke the x86 build. The libs are not installed and the binary gets no rpath, so running `voice_control` needs that lib directory on `LD_LIBRARY_PATH` (it also needs `libmsc.so`).
- There are **~150 hardcoded `/home/ubuntu/...` paths**. Most are `/home/ubuntu/ros2_ws/src/...`; others point to image-only tools (`/home/ubuntu/software/actionset_editor/ActionGroups`, `/home/ubuntu/software/lab_tool`, `/home/ubuntu/third_party/...`).
- On a real robot, hardware is reached through udev symlinks: `/dev/rrc` is the STM32 controller board at 1 Mbaud (`ros_robot_controller_sdk.py`), and `/dev/lidar` is the LiDAR.

## Required environment variables

`~/.robotrc` normally sets these. Launch files and nodes read them with `os.environ[...]`, so a missing variable raises `KeyError` at launch time:

- `need_compile`: `True` makes launch files find other packages via `get_package_share_directory` (the installed/colcon build). Any other value makes them use hardcoded source paths under `/home/ubuntu/ros2_ws/src`. **Set `need_compile=True` when working in this workspace.**
- `ASR_LANGUAGE` (`Chinese` / `English`), `ASR_MODE` (`offline` / online), `MIC_TYPE`: used by voice and large-model packages.
- `DEPTH_CAMERA_TYPE`, `MACHINE_TYPE`: select the camera/robot variant in peripherals and examples.
- `MASTER`, `HOST`: used by multi-robot examples.

## Build and test

Run these from `ROSpider/` after `source /opt/ros/<distro>/setup.bash`:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source install/local_setup.bash
colcon build --packages-select <pkg> --symlink-install      # one package
```

Message packages must build before the Python packages that import them: `interfaces`, `*_msgs` under `driver/`, `large_models/large_models_msgs`, and `xf_mic_asr_offline_msgs`. colcon handles this ordering through `package.xml` deps.

The only tests are the stock ament lint tests (`test_flake8.py`, `test_pep257.py`, `test_copyright.py`) in each `ament_python` package. There are no functional tests.

```bash
colcon test --packages-select <pkg> && colcon test-result --verbose
colcon test --packages-select <pkg> --pytest-args -k flake8   # single test
```

Python packages install launch/config files through `data_files` globs in `setup.py`. A new launch file, config file, or node entry point must match those globs (or be added to them) and to `console_scripts`, or it won't be installed.

## Running (on the robot)

```bash
ros2 launch bringup bringup.launch.py          # full stack (also what start_app_node.service runs)
ros2 launch controller controller.launch.py    # motion stack only
ros2 launch app start_app.launch.py
ros2 launch slam slam.launch.py slam_method:=slam_toolbox
ros2 launch navigation navigation.launch.py map:=map_01
```

## Architecture

**Launch composition.** Launch files use `OpaqueFunction(launch_setup)` and `IncludeLaunchDescription` with the `need_compile` path switch described above. `bringup.launch.py` includes: `controller` → `peripherals` depth camera + LiDAR → rosbridge websocket + `web_video_server` (for the Hiwonder mobile app) → `app/start_app.launch.py` → joystick → `init_pose`, plus the `bringup/startup_check` node. The "app" nodes (line following, object tracking, gesture, self-balancing, kick, LiDAR behaviors) all start idle and are switched on by service calls from the phone app over rosbridge.

**Motion pipeline** (all in `src/driver/`), top to bottom:

1. `controller/move_controller.py` is the public motion API. It subscribes to `~/cmd_vel` (Twist), `~/traveling` (gait params), `~/pose_transform_euler`, `~/set_pose_euler`, `~/set_leg_absolute|relatively`, and `~/run_actionset`. It also runs action groups via `servo_controller.action_group_controller` from the ActionGroups directory.
2. `controller/step_controller.py` does the gait loop and leg placement, using the binary `kinematics.so` through the `kinematics` package. It publishes `ServosPosition` on `servo_controller`.
3. `servo_controller/controller_manager.py` maps joint names (`coxa_/femur_/tibla_{LF,LM,LR,RF,RM,RR}_joint`) to servo IDs and pulse ranges from `config/servo_controller.yaml`, and publishes `joint_states`.
4. `ros_robot_controller` publishes on `ros_robot_controller/bus_servo/set_position`. It is the only node that talks to the board over serial (`/dev/rrc`), and it also handles IMU, buttons, buzzer, RGB, and OLED.

**Odometry.** `odom_publisher`/`move_controller` publish `odom/raw`, which is fused with the filtered IMU by `robot_localization` EKF (`controller/config/ekf.yaml`) into `odom`. `rf2o_laser_odometry` is an alternative. Namespaces for multi-robot use come from the `namespace`/`use_namespace` launch args, applied to `ekf.yaml` with `ReplaceString('namespace/')`.

**Shared interfaces.** Cross-package messages and services (`ObjectsInfo`, `ColorsInfo`, `RunActionSet`, `SetPose2D`, ...) live in `interfaces`. `interfaces/mapping_rules.yaml` maps them to ROS 1 `hiwonder_interfaces` for ros1_bridge. Motion-specific messages are in `kinematics_msgs` and `servo_controller_msgs`; board-level ones are in `ros_robot_controller_msgs`.

**Voice and large models.**
- `xf_mic_asr_offline` wraps the iFlytek (xf) offline ASR SDK (C++ node + prebuilt `libmsc`). Directories named `msc/` hold iFlytek appid-specific resources.
- `large_models/large_models/config.py` picks the backend from `ASR_LANGUAGE`: Chinese uses Aliyun DashScope / StepFun, anything else uses OpenAI-compatible endpoints. Offline mode uses Ollama + sherpa-onnx under `~/third_party`. API keys are empty-string placeholders in that file.

**Applications and examples.** `app`, `example`, `competition`, and `large_models_examples` are consumers of the stack above. They publish to the controller topics and never talk to hardware directly. Code comments are often in Chinese.

## Simulation (PC, no hardware)

The user-facing how-to, in Thai, is `ROSpider/SIMULATION.md`. Entry points:
- `ros2 launch rospider_gazebo {gazebo,slam,rtabmap_slam,vslam,navigation,rtabmap_navigation,moveit,pick_place}.launch.py`
- URDF viewer: `rospider_description display.launch.py`, which needs `need_compile=True`
- MoveIt on mock hardware: `robot_moveit_config demo.launch.py`

`src/simulations/rospider_gazebo` is an Entech addition, not part of upstream. Several choices in it look odd but are deliberate:
- **Walking is kinematic.**
  - `scripts/sim_gait.py` stands in for the real `move_controller`/`kinematics.so`. It subscribes to `/controller/cmd_vel`, runs a tripod gait on `leg_controller` (numeric IK on the URDF leg chains; all leg joints at 0 is the standing pose), and publishes the body twist on `/sim/base_cmd_vel`. The twist is scaled down when the stride can't keep up (~0.23 m/s).
  - The gz VelocityControl system moves the body with that twist; the feet don't push it. Stance feet are moved opposite to the body so they stay planted.
  - OdometryPublisher provides `/odom` and TF `odom→base_footprint`.
- **Changes `gazebo.launch.py` makes to the URDF** (Hiwonder's URDF files stay untouched):
  - It raises the leg joints' velocity limit from the URDF's placeholder 1 rad/s to 5 rad/s. Gazebo enforces the limit, and at 1 rad/s the legs lag the gait.
  - It strips every mesh `<collision>` from the URDF. The STL meshes total ~1.5M triangles and slowed the sim to RTF 0.02.
  - A frictionless box, `sim_skid_link`, carries the body instead. Without it, VelocityControl lets the body sink by g·dt every physics step.
  - The LiDAR ignores the robot. Legs and arm cross its scan plane, and Gazebo clamps too-close hits to `range_min` instead of dropping them. So `gazebo.launch.py` gives every robot visual `visibility_flags` 2, and the LiDAR's `visibility_mask` excludes that bit.
- **Topic names match the real robot:** `/scan`, `/imu`, `/odom`, `/depth_cam/...` and `/controller/cmd_vel`, mapped in `config/gz_bridge.yaml`.
  - Everything that moves the robot, Nav2 included (via `collision_monitor.cmd_vel_out_topic`), publishes `/controller/cmd_vel`, as on the real robot. Only `sim_gait`'s `/sim/base_cmd_vel` is bridged, since gz-transport allows one bridge entry per Gazebo topic.
  - Camera topics are bridged with `lazy: true`, so they're only converted while something subscribes. The point cloud alone took ~40% of a CPU core and slowed the sim. Because the sensor only renders while something subscribes, the first RGB frame a new subscriber gets is black.
  - RViz starts together with SLAM/Nav2 after the 5 s delay. If it starts before the sim clock runs, it logs "earlier than all the data in the transform cache".
- **SLAM/Nav2 plumbing.** SLAM and Nav2 use Jazzy's own `slam_toolbox/online_sync_launch.py` and `nav2_bringup`, loaded with Hiwonder's parameters. The upstream `slam`/`navigation` launch files can't be used:
  - They are Humble-specific: slam_toolbox started as a non-lifecycle node, plugin names written with `/`, the TEB planner, and controller params without `use_sim_time`.
  - Their `sim:=true` still starts the hardware drivers.
  - `slam_toolbox` is brought up by a `nav2_lifecycle_manager`, not by `online_sync_launch.py`'s own autostart. The autostart sometimes loses the configure response, and then the node never activates and never publishes `/map`.
- **V-SLAM (`rtabmap_slam.launch.py`, `vslam.launch.py`).**
  - `vslam.launch.py` is camera-only and self-contained.
    - It uses only its own files: `config/vslam.yaml`, `config/vslam_nav2_params.yaml`, `rviz/vslam.rviz`. Maps go in `maps/vslam/`.
    - Nothing in it subscribes to `/scan`.
    - Its RTAB-Map values are Hiwonder's minus the LiDAR: `subscribe_scan` false, `Reg/Strategy` 0 (visual), `Grid/Sensor` 1 (depth).
    - It also sets `map_always_update` true. Otherwise `/map` is only published when a node is added, and `map_saver` times out once the robot stands still.
  - `localization:=true` loads the 3D map.
    - It sets `Mem/IncrementalMemory` false and `Mem/InitWMWithAllNodes` true, and never passes `-d`, since `-d` deletes the database.
    - It adds nav2_bringup's `navigation_launch.py` with `vslam_nav2_params.yaml`. Every obstacle source there is `/vslam/obstacle_cloud`: a ~2k-point cloud from `rtabmap_util/point_cloud_xyz` (decimation 4, 5 cm voxels, 0.2–3 m).
    - The gz camera cloud (307k points at 15 Hz) was too heavy. With both costmaps and the collision monitor reading it, the planner stopped acknowledging goals in time, and goals aborted.
    - The collision monitor uses `base_shift_correction: false`, because the camera's TF comes from the arm's joint_states and lags the cloud.
    - Camera-only maps store no scans, so use this mode rather than `rtabmap_navigation.launch.py`, which uses ICP registration.
  - `rtabmap_slam.launch.py` uses Hiwonder's `slam/launch/include/rtabmap.launch.py` unchanged. That file works on Jazzy, and its topic names already match the sim. It starts `rtabmap` with `-d`, which recreates the database on every run.
  - `rtabmap_navigation.launch.py` reuses a saved map. It combines Hiwonder's `navigation/launch/include/rtabmap.launch.py` (localization mode, `Mem/IncrementalMemory` false: adds no new nodes, though `rtabmap` still writes to the `.db` on shutdown) with nav2_bringup's `navigation_launch.py`, without map_server/AMCL, since RTAB-Map publishes `/map` and `map→odom`.
  - **Maps live in `ROSpider/maps/`.** Every launch that saves or loads a map takes `map:=<name>`, or a path containing `/`.
    - `rospider_gazebo/maps.py` (installed with `ament_python_install_package`) resolves a name to `<workspace>/maps/<name>.db` or `.yaml`. It finds the workspace from the package's install prefix. `vslam.launch.py` uses the subfolder `maps/vslam/`.
    - Hiwonder's includes hardcode `~/.ros/rtabmap.db`, so the resolved path is injected with `SetParameter` in a `GroupAction` around the include.
    - `.db` files are git-ignored: they're tens to hundreds of MB, and GitHub rejects files over 100 MB. 2D maps are small and can be committed.
    - Don't copy a `.db` while `rtabmap` is running; the copy comes out malformed.
  - The camera sits on the arm. `gazebo.launch.py arm_pose:=horizontal` sets `joint4` to -0.286 so the camera is level, like the real robot's `init_horizontal` action. The default `init` pose tilts the camera 52° down at the floor.
  - The world's walls are 1 m high and carry procedural poster textures (`worlds/textures/`). Without them, visual loop closure finds nothing to match.
- **Nav2 params and map.**
  - `config/nav2_params.yaml` is Jazzy's defaults plus Hiwonder's values: DWB, `robot_radius` 0.01, max 0.05 m/s.
  - It deviates from Hiwonder's DWB values in two places. `FollowPath.xy_goal_tolerance` 0.15 would stall the robot 0.05–0.15 m short of the goal, because the goal checker wants 0.05. `sim_time` 10 made DWB crawl near the goal until the progress checker aborted. The real robot's `navigation/config` probably has the same issues (not tested on hardware).
  - `maps/rospider_room.*` was built in the sim starting from the spawn pose, and AMCL's initial pose assumes that same pose.
- **Controllers and MoveIt.**
  - The controller names match `servo_controller` and MoveIt.
  - The SRDF names its virtual-joint parent frame `world_feame` (misspelled), so `moveit.launch.py` publishes a static TF `world_feame→odom`.
- **Headless mode.** `gui:=false` needs working EGL. On the dev PC, EGL tries the NVIDIA card first and fails while the NVIDIA driver is mismatched.
- **Pick and place (`pick_place.launch.py`).** Not MoveIt: `robot_moveit_config` sets `position_only_ik: true`, which can't constrain grasp orientation, so the pick path uses its own closed-form IK instead. Grasping is a `DetachableJoint` weld on `link5`, not friction — and Gazebo forms that weld the instant the cube spawns, before any node commands it, so the node must publish a detach at startup or the first arm move drags the whole scene. The pedestal and cubes are spawned at run time rather than added to `rospider_room.sdf`, so that world file keeps backing `maps/rospider_room.*` and Nav2's AMCL initial pose. The three drop slots sit in a row, not stacked: at stacking height the arm has no steep-approach solution left, and the only shallow ones sweep the held cube through the cubes already placed.

Local patches to upstream packages, all needed for Jazzy/x86:
- `xf_mic_asr_offline/CMakeLists.txt`: picks the lib folder by CPU architecture (see above).
- `robot_moveit_config`:
  - installs `rviz/`
  - declares `trac_ik_kinematics_plugin` as a dependency
  - sets `max_acceleration: 1.0` in `joint_limits.yaml`. That is Humble's implicit default; Jazzy's TOTG aborts every plan without an explicit value.
