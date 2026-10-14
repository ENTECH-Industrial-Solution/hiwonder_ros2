# Entech × Hiwonder ROS 2

[ไทย](README.md) | English

ROS 2 code for the Hiwonder robots that Entech teaches with and builds on. There is **one folder per robot model**, and each folder is its own colcon workspace. More models will be added over time.

<p align="center">
  <img src="ROSpider/sources/01.png" alt="ROSpider" width="480"/>
</p>

## Robots in this repo

| Model | Robot | Real robot | Simulation (PC) | Docs |
|---|---|---|---|---|
| [ROSpider](ROSpider/) | 6-legged spider robot (18 servos) + robot arm, depth camera, LiDAR | Jetson, Ubuntu 22.04, ROS 2 Humble | Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic | [README](ROSpider/README.en.md) (sim + workshop exercises), [REAL_ROBOT](ROSpider/REAL_ROBOT.en.md) (real robot), [Hiwonder docs](https://docs.hiwonder.com/projects/ROSpider/en/jetson-orin-nano-version/) |

## Layout

```
hiwonder_ros2/
├── README.md          Thai version of this file
├── README.en.md       ← this file
├── CLAUDE.md          technical notes for developers
├── docs/              specs and plans for the features we added
└── ROSpider/          ROSpider's colcon workspace
```

## Important rules

- **Build only from inside a model's folder.** Never run `colcon build` in this folder: Hiwonder's models reuse package names (`app`, `bringup`, `controller`, ...) and colcon rejects duplicates.
- **Never source two models' `install/` in the same terminal.**
- Each model's code is pulled from Hiwonder's repo with `git subtree` (its history is kept). Entech's additions live in this repo.

## Install and build (first time)

Install ROS 2 Jazzy (`ros-jazzy-desktop`) first, following the [ROS guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

```bash
sudo apt install git python3-rosdep python3-colcon-common-extensions
sudo rosdep init   # once per machine; an "already exists" error is fine
rosdep update
cd ~
git clone https://github.com/ENTECH-Industrial-Solution/hiwonder_ros2.git
cd ~/hiwonder_ros2/ROSpider
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
echo "source ~/hiwonder_ros2/ROSpider/env.bash" >> ~/.bashrc
```

The last line makes every new terminal load [env.bash](ROSpider/env.bash) by itself (ROS 2, this workspace and the settings the sim needs). Close and reopen terminals that were already open.

env.bash sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` and `GZ_IP=127.0.0.1`, which keep ROS and Gazebo traffic on your own machine. Without them, every sim on the same network sees the others' topics (`/clock`, `/controller/cmd_vel`, ...), the sim time jumps back and forth, and the robot ignores teleop.

## Uninstall

Remove this workspace's lines from `~/.bashrc`, delete the repo folder (builds included), and delete the values the tuning windows saved:

```bash
sed -i -e '\|hiwonder_ros2/ROSpider/|d' -e '/^#Hiwonder$/d' -e '/^ $/d' ~/.bashrc
rm -rf ~/hiwonder_ros2
rm -f ~/.ros/{apriltag,color_challenge,color_detect,yolo_detect}_tuned.json
```

Then close every open terminal (they still hold env.bash's settings). Maps or files you saved outside the repo folder have to be deleted by hand. Leave the packages `apt` and `rosdep` installed: they belong to ROS and other work may use them.

## Pulling updates from Hiwonder

```bash
git subtree pull --prefix=ROSpider hiwonder Jetson_Nano_ROS2
```

The remote `hiwonder` is https://github.com/Hiwonder/ROSpider.git

## Adding a new robot model

1. `git remote add <name> <Hiwonder repo URL>`
2. `git subtree add --prefix=<model> <name> <branch>`
3. Add a row to the "Robots in this repo" table above (in both languages)
