# Preparing your computer for the ROSpider workshop

[ไทย](SETUP.md) | English

This guide is for participants. Finish sections 1–5, in order, **before the workshop day**: downloading and building take a while (about 30–60 minutes, depending on your internet and computer). If you get stuck, see section 7 or contact the instructors before the workshop.

## 1. Pick the method for your computer

The workshop's simulation runs **only on Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic**. If your computer runs something else, **you do not need to reinstall your OS**: you create an Ubuntu 24.04 inside it instead.

| Your computer | Method | Go to |
|---|---|---|
| Ubuntu 24.04 | Install directly | Section 2A |
| Ubuntu 22.04, 20.04 or another Linux (ROS 2 Humble already installed is fine) | A distrobox container | Section 2B |
| Windows 10/11 | WSL2 | Section 2C |
| macOS | Not supported (Gazebo in a VM has no GPU, so the sim is very slow) | Contact the instructors |

Recommended: 8 GB RAM or more (16 GB runs smoother), at least 20 GB free disk space, an Intel/AMD/NVIDIA graphics card with its driver installed.

## 2. Get an Ubuntu 24.04

### 2A. Your computer already runs Ubuntu 24.04

Nothing to prepare. Go to section 3.

### 2B. Another Ubuntu version: use distrobox

distrobox runs Ubuntu 24.04 as a container on your computer. Gazebo and RViz windows open on your normal desktop and use your graphics card. The ROS already on your computer (Humble, for example) is left untouched.

1. Install podman and distrobox (once):

   ```bash
   sudo apt install podman curl
   curl -s https://raw.githubusercontent.com/89luca89/distrobox/main/install | sudo sh
   ```

2. Create a container named `jazzy` with its own home folder:

   ```bash
   distrobox create --name jazzy --image ubuntu:24.04 --home ~/jazzy-home
   ```

   With an NVIDIA graphics card, add `--nvidia` at the end of that command.

3. Enter the container (the first time it spends a while setting itself up):

   ```bash
   distrobox enter jazzy
   ```

4. Inside the container, copy the default shell settings into the new home (otherwise the terminal may not read `~/.bashrc`):

   ```bash
   cp -n /etc/skel/.bashrc /etc/skel/.profile ~/
   ```

**Why a separate home (`--home ~/jazzy-home`)**: without it, the container shares your computer's `~/.bashrc`. The lines section 4 adds there would then make your computer's normal terminals load a workspace built for Jazzy on top of your own ROS, and break whatever else you use ROS for.

From here on, **type every command in sections 3–5 inside the container** (in a terminal where you ran `distrobox enter jazzy`).

### 2C. Windows: use WSL2

1. Open PowerShell with **Run as administrator** and run:

   ```powershell
   wsl --install -d Ubuntu-24.04
   ```

2. Restart if asked, then open the **Ubuntu 24.04** app from the Start menu and set a user name and password when asked.
3. If you installed WSL a long time ago, run `wsl --update` in PowerShell first: Gazebo and RViz windows need the WSLg that comes with recent WSL.

From here on, type every command in sections 3–5 in the Ubuntu 24.04 window.

## 3. Install ROS 2 Jazzy

Shortened from the [ROS installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). If the commands below fail, follow that guide instead.

```bash
sudo apt update && sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop
```

## 4. Download and build the workspace

```bash
sudo apt install -y git python3-rosdep python3-colcon-common-extensions
sudo rosdep init   # an "already exists" error is fine
rosdep update
cd ~
git clone https://github.com/ENTECH-Industrial-Solution/hiwonder_ros2.git
cd ~/hiwonder_ros2/ROSpider
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
echo " " >> ~/.bashrc
echo "#Hiwonder" >> ~/.bashrc
echo "source ~/hiwonder_ros2/ROSpider/env.bash" >> ~/.bashrc
echo "source ~/hiwonder_ros2/ROSpider/install/setup.bash" >> ~/.bashrc
```

`rosdep install` installs Gazebo, Nav2, slam_toolbox, RTAB-Map, MoveIt and the rest for you. It is the slowest step.

If `colcon build` hangs or the computer runs out of RAM, build one package at a time instead: `colcon build --symlink-install --parallel-workers 1`

When it is done, **close every terminal and open a new one** (with distrobox, run `distrobox enter jazzy` again too).

## 5. Test

1. Check that the terminal loaded everything:

   ```bash
   echo $ROS_DISTRO   # must print jazzy
   echo $GZ_IP        # must print 127.0.0.1
   ```

2. Start the sim (terminal 1):

   ```bash
   ros2 launch rospider_gazebo gazebo.launch.py
   ```

   A Gazebo window must open with the spider robot standing in a room.

3. Drive the robot (terminal 2; with distrobox, run `distrobox enter jazzy` first):

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/controller/cmd_vel
   ```

   Press `i` and the robot must walk forward. Press `k` to stop.

4. While the robot walks, look at the **real time factor** at the bottom right of Gazebo. Below ~0.5 means the computer is too slow for some exercises: tell the instructors beforehand.

5. (Optional) Test a heavier setup. Stop everything with Ctrl+C, then run `ros2 launch rospider_gazebo slam_challenge.launch.py`. An RViz window must follow within a few seconds.

If all 5 pass, your computer is ready.

## 6. On the workshop day: opening terminals

- **Ubuntu 24.04 and WSL2**: open a terminal and use the workshop commands as written.
- **distrobox**: every new terminal must enter the container first, then run the command:

  ```bash
  distrobox enter jazzy
  cd ~/hiwonder_ros2/ROSpider
  ros2 launch rospider_gazebo slam_challenge.launch.py
  ```

  Exercises that use two terminals (a launch in one, `ros2 run rospider_gazebo check_slam.py` in the other, for example) need `distrobox enter jazzy` in both. If you forget, you get `Package 'rospider_gazebo' not found` or `ros2: command not found`.

  Shortcut: put `alias jz='distrobox enter jazzy'` in your **computer's own** `~/.bashrc` and just type `jz`.

- **Editing exercise files with an editor on your computer** (VS Code, for example): the files are in

  | Method | Path of the ROSpider folder, opened from your computer |
  |---|---|
  | Ubuntu 24.04 | `~/hiwonder_ros2/ROSpider` |
  | distrobox | `~/jazzy-home/hiwonder_ros2/ROSpider` (inside the container it is `~/hiwonder_ros2/ROSpider`, the same files) |
  | WSL2 | type `\\wsl$\Ubuntu-24.04\home\<user name>\hiwonder_ros2\ROSpider` in File Explorer, or run `code .` in the Ubuntu window |

  Edit a `.yaml` file and run again; no rebuild needed.

- **Don't run your computer's own ROS at the same time as the sim** (with distrobox): the container shares your computer's network.

## 7. Common installation problems

- **`Package 'rospider_gazebo' not found`**: this terminal has not loaded the workspace. With distrobox, check that you are inside the container (`echo $ROS_DISTRO` must print `jazzy`). If you are, check that `colcon build` finished without `Failed` and that `~/.bashrc` has the two `source ~/hiwonder_ros2/ROSpider/...` lines.
- **distrobox: `echo $ROS_DISTRO` prints nothing although you are in the container**: the terminal does not read `~/.bashrc`. Do step 4 of section 2B (`cp -n /etc/skel/.bashrc /etc/skel/.profile ~/`), then `exit` and `distrobox enter jazzy` again (`cp -n` never overwrites a file that exists, so the workspace lines in `~/.bashrc` stay).
- **`rosdep install` says it cannot resolve some keys**: if they belong to the real robot (voice, camera or LiDAR drivers, for example), that is fine; the sim does not use them. All that matters is that `colcon build` succeeds.
- **Gazebo closes at once, or prints `eglInitialize failed`**: a graphics driver problem. Try a reboot. With distrobox and an NVIDIA card, check that the container was created with `--nvidia`.
- **WSL2: no Gazebo/RViz window appears**: run `wsl --update`, then `wsl --shutdown` in PowerShell, and open Ubuntu again.
- Problems while using the sim: see "Common problems" in the [README](README.en.md).

## 8. Checklist before the workshop

- [ ] `echo $ROS_DISTRO` prints `jazzy`
- [ ] `gazebo.launch.py` opens, and the robot walks with teleop
- [ ] Real time factor is not below ~0.5
- [ ] You know how to open the files in the `ROSpider` folder with an editor
- [ ] (distrobox) You remember that every terminal needs `distrobox enter jazzy` first
- [ ] Shortly before the workshop, run `cd ~/hiwonder_ros2 && git pull`, then `cd ROSpider && colcon build --symlink-install` again, in case the exercises were updated

To remove everything afterwards, see "Uninstall" in the [README](README.en.md). With distrobox, `distrobox rm jazzy` and then `rm -rf ~/jazzy-home` remove it all.
