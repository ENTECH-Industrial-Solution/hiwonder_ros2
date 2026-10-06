# ROSpider on the real robot (real-robot station, afternoon)

[ไทย](REAL_ROBOT.md) | English

This guide is for instructors. Use it together with [README.en.md](README.en.md) (simulation): in the morning participants do the exercises in the sim; in the afternoon they try their sim answers on the real robot.

## 1. Overview

- **The robot runs Hiwonder's code that comes with its system image, not the sim package.** It uses ROS 2 Humble on a Jetson, the code is in `/home/ubuntu/ros2_ws`, and nothing needs to be installed or built.
- **Launch commands have the same names as in the sim**; only the package name differs. For example, the sim uses `rospider_gazebo apriltag_track.launch.py`; the robot uses `example apriltag_track.launch.py`.
- **The automatic checkers (`check_*.py`) don't work on the robot**, because they read true positions from Gazebo. On the robot, the instructor judges with the criteria in this guide, using tape on the floor to make the calls clear.
- **The idea of the robot exercises: "how far do the sim answers carry over to the real thing?"** No settings are set wrong on purpose; reality brings its own problems: room lighting, units of distance, and the robot's own inaccuracy.
- **The commands in this guide were read from Hiwonder's code in this repo and have not yet been run on the real robot.** In the morning, rehearse every item on the robot once first (section 3).

## 2. Preparation before the workshop day

Equipment:

| Item | Quantity | Used in |
|---|---|---|
| Red, green and blue cubes, about 3–5 cm (the size the robot's kit can pick) | 1–2 per colour | Exercises 1, 2 |
| An orange or pink cube (a decoy close to red) | 1 | Exercise 1 |
| A cardboard box about 30 cm tall (to cast shadow) | 1 | Exercise 1 |
| A pedestal as tall as the one Hiwonder's track_and_grab uses | 1 | Exercise 2 |
| A tray or box to drop into | 1 | Exercise 2 |
| AprilTag boards, tag36h11, ids 1, 2, 3 (the kit's boards, or printed at the same size) | 1 set | Exercise 3, mission |
| Coloured floor tape, tape measure | 1 | All |
| Cardboard boxes for room A/B walls and the doorway (like the sim's maze room, scaled down) | as space allows | Mission |
| Fully charged spare batteries | at least 1 | All |

Preparation:

1. **Reach the robot from the instructor's machine** through NoMachine (shows the robot's screen; needed for the LAB_Tool window) or SSH
2. **Back up the colour file**, to restore it between groups
   ```bash
   cp ~/software/lab_tool/lab_config.yaml ~/lab_config.backup.yaml
   ```
3. **Make the mission arena's map in advance**, following Hiwonder's guide (`ros2 launch slam slam.launch.py`, then save the map). Don't let each group build its own; it would use up all the robot time
4. **The participants' laptops must not share the robot's `ROS_DOMAIN_ID`.** The sim on a laptop also publishes `/controller/cmd_vel`; with the same number on the same Wi-Fi, the real robot walks to the sim's commands
5. **Tape the floor**: the robot's start point (with an arrow for its heading), the cube spots for exercise 1, the pedestal and tray for exercise 2, the tag positions for exercise 3

## 3. Morning rehearsal (30 min)

Run exercises 1–3 and the mission on the robot once, following sections 5–6. Note everything that doesn't match this guide, such as command names, file locations or where the robot stops, and fix the guide before teaching. Measure and keep:
- How many cm from the AprilTag the robot stops (the answer to exercise 3's question)
- The distance at which track_and_grab can pick (to tape the pedestal's spot for exercise 2)

## 4. Start-up every time groups change

On the robot (NoMachine or SSH):

```bash
sudo systemctl stop start_app_node.service
```

This stops the program the robot starts at boot (for the phone app); otherwise it clashes with the demos you start. Then:
1. Put the robot on the start point, facing the arrow
2. Restore the colour file from the backup
   ```bash
   cp ~/lab_config.backup.yaml ~/software/lab_tool/lab_config.yaml
   ```
3. Check the battery; if the robot starts walking unsteadily, swap it

Each exercise starts its own programs. **Always close the previous exercise's (Ctrl+C) before starting the next**, because each Hiwonder launch opens the camera and the robot controller itself, and two at once clash.

## 5. Real-robot exercises (20 min per group)

### Exercise 1: tune colours under real light (8 min)

**Set-up**: put the red, green and blue cubes and the orange decoy on the taped spots in front of the robot. Use the box to put the blue cube in shadow.

**Commands** (on the robot, 2 terminals):
```bash
ros2 launch peripherals depth_camera.launch.py
```
```bash
python3 ~/software/lab_tool/main.py
```

**Steps**:
1. Open the red values from the sim on the laptop (`~/.ros/color_challenge_tuned.json` or `config/color_challenge.yaml`) and set LAB_Tool's red sliders to match
2. Look at the mask: the sim values miss things, or catch too much, under real light
3. Retune red, green and blue one at a time, then press **Save** (writes `lab_config.yaml`, which exercise 2 uses)

**Pass criteria** (from the mask, one colour at a time), as in the sim:
- Level 1: each colour's mask shows all of that colour's cubes
- Level 2: the red mask doesn't show the orange decoy
- Level 3: the blue mask still shows the cube in shadow

**Closing question**: why don't the sim values work? In the sim the light is constant and colours are flat; in reality there is light from several directions, shadows, and a camera that adjusts its own exposure.

### Exercise 2: pick an object (6 min)

**Set-up**: put a cube on the pedestal, taped at the distance the robot can pick from (measured in the rehearsal). The instructor picks the target colour.

**Command** (replace `blue` with the colour the instructor names):
```bash
ros2 launch example track_and_grab.launch.py color:=blue start:=true
```

**Pass criteria**:
- Level 1: the camera turns to follow the ordered colour, not another
- Level 2: the arm lifts the cube
- Level 3: it puts it in the tray without dropping it

This exercise uses the colour ranges the group just tuned in exercise 1. Poor tuning breaks it at level 1, so participants see how the parts connect.

### Exercise 3: AprilTag (4 min)

**Set-up**: put tags 1, 2, 3 in front of the robot, about 40 cm apart and about 1 m from the start point.

**Command**:
```bash
ros2 launch example apriltag_track.launch.py target_tag:=2
```

**Pass criteria**:
- Level 1: the robot walks to tag 2, not another
- Level 2: the robot stops in front of the tag without hitting it

**Closing question**: measure with the tape how many cm from the tag the robot stopped, and compare with the sim's 0.35 m.
- On the real robot the stop distance is hardcoded in Hiwonder's code (`d_stop = 15`); it can't be changed from the command line
- Its unit isn't metres but whatever Hiwonder's tag detector computes
- Lesson: the same value in different units; measure against reality before you use it

## 6. Final mission on the real robot (all groups together, 20 min)

Done at the end of the day: the group that did best on the sim mission commands the robot in front of everyone.

**Different from the sim**, because Hiwonder's code on the robot does less than the sim version:
- One tag and one block
- Commanded step by step by hand; there is no mission file to run automatically
- Hiwonder's track_and_grab puts the block down beside the robot right after picking, and can't carry it back, so the robot mission ends at the pick

**Steps** (close the previous command with Ctrl+C before starting the next, every time):
1. **Nav2 to room B**
   ```bash
   ros2 launch navigation navigation.launch.py map:=<arena map name>
   ```
   In RViz, use **2D Goal Pose** to click a goal in room B, at the coordinates the group wrote in its sim mission file
2. **Walk to the right tag**
   ```bash
   ros2 launch example apriltag_track.launch.py target_tag:=<id>
   ```
3. **Pick the block**
   ```bash
   ros2 launch example track_and_grab.launch.py color:=<colour> start:=true
   ```

**Pass criteria**:
- Level 1: reached room B
- Level 2: stopped in front of the right tag
- Level 3: picked the block of the right colour

**Closing question**: how far did the steps written in the sim carry over to reality? Which steps had to change, and why?

## 7. Group rotation (example: 4 groups, 3-hour afternoon)

Adjust to the real number of groups: 20 minutes per group on the robot, plus 5 minutes to change over.

| Time | Real-robot station | Other groups |
|---|---|---|
| 13:00–13:45 | Instructor sets up and demonstrates once | Final mission in the sim |
| 13:45–14:10 | Group 1 | Sim mission, or revisit unpassed exercises |
| 14:10–14:35 | Group 2 | 〃 |
| 14:35–14:50 | Break / swap batteries | |
| 14:50–15:15 | Group 3 | 〃 |
| 15:15–15:40 | Group 4 | 〃 |
| 15:40–16:00 | Final mission on the real robot (all groups watch), then wrap-up | |

## 8. Score sheet

Print one per group and tick ✓ each level passed.

| Exercise | Level 1 | Level 2 | Level 3 | Notes |
|---|---|---|---|---|
| 1 Colours under real light | All colours found ☐ | Decoy not caught ☐ | Cube in shadow found ☐ | |
| 2 Pick an object | Followed the right colour ☐ | Lifted it ☐ | Put it in the tray ☐ | |
| 3 AprilTag | Went to the right tag ☐ | Stopped without hitting ☐ | Measured stop distance: ___ cm | |
| Mission (representative group) | Reached room B ☐ | Stopped at the right tag ☐ | Picked the right colour ☐ | |

## 9. Common problems

- **The robot moves by itself or ignores commands**: `start_app_node.service` hasn't been stopped, or a previous exercise's command is still running. Close everything and start again
- **The robot follows the sim on a laptop**: the laptop's `ROS_DOMAIN_ID` matches the robot's. Change the number on the laptop
- **A new group's colours catch odd things**: the colour file wasn't restored (section 4)
- **The robot staggers or sags**: low battery; swap it
- **It can't reach the block**: the pedestal is further than the distance measured in the rehearsal; move it back to the tape
