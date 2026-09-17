# ROSpider Simulation

วิธีรัน ROSpider แบบ simulation ล้วนๆ บน PC (ไม่ต่อหุ่นจริง) — ทดสอบบน Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic

## จำลองอะไรได้บ้าง

| ต้องการ | ใช้ | หมายเหตุ |
|---|---|---|
| ดูโมเดล/ขยับข้อต่อด้วย slider | `rospider_description display.launch.py` | RViz อย่างเดียว ไม่มี physics |
| วางแผนแขนกลแบบไม่ใช้ Gazebo | `robot_moveit_config demo.launch.py` | mock hardware |
| หุ่นในสภาพแวดล้อม + เซนเซอร์ | `rospider_gazebo gazebo.launch.py` | LiDAR, depth camera, IMU, odom |
| ทำแผนที่ (SLAM) | `rospider_gazebo slam.launch.py` | slam_toolbox + ค่า `slam/config/slam.yaml` |
| ทำแผนที่ด้วยกล้อง + LiDAR | `rospider_gazebo rtabmap_slam.launch.py` | RTAB-Map (RGB-D + LiDAR) ด้วยค่าของ Hiwonder |
| นำทางด้วยแผนที่ RTAB-Map | `rospider_gazebo rtabmap_navigation.launch.py` | RTAB-Map โหมด localization + Nav2 |
| V-SLAM กล้องอย่างเดียว | `rospider_gazebo vslam.launch.py` | RTAB-Map ใช้แค่ depth camera (ภาพสี + depth) + odometry ไม่แตะ LiDAR เลย มีไฟล์ของตัวเองทั้งหมด — ทำแผนที่ หรือโหลดแผนที่ 3D มานำทาง (`localization:=true`) |
| นำทาง (Nav2) | `rospider_gazebo navigation.launch.py` | แผนที่ 2D ห้อง sim ที่ทำไว้แล้ว หรือแผนที่ของเราเอง |
| แขนกล MoveIt ใน Gazebo | `rospider_gazebo moveit.launch.py` | สั่งแขน/gripper ผ่าน MoveIt |
| หยิบและวางลูกบาศก์สีอัตโนมัติ | `rospider_gazebo pick_place.launch.py` | ตรวจจับสีด้วย OpenCV + IK ปิดรูปเอง ไม่ใช้ MoveIt |
| หน้าต่างเดโมของ Hiwonder (สี, AprilTag, AR, KCF, depth, MediaPipe) | `rospider_gazebo vision_demo.launch.py demo:=...` | ย้ายมาจาก `src/example/` 15 ตัว — ดูข้อ 11 |

**ข้อจำกัดสำคัญ:** โค้ดเดินจริงของ Hiwonder (`driver/kinematics/kinematics.so`) เป็น binary ของ ARM (Jetson) เท่านั้น ไม่มี source จึงรันบน PC ไม่ได้ ใน sim จึงใช้ท่าเดินที่เขียนขึ้นเอง (`scripts/sim_gait.py`) — ก้าวขาแบบ tripod (ยกทีละ 3 ขาสลับกัน) ตามความเร็วที่สั่ง เท้าที่แตะพื้นอยู่นิ่งกับพื้น แต่ตัวหุ่นถูก Gazebo เลื่อนไปตาม `cmd_vel` โดยตรง (ขาไม่ได้ออกแรงดันพื้นจริง) — เหมาะกับทดสอบ SLAM / Nav2 / vision / แขนกล แต่ไม่เหมาะกับทดสอบการเดินบนพื้นขรุขระ การทรงตัว หรือท่าเดินของหุ่นจริง

## ติดตั้งครั้งแรก

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

`rosdep` จะติดตั้ง `ros-jazzy-trac-ik-kinematics-plugin` ให้ด้วย (ถ้าไม่มี MoveIt จะหา IK ของแขนไม่ได้ — ลาก marker ใน RViz ไม่ได้ แต่สั่งแบบ joint ยังได้) ถ้าไม่อยากรัน rosdep ทั้งหมด:

```bash
sudo apt install ros-jazzy-trac-ik-kinematics-plugin
```

Build:

```bash
colcon build
source install/local_setup.bash
```

ทุก terminal ที่จะรันคำสั่งด้านล่างต้อง `source /opt/ros/jazzy/setup.bash` และ `source install/local_setup.bash` ก่อน

## 1. ดูโมเดลใน RViz

launch file ของ Hiwonder ต้องมีตัวแปร `need_compile`:

```bash
export need_compile=True
ros2 launch rospider_description display.launch.py
```

## 2. MoveIt แบบ mock (ไม่มี Gazebo)

```bash
ros2 launch robot_moveit_config demo.launch.py
```

## 3. Gazebo

```bash
ros2 launch rospider_gazebo gazebo.launch.py
```

โลกเริ่มต้นคือห้อง 4×3 ม. (`worlds/rospider_room.sdf`) มีกำแพงกั้น กล่อง ทรงกระบอก และลูกบาศก์สีแดง/เขียว/น้ำเงินหน้าหุ่นไว้ทดสอบ vision (ลูกบาศก์ไม่มี collision — หุ่นเดินทะลุได้ เพราะเตี้ยกว่าระนาบ LiDAR ทำให้ Nav2 หลบไม่ได้)
argument: `world:=<path.sdf>`, `x:=` `y:=` `yaw:=` (จุดเกิด), `arm_pose:=horizontal` (ให้กล้องบนแขนมองตรงไปข้างหน้า แทนท่าเริ่มต้นที่ก้มมองพื้น), `gui:=false` (ไม่เปิดหน้าต่าง Gazebo — ดูหัวข้อปัญหาที่พบบ่อย)

ขับหุ่นด้วยคีย์บอร์ด (อีก terminal):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/controller/cmd_vel
```

ขาก้าวได้เร็วสุดราว 0.23 ม./วิ. (หมุน ~1 rad/วิ.) ถ้าสั่งเร็วกว่านั้นตัวหุ่นจะถูกลดความเร็วลงให้ขาตามทัน — ค่าเริ่มต้นของ teleop_twist_keyboard คือ 0.5 ม./วิ. กด `z` เพื่อลดความเร็ว (หุ่นจริงเดินราว 0.03–0.05 ม./วิ.)

### Topic ที่ sim ให้ (ชื่อเดียวกับหุ่นจริง)

| Topic | ชนิด |
|---|---|
| `/controller/cmd_vel` | สั่งเคลื่อนที่ (Twist) — teleop, app, Nav2 ใช้ topic นี้ (`sim_gait` รับไปก้าวขา) |
| `/scan` | LaserScan 360° ระยะ 0.2–12 ม. (ไม่เห็นตัวหุ่นเอง) |
| `/imu` | Imu |
| `/odom`, TF `odom → base_footprint` | odometry จาก Gazebo (ไม่มี drift) |
| `/joint_states` | ข้อต่อทั้ง 29 ตัว |
| `/depth_cam/rgb/image_raw`, `/depth_cam/rgb/camera_info` | ภาพสี 640×480 |
| `/depth_cam/depth/image_raw`, `/depth_cam/depth/camera_info` | depth แบบ `32FC1` (หน่วยเมตร) |
| `/depth_cam/depth/points` | PointCloud2 |

Controller (ros2_control): `leg_controller`, `arm_controller`, `gripper_controller` (action `<ชื่อ>/follow_joint_trajectory` ชื่อเดียวกับบนหุ่นจริง)

ดูภาพกล้อง: `ros2 run rqt_image_view rqt_image_view /depth_cam/rgb/image_raw`

## 4. SLAM (ทำแผนที่ด้วย LiDAR)

```bash
ros2 launch rospider_gazebo slam.launch.py
```

เปิด Gazebo + slam_toolbox + RViz (`slam/rviz/slam.rviz`) แล้วขับหุ่นด้วย teleop ให้ทั่วห้อง จากนั้นบันทึกแผนที่ 2D — ดูหัวข้อ "บันทึกและเรียกใช้แผนที่"

## 4.1 RTAB-Map (กล้อง + LiDAR)

```bash
ros2 launch rospider_gazebo rtabmap_slam.launch.py map:=room1
```

เปิด Gazebo โดยยกกล้องบนแขนให้มองตรงไปข้างหน้า (เหมือนท่า `init_horizontal` ที่หุ่นจริงใช้ก่อนรัน RTAB-Map) แล้วรัน RTAB-Map ด้วย launch ของ Hiwonder (`slam/launch/include/rtabmap.launch.py`) — ใช้ภาพสี + depth + LiDAR และ odometry จาก `/odom` พร้อม RViz (`slam/rviz/rtabmap.rviz`) แสดง point cloud, graph และแผนที่ 2D ขับหุ่นด้วย teleop ให้ทั่วห้อง ผนังและสิ่งกีดขวางในห้อง sim มีลวดลายให้กล้องจับ feature ได้ RTAB-Map จึงหา loop closure ได้ (ผนังสีเรียบจะหาไม่เจอ)

- แผนที่ถูกบันทึกตอนปิด launch ลง `ROSpider/maps/room1.db` (ไม่ใส่ `map:=` จะเป็น `rtabmap.db`) — เริ่มรอบใหม่ด้วยชื่อเดิม ไฟล์เดิมจะถูกเขียนทับ
- เปิดดูฐานข้อมูล: `rtabmap-databaseViewer maps/room1.db` (รันจากโฟลเดอร์ `ROSpider`)
- RTAB-Map ใช้ CPU มาก real time factor จะลดลง (ราว 0.6 บนเครื่องที่ทดสอบ)

## 4.2 V-SLAM กล้องอย่างเดียว (depth camera)

```bash
ros2 launch rospider_gazebo vslam.launch.py map:=room_vslam
```

V-SLAM รันด้วยไฟล์ของตัวเองทั้งหมด ไม่ยืมของ `rtabmap_slam` หรือของ Hiwonder:

| ไฟล์ | ใช้ทำอะไร |
|---|---|
| `config/vslam.yaml` | ค่า RTAB-Map (มาจากค่าของ Hiwonder แต่ตัด LiDAR ออก) และค่าของ point cloud ที่ใช้หลบสิ่งกีดขวาง |
| `config/vslam_nav2_params.yaml` | ค่า Nav2 ตอนนำทาง — หลบสิ่งกีดขวางด้วย depth camera |
| `rviz/vslam.rviz` | RViz: ภาพกล้อง, point cloud 3D ของแผนที่, graph, แผนที่ 2D, costmap, เส้นทาง |
| `ROSpider/maps/vslam/` | โฟลเดอร์แผนที่ของ V-SLAM |

ไม่ใช้ LiDAR เลยทั้งตอนทำแผนที่และตอนนำทาง — RTAB-Map ใช้แค่ภาพสี + depth จาก depth camera และ odometry จาก `/odom`:

- ต่อแผนที่และหา loop closure จาก feature ในภาพ (ไม่ใช้ ICP ของ LiDAR)
- แผนที่ 2D สร้างจาก depth (ตัดพื้นออก และไม่นับจุดที่ใกล้กว่า 0.2 ม. เพราะกล้องเห็นนิ้ว gripper)
- แผนที่ 3D ถูกบันทึกตอนปิดลง `ROSpider/maps/vslam/room_vslam.db` (ไม่ใส่ `map:=` จะเป็น `maps/vslam/map.db`) — เริ่มรอบใหม่ด้วยชื่อเดิม ไฟล์เดิมจะถูกเขียนทับ

### เรียกแผนที่ 3D กลับมาใช้นำทาง

```bash
ros2 launch rospider_gazebo vslam.launch.py localization:=true map:=room_vslam
```

RTAB-Map โหลดแผนที่ 3D ทั้งหมดจากไฟล์ (ภาพ + point cloud) หาตำแหน่งตัวเองโดยเทียบภาพจากกล้องกับแผนที่ แล้วส่งแผนที่ 2D ที่ฉายจากแผนที่ 3D กับตำแหน่งหุ่นให้ Nav2 — กด **2D Goal Pose** ใน RViz เพื่อสั่งเดิน RViz แสดง point cloud 3D ของแผนที่ด้วย

- โหมดนี้ไม่เพิ่มข้อมูลใหม่ลงแผนที่ และไม่ลบไฟล์ (โหมดทำแผนที่ต่างหากที่เริ่มไฟล์ใหม่ทุกครั้ง) แต่ตอนปิดยังเขียนข้อมูลบางส่วนลงไฟล์ — ถ้าต้องการต้นฉบับเดิมให้ copy เก็บไว้ก่อน
- หาตำแหน่งด้วยกล้อง และ Nav2 หลบสิ่งกีดขวางด้วย depth camera (point cloud ขนาดเล็กที่สร้างจากภาพ depth ราว 2 พันจุด) — เห็นเฉพาะด้านหน้ากล้อง (~69°) ด้านข้างและด้านหลังมองไม่เห็น ต่างจาก LiDAR ที่เห็นรอบตัว
- ใช้แผนที่จาก `vslam.launch.py` เท่านั้น — แผนที่จาก `rtabmap_slam.launch.py` ให้ใช้กับ `rtabmap_navigation.launch.py`

ข้อจำกัด: หันเข้าหาผนังเรียบหรือใกล้ผนังมาก ภาพจะมี feature น้อยจนต่อแผนที่หรือหาตำแหน่งพลาดง่ายกว่าแบบมี LiDAR และ depth เห็นระยะสั้นกว่า LiDAR (sim ตั้งไว้ 8 ม. แผนที่ 2D ใช้ถึง 5 ม.)

## 5. Navigation

```bash
ros2 launch rospider_gazebo navigation.launch.py
```

ใช้แผนที่ `rospider_room` ที่ทำจากห้อง sim ไว้แล้ว และตั้งตำแหน่งเริ่มต้นให้อัตโนมัติ (หุ่นเกิดที่ origin ของแผนที่) — กด **2D Goal Pose** ใน RViz เพื่อสั่งเดิน หรือ:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.9, y: 0.3}, orientation: {w: 1.0}}}}"
```

ใช้แผนที่ของเราเอง: `map:=room1` (แผนที่ 2D ใน `ROSpider/maps/`) — ถ้าหุ่นไม่ได้เกิดที่ origin ของแผนที่ ให้ตั้งตำแหน่งด้วย **2D Pose Estimate**

ความเร็วและ footprint มาจาก config ของหุ่นจริง (`navigation/config`) — สูงสุด 0.05 ม./วิ. หุ่นจึงเดินช้า (เป้าหมายห่าง ~1 ม. ใช้เวลาราว 1 นาที) ต่างจากค่าของ Hiwonder 2 ค่าใน DWB คือ `xy_goal_tolerance` และ `sim_time` เพราะค่าเดิมทำให้หุ่นหยุดก่อนถึงเป้าหมายและ goal ไม่จบ (รายละเอียดที่หัวไฟล์ `src/simulations/rospider_gazebo/config/nav2_params.yaml`)

ลองค่า Nav2 อื่นโดยไม่ต้อง build ใหม่: `params_file:=/abs/path/my_nav2_params.yaml`

## บันทึกและเรียกใช้แผนที่

แผนที่ทุกแบบเก็บไว้ใน workspace ที่ `ROSpider/maps/` ใส่แค่**ชื่อ**ใน `map:=` (ถ้าอยากใช้ไฟล์ที่อื่น ใส่ path ที่มี `/` ได้ เช่น `map:=$HOME/other/room.db`) — ตอนเริ่ม launch จะพิมพ์ path ของแผนที่ออกมาให้เห็น

| แผนที่ | ทำด้วย | เรียกใช้ด้วย |
|---|---|---|
| 2D `room1.yaml` + `.pgm` | `map_saver_cli` ระหว่าง `slam` / `rtabmap_slam` / `vslam` | `navigation.launch.py map:=room1` |
| 3D `room1.db` (กล้อง + LiDAR) | `rtabmap_slam.launch.py map:=room1` | `rtabmap_navigation.launch.py map:=room1` |
| 3D `vslam/room_vslam.db` (กล้องอย่างเดียว) | `vslam.launch.py map:=room_vslam` | `vslam.launch.py localization:=true map:=room_vslam` |

### แผนที่ 2D (`.yaml` + `.pgm`) — ใช้กับ Nav2 + AMCL (LiDAR)

บันทึกระหว่างที่ launch ทำแผนที่ยังรันอยู่ (ขับให้ทั่วก่อน) — รันจากโฟลเดอร์ `ROSpider`:

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider
ros2 run nav2_map_server map_saver_cli -f maps/room1 --ros-args -p use_sim_time:=true
```

เรียกใช้:

```bash
ros2 launch rospider_gazebo navigation.launch.py map:=room1
```

AMCL ตั้งตำแหน่งเริ่มต้นไว้ที่จุด (0, 0) ของแผนที่ = จุดที่หุ่นเกิดตอนเริ่มทำแผนที่ ถ้าไม่ตรง ให้กด **2D Pose Estimate** ใน RViz

### แผนที่ 3D (`.db`) — RTAB-Map

ขับให้ทั่วแล้วปิด launch ทำแผนที่ด้วย **Ctrl+C** และรอจนขึ้น `Saving database/long-term memory...done!` — ไฟล์ `.db` ถูกเขียนตอนปิดเท่านั้น จากนั้นเรียกใช้ด้วยคำสั่งในตารางด้านบน

- **อย่า copy ไฟล์ `.db` ระหว่างที่ RTAB-Map ยังรันอยู่** ไฟล์ที่ได้จะเสีย (`database disk image is malformed`)
- ในไฟล์ `.db` มีแผนที่ 2D อยู่ด้วย ถ้าอยากใช้กับ `navigation.launch.py` ให้รันโหมดโหลดแผนที่แล้วสั่ง `map_saver_cli` ตามด้านบน

### git

ไฟล์ `.db` ใหญ่หลายสิบถึงหลายร้อย MB (GitHub รับไฟล์ละไม่เกิน 100 MB) จึงถูก ignore ไว้ใน `ROSpider/.gitignore` — แผนที่ 2D ไฟล์เล็ก commit ได้ตามปกติ ถ้าต้องการแชร์ไฟล์ `.db` ให้ใช้ Git LFS หรือส่งไฟล์แยก

## 6. MoveIt ใน Gazebo

```bash
ros2 launch rospider_gazebo moveit.launch.py
```

RViz จะเปิดแท็บ MotionPlanning — เลือก planning group `arm` หรือ `gripper` แล้ว Plan & Execute แขนใน Gazebo จะขยับตาม และภาพกล้อง (ติดอยู่บนแขน) เปลี่ยนตาม

## 7. หยิบและวางวัตถุ (pick and place)

```bash
export need_compile=True
ros2 launch rospider_gazebo pick_place.launch.py
```

หุ่นยืนอยู่กับที่ ก้มแขนมองหาลูกบาศก์สีบนแท่นด้วยกล้อง RGB-D ที่ติดอยู่บนแขน แล้วหยิบไปวางเรียงกันทีละลูกที่อีกจุดหนึ่งหน้าหุ่น การตรวจจับสีใช้ OpenCV แยกสีในปริภูมิ LAB แบบเดียวกับหุ่นจริง (`scripts/color_detect.py`, ค่าปรับที่ `config/color_detect.yaml`) แล้วส่งผลออกทาง `/yolo/object_detect` (`interfaces/ObjectsInfo`) — หัวข้อและชนิดข้อความเดียวกับ `yolo_node.py` ของหุ่นจริง จึงเปลี่ยนไปใช้ YOLO จริงภายหลังได้โดยไม่ต้องแก้โค้ดส่วนหยิบ-วาง (`scripts/pick_and_place.py`)

ไม่ใช้ MoveIt ในเส้นทางหยิบ-วาง: คุมด้วย IK ปิดรูปเอง (`arm_ik.py`) แทน เพราะ `robot_moveit_config` ตั้ง `position_only_ik: true` (คุมได้แค่ตำแหน่งปลายมือ ไม่คุมทิศทาง) แต่การก้มลงหยิบต้องคุมมุมมือด้วย ส่วนการ "จับ" ลูกบาศก์ใช้ `DetachableJoint` เชื่อมลูกบาศก์เข้ากับ `link5` (มือจับ) แทนแรงเสียดทานจริง

รันทีละสี:

```bash
ros2 launch rospider_gazebo pick_place.launch.py auto_start:=false
ros2 service call /pick_and_place/start interfaces/srv/SetString "{data: 'red'}"
ros2 service call /pick_and_place/stop std_srvs/srv/Trigger
```

### หยิบ → ขับไปที่อื่น → สั่งวาง

`~/start` เป็นวงจรอัตโนมัติก้อนเดียว ไม่มีจังหวะให้แทรก ถ้าอยากคุมเองว่าจะวางตอนไหนให้ใช้คู่ `~/pick` กับ `~/place` แทน — หยิบแล้วหุ่นจะ **ถือค้างไว้** จนกว่าจะสั่งวาง ระหว่างนั้นขับไปไหนก็ได้

**terminal 1** เปิดซิม (ต้อง `auto_start:=false` ไม่งั้นวงจรอัตโนมัติจะแย่งหยิบไปก่อน)

```bash
export need_compile=True
ros2 launch rospider_gazebo pick_place.launch.py auto_start:=false
```

**terminal 2** สั่งหยิบ แล้วมันจะยกขึ้นท่าถือและค้างรอ

```bash
ros2 service call /pick_and_place/pick interfaces/srv/SetString "{data: 'red'}"
```

**terminal 3** ขับหุ่นไปจุดที่อยากวาง

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/controller/cmd_vel
```

**terminal 2** สั่งวาง — เว้น `data` ว่างไว้จะวางตรงกลางหน้าหุ่น ณ ตำแหน่งที่ยืนอยู่ตอนนั้น

```bash
ros2 service call /pick_and_place/place interfaces/srv/SetString "{data: ''}"
```

หรือระบุพิกัดเองเป็น `"x y z"` เมตร เทียบ `base_footprint` (คือเทียบตัวหุ่น ไม่ใช่พิกัดโลก) เช่นวางเยื้องไปทางซ้าย 6 ซม. โดยไม่ต้องขยับหุ่น:

```bash
ros2 service call /pick_and_place/place interfaces/srv/SetString "{data: '0.17 0.06 0.035'}"
```

`z` คือ**ความสูงที่ปล่อยมือ** ไม่ใช่ความสูงที่ลูกบาศก์จะไปนอน ค่า default `0.035` = จุดศูนย์กลางลูกบาศก์ตอนนอนบนพื้น (0.025) + ระยะปล่อย 1 ซม. ถ้าจะวางกลับขึ้นแท่นสูง 8 ซม. ให้ใส่ราว `0.135` แทน ปล่อยจากสูงเกินไปลูกบาศก์จะกระเด้ง — ตอนทดสอบใช้ 0.135 บนพื้น (ตก 11 ซม.) เด้งออกไป 2-6 ซม. ถ้าใส่ `z` ต่ำกว่าพื้นผิวที่อยู่ข้างล่าง มือจะกดลูกบาศก์ลงไป (มือไม่มี collision แต่ลูกบาศก์มี) ปกติมันจะไปนอนบนพื้นผิวนั้นเอง — ทดสอบแล้ว วางด้วยค่า default ขณะยังอยู่เหนือแท่น ลูกบาศก์นอนบนแท่นที่ 0.105 ถูกต้อง คลาดเคลื่อน 2.3 ซม. แต่ถ้าจุดนั้นมีของอยู่แล้วลูกบาศก์จะถูกบีบกระเด็นออกข้าง — อีกการทดสอบวางสองลูกห่างกัน 5.5 ซม. (น้อยกว่าความกว้าง 5 ซม. บวกระยะเผื่อ) ลูกที่สองกระเด็นไป 10 ซม. ตกจากแท่น สรุป: ใส่ `z` ให้ตรงกับพื้นผิวที่จะวาง และเว้นที่รอบจุดวางไว้

วนซ้ำ `pick` → ขับ → `place` ได้เรื่อย ๆ เสร็จแล้วโหนดกลับไป `IDLE` รอคำสั่งถัดไป (ไม่ไล่หยิบสีอื่นต่อเอง)

**สิ่งที่ควรรู้**

- ทุก service คืน `success` กับ `message` เสมอ สั่ง `place` ตอนไม่ได้ถืออะไร หรือ `pick` ตอนถืออยู่แล้ว หรือพิมพ์พิกัดที่แขนเอื้อมไม่ถึง จะได้ `success: false` พร้อมเหตุผล
- ตอนถืออยู่ **ไม่มี timeout** ขับนานแค่ไหนก็ได้ (สถานะอื่นมี timeout 15 วิ)
- ลูกบาศก์ถูก weld กับ `link5` จึงติดไปกับหุ่นเอง และท่าถือยกไว้สูง 0.22 ม. พ้นแท่นและพ้นขา
- `~/stop` ระหว่างถืออยู่จะปล่อยของลงตรงนั้น ไม่ค้างติดมือ

### วางเรียงแถว ไม่ซ้อนกัน

ช่องวางทั้งสามอยู่ที่ (0.160, +0.07, 0.135), (0.160, 0.00, 0.135), (0.160, -0.07, 0.135) ใน `base_footprint` (`config/pick_place.yaml`, `drop_slots`) เติมตามลำดับที่หยิบสำเร็จ ไม่ใช่ตำแหน่งคงที่ต่อสี ที่วางเรียงแทนที่จะซ้อนสามชั้นเพราะแขนสั้นเกินไป: ที่ความสูงซ้อนชั้นที่สาม มีแค่มุมก้ม 50-60 องศาที่ยังแก้ IK ได้ และมุมป้านขนาดนั้นต้องกวาดลูกบาศก์ที่ถืออยู่ผ่านลูกบาศก์ที่วางไว้แล้วในแนวราบ ผลวัดจริง: ลูกบาศก์แต่ละลูกห่างจากช่องของตัวเองไม่เกิน ~0.9 ซม.

### กล้องเห็นลูกบาศก์สีเดียวกันสองลูก

`/yolo/object_detect` รายงานวัตถุ 6 ชิ้น ไม่ใช่ 3 ชิ้น: `worlds/rospider_room.sdf` มีลูกบาศก์ตกแต่ง (visual-only ไม่มี collision) สีแดง/เขียว/น้ำเงินอยู่ที่ x=0.55 อยู่ก่อนแล้ว เป็นสีและขนาด (5 ซม.) เดียวกับลูกบาศก์ที่หยิบได้จริงที่ x=0.235 ทุกประการ `pick_and_place.py` เลือกกล่องที่ใหญ่ที่สุดต่อสีก่อน แล้วยืนยันด้วย IK ว่าจุดนั้นแขนเอื้อมถึงจริง — พฤติกรรมนี้เกิดบนหุ่นจริงด้วยเช่นกัน ใครอ่าน `/yolo/object_detect` ต่อจากนี้ต้องรู้ไว้

### ต้อง detach ก่อนสั่งแขน

Gazebo เชื่อมลูกบาศก์ทั้งสามเข้ากับมือจับ (`link5`) ทันทีตอนสแปวน์ (ก่อนโค้ดฝั่งเราสั่งอะไรเลย) `pick_and_place.py` จึงสั่ง detach ทั้งสามสีซ้ำด้วยตัวจับเวลาตอนเริ่มโหนด (ราว 3 วิ) และค้างสถานะ `IDLE` ไม่สั่งแขนจนกว่าขั้นนี้จะจบ — ข้ามขั้นนี้ไม่ได้ ไม่งั้นคำสั่งแขนแรกจะลากทั้งฉาก (แท่น+ลูกบาศก์) หลุดจากจุดตั้ง ใครเขียนโหนดอื่นที่ขยับแขนตัวนี้ต้องเจอเรื่องเดียวกัน

### ปรับค่า

- ช่วงสี LAB: `config/color_detect.yaml` ดูผลได้จากภาพ `/color_detect/image_result` ใน RViz หรือจูนสด ๆ ในหน้าต่าง LAB_Tool (หัวข้อถัดไป)
- ท่ามอง (`look_pose`), มุมเข้าหยิบ, ตำแหน่งช่องวาง (`drop_slots`): `config/pick_place.yaml` — `grasp_z_offset` ไม่ใช่แค่ครึ่งความสูงลูกบาศก์ เพราะจากมุมมองก้มชัน กล่องที่ตรวจจับได้คลุมทั้งหน้าบนและหน้าหน้า (foreshortened) จุดศูนย์กลางกล่องจึงตกที่ขอบบนใกล้กล้อง ไม่ใช่กึ่งกลางหน้าบน ต้องชดเชยด้วยค่านี้ ค่าที่ตั้งไว้แก้ z-bias เท่านั้น — ยังเหลือ x-bias ประมาณ 8 มม. ที่**ตั้งใจไม่แก้** (ดูคอมเมนต์ของ `grasp_z_offset` ในไฟล์เดียวกัน) อย่าปรับ `grasp_z_offset` เพื่อไล่ตาม x-bias นี้ ไม่งั้นจะกลับไปชนบั๊ก "แก้ค่าเดียวกันซ้ำสองที่" ที่เคยเจอและแก้ไปแล้วในสาขานี้
- ตำแหน่งแท่นและลูกบาศก์: คีย์ `scene` ใน `config/pick_place.yaml` (ไม่ใช่ตัวแปรแยกใน launch file อีกต่อไป) — ทั้ง `launch/pick_place.launch.py` และ `test/test_arm_ik.py` อ่านจากที่เดียวกันนี้ ถ้าย้ายตำแหน่งเพียงแก้ที่ `scene` แล้วรัน
  `colcon test --packages-select rospider_gazebo` เพื่อยืนยันว่ายังอยู่ในระยะที่แขนเอื้อมถึง

### จูนช่วงสี LAB ด้วยหน้าต่าง LAB_Tool

```bash
ros2 launch rospider_gazebo pick_place.launch.py tune:=true
```

เปิดหน้าต่าง `LAB_Tool 1.0` ขึ้นมาอีกบาน วางหน้าตาตาม **LAB_Tool ของหุ่นจริง** (คู่มือ Hiwonder หัวข้อ 6.1): ซ้ายคือ mask ของสีที่กำลังเลือก ขวาคือภาพกล้อง ใต้ภาพเป็นแถว `L` `A` `B` แถวละ slider `min` กับ `max` (0-255 พิมพ์ตัวเลขในช่องข้าง slider ได้) ฝั่งขวาคือ `Color list` เลือกสี กับปุ่ม `Add` / `Delete` / `Save` / `Quit` ระหว่างจูนโหนดยัง publish `/yolo/object_detect` และ `/color_detect/image_result` ตามปกติ จึงเห็นผลของ slider ต่อการตรวจจับได้ทันที

ค่าที่จูนอยู่ในปริภูมิ **LAB แบบ 8 บิตของ OpenCV** ตัวเลขเดียวกับที่ LAB_Tool บนหุ่นโชว์: `L` ความสว่าง, `A` เขียว(0)→แดง(255), `B` น้ำเงิน(0)→เหลือง(255) สีเทาอยู่ที่ A = B = 128 พื้นสีเทาอ่อนของ sim วัดได้ A/B ราว 127-130 ส่วนลูกบาศก์แดงดัน A ขึ้นไป ~180 เขียวดึง A ลง ~70 น้ำเงินดึง B ลง ~60 ปกติจึงเปิด `L` ไว้กว้าง (0-255) ให้ด้านที่อยู่ในเงายังติด แล้วไปบีบที่ `A`/`B`

ไฟล์ config มี**ช่วงเดียวต่อสี** เป็น `min`/`max` อย่างละ 3 ค่า รูปแบบเดียวกับ `lab_config.yaml` บนหุ่นจริง (สีแดงไม่ต้องมี 2 ช่วงแบบ HSV อีกแล้ว เพราะ LAB ไม่มีจุดวนของ hue):

```yaml
    red:
      min: [0, 150, 130]
      max: [255, 255, 255]
```

ปุ่ม:

| ปุ่ม | ทำอะไร |
|---|---|
| `Add` | ถามชื่อสีใหม่ (ใช้ได้เฉพาะ `a-z`, `0-9`, `_` — ชื่อนี้กลายเป็น `class_name` บน `/yolo/object_detect` และคีย์ใน YAML) แล้วเริ่มด้วยช่วงเปิดกว้างสุด `[0,0,0]`-`[255,255,255]` ให้ mask เห็นทุกอย่างก่อน ค่อยบีบ A/B ลงหาเป้า |
| `Delete` | ลบสีที่เลือกอยู่ (ลบสีในตัวก็ได้ เหมือน LAB_Tool — ค่าใน YAML จะกลับมาเมื่อลบไฟล์ JSON) |
| `Save` | เซฟค่าปัจจุบันลง `~/.ros/color_detect_tuned.json` และพิมพ์บล็อก YAML ออกทาง terminal สำหรับวางกลับเข้า `config/color_detect.yaml` |
| `Quit` | ปิดหน้าต่าง (โหนดยังตรวจจับต่อ) |

ภาพ `/color_detect/image_result` วาดเหมือน `color_detect_node.py` ของหุ่นจริง: **รูปเดียวรอบก้อนที่ใหญ่ที่สุด**ในบรรดาทุกสี เป็นวงกลม (`detect_type: circle` ค่าเริ่มต้น ตามที่ `color_position`/`color_recognition` ต้นฉบับขอ) หรือกรอบหมุน + จุดกลาง (`detect_type: rect`) ด้วยสี `range_rgb` ของ Hiwonder ไม่มีชื่อสีบนภาพ — ส่วน `/yolo/object_detect` ยังส่งทุกก้อนของทุกสีเหมือนเดิม กรอบที่วาดให้สีที่เพิ่มเองจะใช้สี BGR ของกึ่งกลางช่วง LAB ที่ตั้งไว้

**ลำดับความสำคัญของค่า:** `config/color_detect.yaml` คือค่าที่ commit ไว้ ส่วนไฟล์ JSON เป็นตัวทับชั่วคราวสำหรับจูน ถ้ามีไฟล์ JSON อยู่ โหนดจะโหลดมาทับตอนเริ่ม **และขึ้น log เตือนว่ากำลังใช้ไฟล์ไหน** — กัน JSON เก่าค้างแล้วทับ YAML แบบเงียบ ๆ จนไล่หาไม่เจอ ลบไฟล์ JSON ทิ้งเพื่อกลับไปใช้ค่าใน YAML และใช้บล็อก YAML ที่ `Save` พิมพ์ให้ ย้ายค่าที่พอใจแล้วกลับเข้า YAML ซึ่งเป็นไฟล์ที่ commit จริง ไฟล์ JSON รูปแบบเก่าจากสมัย HSV (มีคีย์ `lower`/`upper`) จะถูก**ปฏิเสธพร้อม error บอกชื่อไฟล์** ไม่ถูกแปลความเป็น LAB เงียบ ๆ — ลบทิ้งแล้วจูนใหม่

เปลี่ยนที่เก็บไฟล์ได้ที่พารามิเตอร์ `tuned_path` (เช่นชี้เข้ามาใน repo ถ้าอยากให้ commit ตามไปด้วย)

> ถ้าป้ายชื่อ slider ว่างเปล่า แปลว่า OpenCV ตัวที่ติดตั้งไม่มีฟอนต์ของ Qt ติดมาด้วย (โหนดจะขึ้น log เตือนพร้อมคำสั่งแก้ให้) แก้ได้ด้วย:
> ```bash
> CV2=$(python3 -c "import cv2,os;print(os.path.dirname(cv2.__file__))")
> mkdir -p $CV2/qt/fonts && cp /usr/share/fonts/truetype/dejavu/DejaVuSans.ttf $CV2/qt/fonts/
> ```
> ถึงป้ายจะว่าง แถบค่าที่วาดอยู่ล่างภาพก็ยังบอกค่าจริงครบ จูนต่อได้อยู่ดี

### ก่อนรัน

ถ้ารันซ้ำหลายรอบ เช็กก่อนว่าไม่มี `sim_gait.py`, `robot_state_publisher` หรือ `gz sim` ค้างจากรอบก่อน (เคยทำให้ `arm_controller` มา activate ช้าจนหยิบไม่ทันแล้ว timeout ทุกสี) และถ้าเพิ่งรันคำสั่ง ROS CLI สั้น ๆ ไปหลายครั้งติดกัน ให้ลอง `rm -f /dev/shm/fastrtps_*` ถ้า node ใหม่หา node อื่นไม่เจอ (shared-memory ค้างจาก process ที่ตายไปแล้ว)

### ตรวจสอบว่าทำงานถูกต้อง

1. แท่นไม่ทับขาหุ่นและไม่ซ้อน `sim_skid_link` ลูกบาศก์วางนิ่งบนแท่น
2. `/color_detect/image_result` วาดกรอบครบทุกสี **กรอบเดียวต่อสี** จากท่ามอง (`joint4 = -1.55`) — ที่มุมก้มนี้ลูกบาศก์ตกแต่งที่ x=0.55 หลุดพ้นกรอบภาพไปทั้งหมด (คำนวณจากเรขาคณิตกล้อง) จึงไม่มีกรอบที่สองให้เห็น ยืนยันจาก `/yolo/object_detect` จริงบนฉากที่ยังไม่ถูกแตะต้อง: `red [69,110,234,262]`, `green [257,110,390,262]`, `blue [411,110,578,262]` — สามกรอบพอดี ไม่ใช่หก ถ้าเห็น**สองกรอบต่อสี**แสดงว่าแขนยังไม่ถึงท่ามอง (เช่น ยังอยู่ที่ท่าเริ่มต้น `init` ซึ่งก้มน้อยกว่า ลูกบาศก์ตกแต่งจะกลับเข้ามาในเฟรม — ดูหัวข้อ "กล้องเห็นลูกบาศก์สีเดียวกันสองลูก" ด้านบน)
3. ตำแหน่งที่ล็อกไว้ (ดูใน log ของ `pick_and_place`) ห่างจากตำแหน่ง spawn จริงไม่เกิน ~1 ซม.
4. `/grasp/<สี>/attach` ทำให้ลูกบาศก์ติดมือ และ `detach` ทำให้ตก
5. ลูกบาศก์ทั้งสามลูกไปอยู่ในช่องวางของตัวเอง (`drop_slots`) ห่างจากจุดกึ่งกลางช่องไม่เกิน 3 ซม. (วัดจริงซ้ำสองรอบได้ ~0.9 ซม.)

## 8. AprilTag (ตรวจจับป้ายและหาตำแหน่ง 3 มิติ)

ป้าย AprilTag บอกสองอย่าง: **มันคือป้ายหมายเลขอะไร** และ **มันอยู่ตรงไหนเทียบกับกล้อง** อย่างที่สองมีประโยชน์กว่ามาก เพราะได้ท่าทาง 6 แกนที่เอาไปจัดตำแหน่งหุ่นหรือวางของได้

```bash
ros2 launch rospider_gazebo pick_place.launch.py arm_pose:=horizontal
```

> **`arm_pose:=horizontal` สำคัญมาก** กล้องติดอยู่บนแขน ท่าเริ่มต้น `init` ก้มลงพื้น 52° ซึ่งเป็นท่าที่ใช้หยิบของ แต่มองไม่เห็นป้ายที่ตั้งอยู่ ถ้าลืมใส่จะไม่เจอป้ายเลยและดูเหมือนตัวตรวจจับพัง

### ดูผลลัพธ์

```bash
ros2 topic echo /apriltag_detect/apriltag_info --once
```
```
data:
- id: 0      # หมายเลขป้าย
  x: 320     # จุดกึ่งกลางป้ายในภาพ (พิกเซล)
  y: 310
  w: 80      # ความกว้างป้ายในภาพ (พิกเซล)
  d: 876     # ระยะห่าง (มิลลิเมตร)
```

ท่าทาง 6 แกนออกมาเป็น TF ชื่อ `tag_<id>`:

```bash
ros2 run tf2_ros tf2_echo depth_cam_frame tag_0 --ros-args -p use_sim_time:=true
```
```
- Translation: [0.000, 0.131, 0.876]
```

อ่านค่าใน optical frame: **z คือระยะไปข้างหน้า**, x คือซ้าย-ขวา, y คือบน-ล่าง (y บวก = ต่ำกว่ากล้อง) ดูภาพที่วาดกรอบและแกนแล้วได้ที่ `/apriltag_detect/image_result`

> ต้องใส่ `-p use_sim_time:=true` ไม่งั้น `tf2_echo` ใช้เวลาของเครื่องไปหา TF ที่ประทับเวลาของซิม แล้วจะหาไม่เจอตลอด

### จูน detector และกำหนดพฤติกรรมต่อป้าย (`tune:=true`)

```bash
ros2 launch rospider_gazebo pick_place.launch.py scene:=false tune:=true arm_pose:=horizontal auto_start:=false
```

`scene:=false` ข้ามการ spawn แท่นหยิบกับลูกบาศก์ — ใช้ตอนสาธิตแค่ป้าย (`approach`/`stop`) เพราะตำแหน่ง spawn ของหุ่นอยู่ห่างแท่นหยิบแค่ 1 ซม. เดินหน้าจากตรงนั้นชนแท่นแล้วตัวเอียงทันที ถ้าจะสาธิต `place` (ต้องหยิบลูกบาศก์ก่อน) ให้ตัด `scene:=false` ออกแล้วใช้ฉากเต็ม

เปิดหน้าต่าง `apriltag_detect tune` (Tk) ค่าเริ่มต้นเป็น **simple view** พอสำหรับกิจกรรม 10 นาที: เห็นป้าย → รู้ id กับระยะ → สั่งหุ่นว่าจะทำอะไรกับป้ายนั้น

| ส่วน | มีอะไร |
|---|---|
| ภาพซ้าย | overlay กล้องพร้อมกรอบและแกนของป้ายที่เห็น |
| **Tags** | แถวละหนึ่ง id: `action` (`none`/`approach`/`place`/`stop`) และ `standoff` (เมตร) แถวมาจาก `behaviors:` ใน YAML บวก id ที่เพิ่งเห็นจะโผล่มาเอง (ตั้งต้นที่ `none`) หรือพิมพ์เลขที่ช่อง **add id** แล้วกดปุ่ม |
| checkbox **Enable behaviors** | เปิด/ปิดพฤติกรรมทั้งหมด |
| บรรทัด status | ข้อความจาก `TagBehavior` (เช่น `tag 0: reached`) ต่อด้วยคำตอบล่าสุดของ `/pick_and_place/place` |
| ปุ่มล่าง | **Save** เซฟลง `~/.ros/apriltag_tuned.json` (เปลี่ยนที่เก็บด้วยพารามิเตอร์ `tuned_path`), **Revert** กลับไปค่าใน `config/apriltag.yaml`, **Print YAML** พิมพ์ค่าปัจจุบันเป็นบล็อก YAML ลง terminal เอาไปวางในไฟล์ได้เลย |
| ปุ่ม **Advanced ▸** | กดเพื่อเผยอีกสอง panel: **Detector** (slider ของ `cv2.aruco.DetectorParameters` — `thresh win min/max/step`, `thresh constant`, `min perimeter rate`, `polygon accuracy`, `corner refinement` none/subpix — บวก `max reproj error px`, เปลี่ยนแล้วมีผลกับเฟรมถัดไปทันที) และ **Control** (9 ค่า: `max_linear max_angular kp_yaw kp_dist yaw_deadband dist_deadband lost_timeout place_height turn_first_rad`) |

กติกาเดียวกับ `color_detect`: YAML เป็นค่าตั้งต้น JSON ทับทีละ key ลบ JSON แล้วกลับเป็น YAML ล้วน **พฤติกรรมเริ่มแบบปิดเสมอ** — checkbox `Enable behaviors` ไม่ถูกเซฟลง JSON (มีแต่ `behaviors_enabled: false` ใน YAML) ทุก launch จึงเริ่มปิดเสมอ ไม่มีไฟล์ที่ลืมไว้ทำให้หุ่นเดินเองตอนรัน demo อื่น

พฤติกรรม:

- **`approach`** — ถ้าป้ายอยู่นอกมุม `turn_first_rad` (0.35 rad ≈ 20°) จากแนวหน้าหุ่น หรืออยู่ด้านหลังกล้อง หุ่นจะหมุนอยู่กับที่ด้วย `max_angular` ก่อน ยังไม่เดินไปพร้อมกัน (กัน P-controller ถอยหลังพาหุ่นทะลุแท่นที่อยู่ข้างหลัง) พอป้ายอยู่ในมุมแคบแล้วจึงเดินเข้าหาด้วย P-controller สองตัว (หมุนตามแนวข้าง, เดินตามระยะ, clamp ที่ `max_linear`/`max_angular`) จน**ระยะจาก `base_footprint` ถึงป้ายเท่า `standoff`** แล้วหยุดนิ่ง — วัดจากตัวหุ่นตามแนวหัน ไม่ใช่จากกล้อง จึงไม่ขึ้นกับท่าแขน ในซิม `approach` ที่ standoff 0.4 หยุดจริงที่ฐานหุ่นห่างป้าย ~0.42 ม.
- **`place`** — เหมือน `approach` แต่พอถึงจะเรียก `/pick_and_place/place` หนึ่งครั้ง โดยคำนวณจุดบนแท่นของสถานีจากท่าป้าย (`tags.tag_to_pedestal_top()` บวก `place_height`) **ต้อง `~/pick` ก่อน** ให้หุ่นอยู่ในสถานะ `CARRY` ถ้า service ตอบ `not reachable` แปลว่า `standoff` ยาวไป ลดลงทีละ 0.05 แล้วปิด-เปิด checkbox ใหม่ (การปิด-เปิดรีเซ็ตสถานะให้เริ่มใหม่) — จะไม่ retry ให้เอง คำตอบล่าสุดของ service โชว์ในบรรทัด status
- **`stop`** — ส่ง twist ศูนย์ตลอดที่เห็นป้ายนี้ ชนะทุก action อื่น (ใช้เป็นป้ายห้ามเข้า)
- เห็นหลายป้าย → เลือกป้ายที่ใกล้สุดที่ action ไม่ใช่ `none`
- ป้ายหายไป → ยืนนิ่ง `lost_timeout` วินาที (กันป้ายกะพริบหลุดเฟรมเดียว) แล้วส่ง twist ศูนย์อีกหนึ่งครั้งจากนั้น**เลิกยุ่งกับ `/controller/cmd_vel`** ให้ teleop/Nav2 ใช้ต่อได้

ตรรกะทั้งหมดอยู่ใน `rospider_gazebo/tag_behavior.py` (ไม่มี ROS) มีเทสต์ `test/test_tag_behavior.py` คุมทิศทางการหมุน, deadband, การเลี้ยวก่อนเดินเมื่อมุมเกิน `turn_first_rad`, การยิง `place` ครั้งเดียว และลำดับการปล่อย `cmd_vel`

### ป้ายจำได้ (tag memory) — ทำไม `place` ยังทำงานตอน CARRY

กล้องติดอยู่ที่ข้อมือ (`link4`) พอ `~/pick` สำเร็จเข้าสถานะ `CARRY` ลูกบาศก์ที่คีบอยู่จะบังกล้องจนป้ายไม่เข้าเฟรมเลย ไม่ว่า standoff จะเป็นเท่าไหร่ — `place` จึงพึ่งการเห็นสดไม่ได้

โหนดจำท่าทางของทุกป้ายที่เคยเห็นไว้ในเฟรม `memory_frame` (พารามิเตอร์ ตั้งต้น `odom`, ใส่ `''` เพื่อปิดการจำ) พอเปิด behaviours แล้ว ป้ายที่ตั้ง action เป็น `approach`/`place` ที่ไม่อยู่ในเฟรมปัจจุบันแต่มีอยู่ในความจำ โหนดจะสร้าง "การเห็นเสมือน" จากท่าที่จำไว้ผ่าน TF แล้วส่งให้ `TagBehavior` เหมือนเห็นจริง (บรรทัด status จะมีคำว่า `[remembered]` ต่อท้าย) ป้าย `stop` ไม่ถูกจำ — ป้ายห้ามเข้าที่จำไว้แต่หุ่นมองไม่เห็นจริงจะทำให้หุ่นหยุดแช่ตลอดไป

ผลที่ตามมา: เมื่อเปิดการจำ ป้ายที่วางสำเร็จแล้วจะไม่มีวัน "หลุดจากสายตา" ในทางความจำ ดังนั้น `place` จะยิงแค่ครั้งเดียวต่อการเปิดสวิตช์หนึ่งครั้ง — ต้องปิดแล้วเปิด checkbox `Enable behaviors` ใหม่เพื่อให้ยิงได้อีกครั้ง

> **ป้ายไม่มีวันหายจริง ๆ เท่ากับ `/controller/cmd_vel` ก็ไม่มีวันถูกปล่อยเหมือนกัน** ข้อความก่อนหน้านี้ในหัวข้อพฤติกรรมที่ว่าป้ายหายไปแล้วหุ่นจะยืนนิ่ง `lost_timeout` วินาทีแล้วปล่อย `cmd_vel` คืนให้ teleop/Nav2 นั้น จริงแค่ตอน `memory_frame: ''` (ปิดการจำ) หรือป้ายนั้นไม่เคยถูกเห็นมาก่อนเลยเท่านั้น เรื่องนี้ใช้กับ `approach` เหมือนกัน ไม่ใช่แค่ `place` — พอเปิดการจำแบบ default (`odom`) แล้วป้ายเคยถูกเห็นสักครั้ง ป้ายที่ตั้ง action เป็น `approach`/`place` จะถูกเรียกคืนเป็นการเห็นเสมือนทุกเฟรมตลอดไป พฤติกรรมจึงไม่มีวัน "lost" และโหนดจะยัง publish `/controller/cmd_vel` ที่ความถี่กล้องเรื่อย ๆ (เดินเข้าหา, ยืนนิ่งที่ standoff, หรือยืนนิ่งหลัง `place`) จนกว่าจะปิด checkbox `Enable behaviors`

> ทำไมต้องวัดจาก `base_footprint`: กล้องบนข้อมือหันลงและหันข้างระหว่าง `CARRY` แกน x/z ของกล้องตอนนั้นไม่บอกอะไรเกี่ยวกับทิศหุ่นเลย ถ้าคุมด้วยพิกัดกล้องตรง ๆ หุ่นจะ "ถึง standoff" ที่ตำแหน่งไม่มีความหมาย โหนดจึงแปลงทุกท่าป้าย (จริงหรือจำ) ผ่าน TF `base_footprint <- <camera frame>` ก่อนเสมอ ถ้า TF หาไม่เจอ พฤติกรรมยังทำงานต่อด้วยเฟรมว่าง (ไม่มีป้ายให้เห็นเลย) เพื่อให้ลำดับ "ยืนนิ่ง แล้วปล่อย `cmd_vel`" ของป้ายที่หายยังทำงานอยู่ ไม่ใช่ปล่อยให้ twist ล่าสุดค้างอยู่เฉย ๆ — ไม่คุมด้วยพิกัดกล้องดิบเด็ดขาด

### `place` ทำได้แค่ไหนจริง ๆ — พูดตรง ๆ

`place` เดินเข้าหาด้วยความจำแล้วเรียก `/pick_and_place/place` — ใช้ได้ดีเมื่อหุ่นอยู่ใกล้สถานีอยู่แล้ว (แค่เดินตรงระยะสั้น ๆ) **ไม่ได้ออกแบบมาให้เดินอ้อมสิ่งกีดขวางเป็นระยะไกลด้วยความจำ** — ทดสอบในฉาก pick_place เต็มพบว่าหุ่นชนแท่นหกล้มระหว่างทาง แผน launch แบบรวม Nav2 ในอนาคตจะให้ Nav2 พาหุ่นไปถึงสถานี แล้วให้ `place` ทำแค่ก้าวสุดท้าย

### สั่งค่าสดด้วย `ros2 param set`

ใช้ได้กับ `behaviors_enabled`, `behaviors.tagN.action`, `behaviors.tagN.standoff`, `control.*`, `detector.*`, `max_reproj_error_px` เช่น

```bash
ros2 param set /apriltag_detect behaviors_enabled true
```

GUI ไม่อ่านค่ากลับมาแสดงให้ ถ้าไปแตะ widget ใดหลังจากนั้น widget จะเขียนทับด้วยค่าที่มันถืออยู่ ไม่ใช่ค่าที่เพิ่ง set

> **จุดพลาดที่รู้อยู่แล้ว (1):** slider `min perimeter rate` มีช่วงตามที่ออกแบบ 0-0.2 แต่ป้ายขนาด 0.9 ม. (ระยะ spawn ปกติ) จะหายจากภาพจริงต่อเมื่อค่าดันเกิน ~0.55 ซึ่งอยู่นอกช่วง slider ทั้งหมด — เลื่อนสุด slider แล้วป้ายจะยังไม่หายไปไหน

> **จุดพลาดที่รู้อยู่แล้ว (2):** ที่ระยะใกล้มาก (< ~0.3 ม. จาก `base_footprint`) `thresh constant` ค่าเริ่มต้นอาจทำให้หาป้ายสดไม่เจอเลย — tag memory ช่วยคลุมช่วงนี้ได้ (โหนดยังใช้ท่าที่จำไว้ต่อ)

### เพิ่มสถานีใหม่

สองขั้นตอน สั่งสร้างไฟล์ แล้วบอกตำแหน่ง

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider/src/simulations/rospider_gazebo
python3 tools/make_tag_textures.py 1 2
```

คำสั่งเดียวได้ทั้ง `worlds/textures/tag_1.png` และ `models/tag_station_1/model.sdf` แล้วแก้ `config/apriltag.yaml`:

```yaml
    stations:
      station0: [0.9, 0.0, 3.14159]   # [x, y, yaw] ในกรอบ world
      station1: [0.0, 1.2, -1.5708]
```

ชื่อ key บอกหมายเลขป้าย (`station1` → `tag_station_1` → `tag_1.png`) ตัวเลข yaw คือหันหน้าไปทางไหน — โมเดลหันหน้าไปทาง **+x** ตอน yaw = 0 หุ่นที่เข้ามาจากทาง +x จะเห็นป้าย

พิกัดสถานีอยู่ในไฟล์ config ไฟล์เดียว **ไม่ได้เขียนลงไฟล์ world** ดังนั้นย้ายไปใช้กับโลกที่คุณออกแบบเองได้โดยแก้ที่เดียว

ปิดตัวตรวจจับและไม่ต้อง spawn สถานี: `tags:=false`

### ขนาดป้ายกับระยะที่อ่านได้

ป้ายกว้าง 0.15 ม. กล้องซิมมี `fx = 467.7 px` ความกว้างในภาพจึงเป็น `467.7 × 0.15 / ระยะ`:

| ระยะ | ความกว้างในภาพ | ใช้ได้แค่ไหน |
|---|---|---|
| 0.3 ม. | 234 px | อ่าน id และท่าทางได้แม่น |
| 1.0 ม. | 70 px | ดี |
| 2.0 ม. | 35 px | อ่าน id ได้ ท่าทางเริ่มไม่แม่น |

ป้าย 36h11 กว้าง 10 ช่องรวมขอบดำ ที่ 35 px จึงเหลือช่องละ 3.5 px

ถ้าจะเปลี่ยนขนาดป้าย **ต้องแก้ที่ `rospider_gazebo/tags.py`** แล้วสร้าง texture กับโมเดลใหม่ ไม่ใช่แก้ `tag_size` ใน `config/apriltag.yaml` อย่างเดียว — ขนาดบอร์ดคำนวณมาจาก `TAG_SIZE` ถ้าสองค่าไม่ตรงกัน ระยะทุกค่าที่รายงานจะผิดตามอัตราส่วนนั้นโดยไม่มีอาการอื่นให้เห็น (มีเทสต์ `test_board_face_matches_the_generated_texture` คุมไว้)

### ป้ายหายไปกลางคัน — ไม่ใช่บั๊ก

กล้องอยู่บนแขน พอ `pick_and_place` สั่งแขนไปท่ามอง (`look_pose`) หรือท่าหยิบ กล้องก็หันไปทางอื่น ป้ายหายจากเฟรมเป็นเรื่องปกติ

โหนดจะ publish `apriltag_info` ที่มี `data` ว่างต่อไปเรื่อย ๆ **ไม่ใช่หยุด publish** เพื่อให้แยกออกว่า "มองอยู่แต่ไม่เจอ" ต่างจาก "โหนดตายแล้ว" และ TF `tag_<id>` จะไม่ถูกประกาศใหม่ ผู้ใช้ต้องเช็ค timestamp เอง ไม่ใช่เชื่อว่าค่าล่าสุดสดเสมอ

## 9. YOLO (ตรวจจับวัตถุแบบเทรนเองได้)

`color_detect` กับ `yolo_detect` **ปล่อยของอย่างเดียวกันเป๊ะ** — `interfaces/ObjectsInfo` บน `/yolo/object_detect` สลับกันได้โดย `pick_and_place` ไม่ต้องแก้อะไรเลยแม้แต่บรรทัดเดียว

```bash
ros2 launch rospider_gazebo pick_place.launch.py detector:=yolo
```

ทั้งสองตัว**ไม่เคยรันพร้อมกัน** เลือกได้ทีละตัว

| | `detector:=color` (ค่าเริ่มต้น) | `detector:=yolo` |
|---|---|---|
| ต้องลงอะไรเพิ่ม | ไม่ต้อง | torch + ultralytics |
| เพิ่มวัตถุใหม่ | ปรับช่วง LAB (หัวข้อ 7) | เก็บภาพแล้วเทรน |
| ทนต่อแสง/เงา | ปานกลาง | ดีกว่า |
| จำแนกได้จาก | สีอย่างเดียว | รูปร่างและลวดลาย |

### ติดตั้ง (เฉพาะทาง YOLO)

python3 ของเครื่องนี้เป็นแบบ externally-managed (PEP 668) และของอื่นในโปรเจกต์นี้ (opencv 5.0, ultralytics) ก็ลงไว้ที่ user site ซึ่งเป็นที่ที่ node ที่ `ros2 launch` เรียกจะมองเห็น — **venv จะมองไม่เห็น** ดังนั้น:

```bash
pip install --user --break-system-packages -r requirements-yolo.txt
```

แล้วลง torch แยกต่างหาก เพราะเลือก build ตามการ์ด:

```bash
# การ์ด Blackwell เช่น RTX 50xx ต้องใช้ cu128 ขึ้นไป wheel ธรรมดาใช้ไม่ได้
pip install --user --break-system-packages torch torchvision     --index-url https://download.pytorch.org/whl/cu128
```

ถ้าไม่มี GPU หรือไม่อยากโหลด 3 GB ใช้ CPU ก็ได้ (inference พอไหว เทรนจะช้ามาก):

```bash
pip install --user --break-system-packages torch torchvision     --index-url https://download.pytorch.org/whl/cpu
```
แล้วตั้ง `device: cpu` ใน `config/yolo.yaml`

> ถ้ายังไม่ได้ลง โหนดจะตายพร้อมข้อความบอกวิธีลง ไม่ใช่ traceback ของ import — ตั้งใจให้เป็นแบบนั้น

> ⚠️ **ต้องลง torch ก่อน แล้วค่อยลง `requirements-yolo.txt`** ลำดับสำคัญ เพราะการลง torch จะดึง numpy 2.x เข้ามาด้วย และไฟล์ requirements ตรึง `numpy<2` ไว้ ต้องให้อันหลังชนะ
>
> **ทำไมต้องตรึง numpy** `cv_bridge` ของ ROS Jazzy เป็น C++ extension ที่คอมไพล์กับ numpy 1.x พอ numpy 2 มาอยู่ที่ user site มันจะบัง numpy 1.26.4 ของระบบ แล้ว `import cv_bridge` พังทันที — **ทำให้ `color_detect` กับ `pick_and_place` ตายไปด้วย ไม่ใช่แค่ YOLO** วัดจริงแล้ว: numpy 2.5.3 = โหนดกล้องทุกตัวตายตั้งแต่ import, numpy 1.26.4 = cv2 5.0, cv_bridge, torch + CUDA, ultralytics ทำงานร่วมกันได้หมด
>
> pip จะเตือนว่า `opencv-python 5.0 requires numpy>=2` **ไม่ต้องสนใจ** ข้อกำหนดนั้นไม่ได้บังคับตอนรัน และ cv2 5.0.0 ทำงานกับ 1.26.4 ได้ปกติ ซึ่งเป็นสภาพที่ workspace นี้ใช้มาตลอด
>
> อาการเวลาเจอ: `ImportError: A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x` แก้ด้วย `pip install --user --break-system-packages 'numpy<2'`

### ตั้งค่า

`config/yolo.yaml`:

| ค่า | ความหมาย |
|---|---|
| `model_path` | ไฟล์โมเดล path เต็มก็ได้ หรือชื่อที่ ultralytics โหลดเองได้ (`yolo11n.pt`) |
| `task` | `detect` = กรอบตรง, `obb` = กรอบเอียง — `pick_and_place` อ่านได้ทั้งคู่ |
| `conf` | ความมั่นใจขั้นต่ำ |
| `device` | `''` ให้เลือกเอง, `'cpu'` บังคับ CPU, `'0'` บังคับ GPU ตัวแรก |
| `classes` | ว่าง = ปล่อยทุกคลาส ใส่รายชื่อเพื่อกรอง |

> **ชื่อคลาสต้องอยู่ใน `colors` ของ `config/pick_place.yaml` ด้วย** ไม่งั้น `pick_and_place` จะไม่ยอมหยิบ มันเช็กชื่อที่รับเข้ามากับรายการนั้น

### จูนสด ๆ ด้วยหน้าต่าง (`tune:=true`)

```bash
ros2 launch rospider_gazebo pick_place.launch.py detector:=yolo tune:=true auto_start:=false
```

เปิดหน้าต่าง `yolo_detect tune` (Tk): ซ้ายคือภาพกล้องพร้อมกรอบและคะแนน ขวามีแค่สองอย่างที่ควรแตะในเวิร์กช็อป

| | ทำอะไร |
|---|---|
| **confidence** | ความมั่นใจขั้นต่ำ ลดลงถ้าโมเดลเห็นของแต่ไม่กล้าบอก เพิ่มขึ้นถ้ามันเห็นผี |
| **classes to publish** | checkbox ต่อคลาสที่โมเดลรู้จัก ติ๊กออก = ไม่ส่งคลาสนั้นไป `/yolo/object_detect` เลย (`pick_and_place` จะไม่เห็น) ติ๊กครบทุกอัน = ไม่กรอง |

บรรทัดสถานะบอกว่าเจอกี่ชิ้น ใช้เวลากี่ ms และกำลังปล่อยคลาสไหนอยู่ ระหว่างจูนโหนดยัง publish ตามปกติ ปิดหน้าต่างแล้วโหนดก็ยังตรวจจับต่อด้วยค่าล่าสุด

ปุ่ม **Save** เซฟ `conf` กับ `classes` ลง `~/.ros/yolo_detect_tuned.json` (เปลี่ยนที่เก็บด้วยพารามิเตอร์ `tuned_path`) ซึ่งจะทับ `config/yolo.yaml` ตอนเปิดครั้งถัดไป — กติกาเดียวกับ `color_detect` และ `apriltag_detect`: YAML เป็นค่าตั้งต้น JSON ทับ ลบ JSON แล้วกลับเป็น YAML ล้วน โหนด log บอกเสมอว่าใช้ไฟล์ไหนอยู่ **Revert** กลับไปค่า YAML, **Print YAML** พิมพ์บล็อกที่วางลง `config/yolo.yaml` ได้เลย

> หน้าต่างนี้**ไม่ได้**เพิ่มคลาสใหม่ให้ — โมเดลที่เทรนมารู้จักแค่คลาสที่เทรน ของใหม่ต้องเก็บภาพแล้วเทรนใหม่ (หัวข้อถัดไป)

### YOLO ใช้เวลาอุ่นเครื่องตอนเริ่ม

โหนดจะรัน inference กับภาพเปล่าหนึ่งครั้ง **ก่อน** สมัครรับภาพจากกล้อง แล้ว log ว่า `warmed up in 5.2s` เพราะ inference ครั้งแรกต้องคอมไพล์ CUDA kernel ใช้เวลาราว 5–10 วินาที ส่วนครั้งต่อ ๆ ไปใช้ 19 ms

ถ้าไม่ทำแบบนี้ โหนดจะรับภาพแล้วเงียบไปสิบวินาที ซึ่ง `pick_and_place` จะยอมแพ้ไปแล้ว (`state_timeout` = 15 วิ) แล้วขึ้น `never saw red; skipping` — ดูเหมือนตรวจจับไม่ได้ ทั้งที่จริงแค่ยังไม่ทันพร้อม

### ลูกบาศก์ตกแต่งไม่ได้อยู่ใน dataset

ในโลกมีลูกบาศก์ตกแต่งสีเดียวกันสามใบที่ x=0.55 (ดูหัวข้อ "กล้องเห็นลูกบาศก์สีเดียวกันสองลูก") `capture_dataset.py` ไม่ได้ label ให้ เพราะไม่ได้อยู่ใน `OBJECTS` — โมเดลจึงเรียนว่าเป็นพื้นหลัง

**ต่างจาก `color_detect` ที่ตรวจเจอทั้งคู่** ผลคือ YOLO ไม่ส่งกรอบของลูกบาศก์ตกแต่งออกมาเลย ซึ่งสำหรับการหยิบถือว่าดีกว่า (ของพวกนั้นเอื้อมไม่ถึงอยู่แล้ว) แต่ถ้าอยากให้พฤติกรรมเหมือนกันเป๊ะ ต้องเพิ่มเข้า `OBJECTS` แล้วเก็บ dataset ใหม่

### เก็บข้อมูลเทรนเอง (ไม่ต้องลากกรอบเอง)

เปิดซิมไว้ก่อน แล้วรันเครื่องมือเก็บข้อมูลอีกหน้าต่าง:

```bash
ros2 launch rospider_gazebo pick_place.launch.py auto_start:=false tags:=false
```
```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider/src/simulations/rospider_gazebo
python3 tools/capture_dataset.py --samples 50 --out ~/datasets/cubes
```

เครื่องมือจะสุ่มย้ายลูกบาศก์ รอให้หยุดนิ่ง **อ่านตำแหน่งจริงกลับมา** แล้วคำนวณกรอบจากเรขาคณิต — ฉายมุมกล่องทั้ง 8 จุดผ่าน `camera_info` และ TF จริง ได้ label ถูกต้อง 100% โดยไม่ต้องลากกรอบเอง

> **ทำไมต้องอ่านตำแหน่งกลับ** ตอนแรกผมเขียนให้เชื่อตำแหน่งที่สั่งไป เพราะ "เราวางเอง เราก็รู้" — **ผิด** ลูกบาศก์เป็นวัตถุที่มีฟิสิกส์ สั่งไป `(0.28, 0.11, 0.105)` วัดได้จริงว่าไปจบที่ `(0.299, 0.110, 0.025)` คือไถล 2 ซม. แล้วตกจากแท่นลงพื้น ตอนวาด label กลับลงภาพเห็นชัดว่ากรอบเลื่อนจากลูกบาศก์ทุกใบ

**ตรวจ label ด้วยตาทุกครั้งก่อนเทรน** — dataset ที่ผิดจะเทรนสำเร็จเงียบ ๆ แล้วได้โมเดลที่มั่นใจแต่ผิด:

```bash
python3 - <<'EOF'
import glob, cv2
D = '/home/YOURNAME/datasets/cubes'
for path in sorted(glob.glob(f'{D}/images/*/*.jpg'))[:5]:
    img = cv2.imread(path); h, w = img.shape[:2]
    lab = path.replace('/images/', '/labels/').replace('.jpg', '.txt')
    for line in open(lab):
        c, x, y, bw, bh = line.split()
        x, y, bw, bh = float(x)*w, float(y)*h, float(bw)*w, float(bh)*h
        cv2.rectangle(img, (int(x-bw/2), int(y-bh/2)),
                      (int(x+bw/2), int(y+bh/2)), (0,255,0), 2)
    out = '/tmp/check_' + path.split('/')[-1]
    cv2.imwrite(out, img); print(out)
EOF
```

กรอบต้องทาบบนวัตถุพอดี ถ้าเลื่อนเท่ากันทุกภาพแปลว่า TF หรือการฉายภาพผิด ถ้ามีกรอบบนของที่ถูกบังอยู่แปลว่า occlusion check ไม่ทำงาน

### เทรน

```bash
python3 tools/train_yolo.py --data ~/datasets/cubes --name cubes
```

ได้ `models/yolo/cubes.pt` แล้วชี้ `model_path` ใน `config/yolo.yaml` มาที่ไฟล์นี้

ค่าเริ่มต้นคือ 100 epochs, batch 8, เริ่มจาก `yolo11n.pt` (เปลี่ยนด้วย `--epochs --batch --base`)

**ต้องใช้กี่รูป** — วัดจริงในซิม (mAP50 บนชุด val 48 รูปเดียวกัน, เริ่มจาก `yolo11n.pt`, RTX 5060):

| รูปที่ใช้เทรน | 30 epochs | 100 epochs | เวลาเทรน 100 ep (GPU) |
|---|---|---|---|
| 5 | 0.18 | 0.50 | ~1 นาที |
| 10 | 0.31 | 0.80 | ~1 นาที |
| 20 | 0.65 | **0.90** | ~1 นาที |
| 181 | — | 0.99 | ~2 นาที |

ฉากในซิมแสงคงที่ **10–20 รูป + 100 epochs ก็พอใช้งาน** 5 รูปยังเดาผิดบ่อยเกินจะให้หยิบ และเมื่อรูปน้อย epochs สำคัญกว่าจำนวนรูป (20 รูป 30 ep ได้แค่ 0.65) ค่า `--samples 50` ด้านบนจึงเผื่อไว้แล้ว ส่วนหลายร้อยรูปซื้อแค่ความทนทานต่อมุมที่ชุดเล็กบังเอิญไม่มี

> การเทรนต่อจาก `models/yolo/cubes.pt` (`--base`) ได้ 0.95 ตั้งแต่ 5 รูป แต่นั่นเพราะมันรู้จักลูกบาศก์สามสีนี้อยู่แล้ว — ไม่ใช่หลักฐานว่าเรียน "ของใหม่" ได้จากรูปน้อย

**ถ้าไม่มี GPU:** 20 รูป × 100 epochs บน CPU (16 เธรด) ใช้ **25 นาที** (1483 วิ, mAP50 0.895 เท่ากัน) — โน้ตบุ๊ก 4–8 เธรดจะนานกว่านั้นราว 2–3 เท่า ในเวิร์กช็อปจึงควรมีโมเดลที่เทรนไว้แล้วสำรอง หรือลด `--epochs 50` ยอมได้ mAP ต่ำลงบ้าง

ค่า mAP50 ที่ต่ำกว่าตารางมากทั้งที่รูปเท่ากัน มักแปลว่า label ผิด ไม่ใช่ epochs น้อย กลับไปตรวจภาพก่อน

### เพิ่มวัตถุใหม่

1. แก้ `OBJECTS` ใน `tools/capture_dataset.py` — ใส่ชื่อโมเดลใน Gazebo, ชื่อคลาส และขนาดกล่อง
2. เก็บข้อมูลใหม่ แล้วเทรนใหม่
3. เพิ่มชื่อคลาสลงใน `colors` ของ `config/pick_place.yaml` ถ้าอยากให้หยิบได้

> ถ้าอยากให้**หยิบ**ของใหม่ได้ด้วย ยังต้องเพิ่ม `DetachableJoint` ใน `urdf/rospider_gazebo.urdf.xacro` ไฟล์โมเดล SDF และ entry ใน `config/gz_bridge.yaml` อีก — ปลั๊กอินนั้นผูก `child_model` ตายตั้งแต่ตอนโหลด หนึ่งปลั๊กอินต่อหนึ่งวัตถุ (ยังไม่ได้ทำให้ง่ายกว่านี้ในรอบนี้)

### ใช้ dataset ที่ label เอง

`train_yolo.py` รับ path เดียวกัน ไม่ว่า dataset จะมาจากเครื่องมือข้างบนหรือจาก Roboflow / labelImg — ทั้งคู่ export เป็นโครงสร้าง YOLO เหมือนกัน ขอแค่มี `data.yaml` ที่บอกชื่อคลาสและโฟลเดอร์ train/val

## 10. Mini game (หยิบ → ขนผ่านสนาม → วางตามป้าย)

เกมสำหรับช่วงบ่ายของเวิร์กช็อป: หุ่นเริ่มที่ต้นโถง หยิบลูกบาศก์สีพาสเทล 3 ลูก ขนผ่านสนามรูปตัว S ไปวางบนสถานีปลายทางที่มีป้าย AprilTag ซึ่งสถานีไหนรับสีไหน**สุ่มทุกรอบ** — หุ่นต้องอ่านเอง

```bash
ros2 launch rospider_gazebo mini_game.launch.py detector:=color seed:=3
```

| arg | ค่า | ความหมาย |
|---|---|---|
| `detector` | `color` / `yolo` | ใครเป็นคนตรวจจับลูกบาศก์ |
| `color_config` | `color_detect_arena.yaml` | ช่วง LAB เริ่มต้น (ตั้งใจให้**ยังจับไม่ได้** ทีมต้องจูน) — `color_detect_arena_solved.yaml` คือเฉลยของวิทยากร |
| `model` | `models/yolo/cubes.pt` | โมเดล YOLO — ตัวที่แจกมารู้จักแค่ลูกบาศก์ RGB ทีม YOLO ต้องเทรนใหม่ (ดูด้านล่าง) |
| `seed` | `0` | สุ่มว่าสถานีไหนได้สีอะไร |
| `mission` | `basic` | ไฟล์ภารกิจใน `config/missions/` |
| `auto_start` | `true` | `false` = รอ `ros2 service call /mission/start std_srvs/srv/Trigger` |
| `tune`, `rviz`, `gui` | | เหมือน launch อื่น — `tune:=true` เปิดหน้าต่างของ detector และ AprilTag; ไฟล์ที่เซฟจากในเกมแยกจากช่วงสาธิตตอนเช้า (`~/.ros/*_game_tuned.json`, `color_detect_arena_tuned.json`) |

### สนาม `worlds/arena.sdf`

โถง 6 × 2 ม. ทางเดินเป็นตัว S: แผงเฉียงหลังจุดเริ่ม → ผนังกั้น A จากด้านเหนือ (ช่อง 0.8 ม. ทางใต้) → ลังในมุมห้องกลาง → ผนังกั้น B จากด้านใต้ (ช่อง 0.8 ม. ทางเหนือ) → จุดสำรวจ (3.0, 0.3) → กล่องด้านใต้ → เสาสองต้น → สถานี 3 จุดเรียงบนผนังท้ายที่ x 5.1 ไฟล์ world เป็นโถงเปล่า **ของในเกมทั้งหมด spawn จาก launch** (แท่น ลูกบาศก์ สถานี) จึงทำแผนที่ได้ด้วย SLAM/V-SLAM ปกติ: `slam.launch.py world:=<path>/arena.sdf` แล้ว `map_saver`

แผนที่ที่ทำไว้ให้อยู่ที่ `maps/arena.yaml` (ผนังและเสาแต่งด้วยมือให้ทึบ) **ทีมทำแผนที่เองได้** ด้วย SLAM หรือ V-SLAM แล้วส่ง `map:=<ไฟล์ .yaml ของทีม>` — launch จะ**วาดแท่นหยิบของลงในแผนที่ให้เอง**ตอนเปิด (แท่นสูง 8 ซม. ต่ำกว่าระนาบสแกน 15 ซม. LiDAR จึงไม่เคยเห็น ถ้าไม่วาด Nav2 จะขับชนแล้วหุ่นหงาย) ไม่ต้องแก้ไฟล์แผนที่ด้วยมือ:

```bash
ros2 launch rospider_gazebo slam.launch.py world:=$PWD/src/simulations/rospider_gazebo/worlds/arena.sdf   # หรือ vslam.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/my_arena --ros-args -p use_sim_time:=true
ros2 launch rospider_gazebo mini_game.launch.py map:=~/my_arena.yaml
```

**Nav2 ในสนามนี้ต่างจากห้องเดโม** (ทับใน launch ไม่ได้แก้ `nav2_params.yaml`): controller เป็น **Regulated Pure Pursuit** แทน DWB — DWB ในช่อง 0.8 ม. หา trajectory ไม่ได้เลย นั่งหมุน/ถอยจนหมดเวลา (วัด: 600 วิ ได้ 2.3 ม.), `robot_radius` 0.15 (ครึ่งแนวทแยงของกล่อง skid; 0.10 มุมกล่องเกี่ยวปลายผนังแล้วหงาย), `inflation_radius` 0.18, ความเร็ว 0.15 m/s (เดินจริง ~0.10), goal tolerance 0.15 ม., และ **static layer ใน local costmap** เพื่อให้เห็นแท่นที่วาดในแผนที่

### ลูกบาศก์พาสเทลและสถานี

ลูกบาศก์ `pink` / `yellow` / `sky` เป็นสีพาสเทล ใน LAB จึงห่างจากสีเทาแค่ 8–15 หน่วยบนแกน chroma (วัดในสนาม: pink A 135–140, yellow B 140–148, sky B 120–123 เทียบพื้นสีเทาอ่อน A/B 127–130) ส่วนช่วงเริ่มต้นใน `color_detect_arena.yaml` ตั้งขอบ chroma ไว้ไกลออกไปอีก 40 จึง**มองไม่เห็นเลย** — โจทย์ของทีมสีคือเลื่อน slider ตัวเดียวต่อสี (`A min` ของ pink, `B min` ของ yellow, `B max` ของ sky) เข้าหาสีเทาจนเห็นลูกบาศก์ แต่ไม่ถึงกับให้พื้นโผล่ (`L min` 100 ที่ตั้งไว้ตัดโปสเตอร์บนผนังซึ่งมืดกว่าออกไปแล้ว) แต่ละสถานีมี**แผ่นสี**ขนาด 0.2 ม. เหนือป้าย สีเดียวกับลูกบาศก์ที่ควรมาวางที่นี่ (ตั้งแผ่นแนวตั้งและเรืองแสงในตัว ไม่ใช่ทาพื้น เพื่อให้กล้องระดับสายตาอ่านได้จากไกลไม่ว่าแสงจะส่องด้านไหน) `stations.station_sdf(id, marker_rgba, tag_size)` สร้าง SDF ให้ตอน launch — ไม่มีไฟล์โมเดลต่อสี **ป้ายในเกมใหญ่ 0.30 ม.** (เดโมใช้ 0.15): จากจุดสำรวจ 2 ม. ป้าย 0.15 กว้างแค่ 30 px อ่านไม่ได้เลย launch จึงส่ง `tag_size` ให้ `apriltag_detect` ด้วย

### ภารกิจ = บล็อกใน YAML

`config/missions/basic.yaml` คือสิ่งที่ทีมแก้: ลำดับบล็อก, waypoint, `standoff`, ชื่อสี

```yaml
steps:
  - survey: survey          # ไปจุดสำรวจ หันซ้าย-กลาง-ขวา อ่านป้าย + แผ่นสี → ตาราง "ป้ายไหน = สีอะไร"
  - goto: pick_table
  - pick: pink              # คืบไปจุด dock ที่แน่นอน (pick_table + dock), /pick_and_place/pick
                            # รอจน CARRY แล้วถอยกลับ pick_table
  - deliver: by_marker      # Nav2 ไปหน้าสถานีที่แผ่นสีตรงกับลูกที่ถือ (ตำแหน่งจากท่าป้ายที่จำไว้)
                            # แล้ว apriltag_detect เดินเข้า standoff และเรียก ~/place ให้
  - goto: home
```

`pick` ที่ยังถือของอยู่ (เพราะ `deliver` ก่อนหน้าล้ม) จะวางลูกนั้นลงพื้นตรงนั้นก่อน (ไม่ได้คะแนน) แล้วทำต่อ ไม่ให้บล็อกที่เหลือล้มทั้งหมด

บล็อกที่มี: `goto`, `survey`, `pick`, `deliver` (`by_marker` หรือเลขป้าย), `place: here`, `say` ไฟล์ที่เขียนผิดจะถูกปฏิเสธตอน launch พร้อมบอกว่าบล็อกไหนผิด `on_fail: skip | retry | stop` กำหนดว่าบล็อกล้มแล้วทำอะไร โหนด `mission` log ทุกบล็อกเป็น `[k/n] block ... ok/FAILED (t s)` และสรุปเวลาทั้งหมดใน `/mission/summary`

**ทำไมต้องสำรวจก่อน:** ตอนถือของกล้องถูกลูกบาศก์บัง (หัวข้อ 8) หุ่นจึงต้องเห็นป้ายทุกใบ*ก่อน*หยิบ สีของแผ่นป้ายโหนด `mission` แยกสีเอง (`mission_plan.marker_boxes`, ช่วง HSV คงที่ในโค้ด เพราะแผ่นเรืองแสงสีคงที่) — **ไม่ได้ใช้ detector ของทีม** ทีม YOLO ที่เทรนแต่ลูกบาศก์จึงไม่ถูกหักคะแนนตอนสำรวจ; detector ของทีมมีผลตอนหยิบเท่านั้น จุด `survey` (3.0, 0) มองเห็นสถานีทั้งสามพร้อมกันในเฟรมเดียว ท่าป้ายถูกจำไว้เป็น TF `tag_<id>_remembered` แล้ว `deliver` คำนวณเป้าหมาย Nav2 จากมัน (ถอยจากป้ายมา `standoff` + 0.3 ม. หันหน้าเข้าป้าย) ทีมจึง**ไม่ต้องปักพิกัดสถานีเอง**

### ทีม YOLO

```bash
ros2 launch rospider_gazebo mini_game.launch.py auto_start:=false arm_pose:=init      # กล้องก้มมองแท่น
python3 tools/capture_dataset.py --objects pastel --world arena --dock 0.5 --samples 20 --out ~/datasets/pastel
python3 tools/train_yolo.py --data ~/datasets/pastel --name pastel        # ~1 นาทีบน GPU
ros2 launch rospider_gazebo mini_game.launch.py detector:=yolo model:=models/yolo/pastel.pt
```

`--dock 0.5` ให้เครื่องมือเดินหน้าจากจุด spawn ไปยืนที่เดียวกับที่ mission หยิบ (ห่างลูกบาศก์ 0.235 ม.) ก่อนเก็บภาพ และถอยกลับเมื่อเสร็จ

(20 รูป × 100 epochs พอสำหรับซิม ดูตารางในหัวข้อ 9) `models/yolo/pastel.pt` ที่แจกมาคือเฉลยสำรอง เทรนจาก 60 รูปที่เก็บด้วยคำสั่งข้างบน (mAP50 0.98) เครื่องมือเก็บภาพจะยกแขนไปท่า `look_pose` ของ `pick_and_place` ให้ก่อน เพราะนั่นคือมุมที่ detector จะเห็นตอนหยิบจริง (`--no-look` ถ้าไม่ต้องการ)

> ลูกบาศก์ที่ถูกลูกอื่นบังเกินครึ่ง detector จะให้ค่ามั่นใจต่ำ (วัดได้ 0.42 กับ `conf` 0.5) — ถ้าลูกบาศก์ถูกเฉี่ยวจนซ้อนกัน ลด `conf` ในหน้าต่างจูน YOLO ได้

### ให้คะแนน

```bash
python3 tools/score.py --seed 3        # seed เดียวกับที่ launch
```

อ่านตำแหน่งลูกบาศก์จริงจาก Gazebo: อยู่บนสถานีที่สีตรง = 10, สถานีอื่น = 3, ที่อื่น = 0 และเวลาจาก `/mission/summary`

### ตัวเลขที่วัดได้ (เครื่องพัฒนา, `detector:=color` ค่าเฉลย, seed 3)

| ช่วง | เวลา |
|---|---|
| สำรวจ (เดิน 3.8 ม. ผ่านตัว S + หันดู 3 มุม) | ~100 วิ |
| กลับมาแท่นหยิบ | 115–175 วิ |
| dock + หยิบ + ถอย | 55–80 วิ |
| ส่งของ (Nav2 ~5 ม. + approach + วาง) | 150–160 วิ |
| **ทั้งเกม 3 ลูก** | **~21 นาที** (วัด 1246 วิ ได้ 20/30 คะแนน; รอบ YOLO 1220 วิ 20/30) |

ระหว่างพัฒนาเจอสิ่งเหล่านี้ ซึ่งเป็นสาเหตุของค่าที่ตั้งไว้ข้างบน: DWB นั่งนิ่งในช่องแคบ, กล่อง skid เกี่ยวมุมผนังเมื่อ `robot_radius` 0.10, Nav2 ขับชนแท่นที่ LiDAR มองไม่เห็น, ป้าย 0.15 ม. อ่านไม่ได้จาก 2 ม., เสาบังป้ายจากจุดสำรวจ, แผ่นสีที่ไม่เรืองแสงมี S แค่ 32 ในเงา, แท่นตื้น 8 ซม. ทำลูกบาศก์ข้าง ๆ ตกเมื่อกริปเปอร์เฉี่ยว, และ Nav2 มาถึง `pick_table` คลาด 5–13 ซม. จนแขนเอื้อมไม่ถึง (จึงต้อง dock ไปจุดแน่นอนด้วย TF ก่อนหยิบ)

## 11. หน้าต่างเดโมของหุ่นจริง (ported UI windows)

ใน `src/example/` ของ Hiwonder มีเดโมที่เปิด**หน้าต่าง OpenCV** อยู่หลายตัว (ตรวจจับสี, AprilTag, AR, KCF, กล้อง depth, MediaPipe) แต่รันบน PC ไม่ได้ เพราะเรียก `controller` / `kinematics.so` / `arm_kinematics` ของ ARM, เรียก service ของบอร์ด STM32 และอ่านไฟล์ที่มีแต่ในอิมเมจของหุ่น (เช่น `/home/ubuntu/software/lab_tool/lab_config.yaml`)

รอบนี้ย้ายมาเขียนใหม่ใน `src/simulations/rospider_gazebo/scripts/` ให้ **หน้าตาและพฤติกรรมเหมือนของ Hiwonder** แต่ต่อกับ topic ของ sim โดยตรง ค่าคงที่ เกณฑ์ตัดสิน และท่าเริ่มต้น (pulse ของ servo 19–24) ยกมาจากต้นฉบับทั้งหมด ส่วนไหนที่ sim ไม่มี (ลำโพง, action group, IK ของ ARM) จะเขียนบอกไว้ใน docstring หัวไฟล์ทุกไฟล์

### รันยังไง

เปิด Gazebo ไว้ก่อน แล้วค่อยเปิดหน้าต่างเดโม (launch นี้**ไม่**เปิด Gazebo ให้ จะได้ขับหุ่นไปมา หรือเปิด `pick_place.launch.py` ค้างไว้ แล้วสลับเดโมได้)

```bash
# terminal 1
ros2 launch rospider_gazebo gazebo.launch.py

# terminal 2
ros2 launch rospider_gazebo vision_demo.launch.py demo:=color_position
```

กด `q` หรือ `Esc` ในหน้าต่างเพื่อปิด ทุกเดโมยัง publish ภาพเดียวกันที่ `/<ชื่อเดโม>/image_result` ด้วย (ดูผ่าน `rqt_image_view` หรือรันแบบไม่มีจอด้วย `show:=false` ก็ได้)

### มีเดโมอะไรบ้าง

| `demo:=` | ย้ายมาจาก | ทำอะไร | ต้องมีอะไรด้วย |
|---|---|---|---|
| `color_position` | `opencv_example/color_position.py` | บอกพิกัดจุดกึ่งกลางของก้อนสีบนภาพ | ตัวตรวจจับ (launch เปิด `color_detect` ให้เอง) |
| `color_recognition` | `opencv_example/color_recognition_node.py` | เห็นสีไหนค้างนาน 30 เฟรม แล้วขยับแขนท่าประจำสีนั้น | เหมือนบน |
| `apriltag_position` | `opencv_example/apriltag_position.py` | อ่าน id / x / y / ความกว้าง / ระยะ ของป้ายที่เห็น | `apriltag_detect` (launch เปิดให้เอง) + มีป้ายในโลก |
| `apriltag_track` | `opencv_example/apriltag_track.py` | เดินเข้าไปหาป้ายที่กำหนดแล้วหยุดที่ระยะ `stop_distance` | เหมือนบน |
| `ar_view` | `opencv_example/ar.py` | วาดลูกบาศก์ (หรือโมเดล `.obj`) ทับป้าย AprilTag | มีป้ายในโลก |
| `kcf_track` | `opencv_example/kcf.py` | กด `s` ลากกรอบเลือกเป้าหมาย แล้วหุ่นหันตาม | — |
| `prevent_falling` | `rgbd_example/prevent_falling_node.py` | เดินหน้า เจอขอบโต๊ะ/ขั้นบันไดแล้วเลี้ยวหนี | กล้อง depth |
| `cross_bridge` | `rgbd_example/cross_bridge_node.py` | คลานข้ามสะพานแคบ เบี่ยงกลับเข้ากลางเอง | ต้องสร้างสะพานในโลกเอง |
| `object_volume` | `rgbd_example/object_volume_measurement.py` | แยกทรงกลม/ทรงกระบอก/กล่อง จาก depth แล้วคำนวณปริมาตร | กล้อง depth + มีของวางอยู่ |
| `object_classification` | `rgbd_example/object_classification.py` | แยกรูปทรง + สี + ตำแหน่งในกรอบกล้อง | เหมือนบน |
| `hand_detect` | `mediapipe_example/hand_detect.py` | วาดโครงกระดูกมือ + บอกชื่อท่ามือ | `source:=webcam` |
| `hand_gesture` | `mediapipe_example/hand_gesture.py` | ทำท่ามือค้างไว้ แล้วหุ่นทำท่าประจำท่ามือนั้น | `source:=webcam` |
| `finger_trajectory` | `mediapipe_example/finger_trajectory.py` | วาดรูปกลางอากาศด้วยนิ้วชี้ แล้วให้ทายว่าเป็นรูปอะไร | `source:=webcam` |
| `face_track` | `mediapipe_example/face_track.py` | กล้องบนแขนหันตามหน้าคน | `source:=webcam` |
| `pose_control` | `mediapipe_example/pose_control.py` | ขาหน้าสองข้างขยับตามแขนคน | `source:=webcam` |

### เดโม MediaPipe ต้องใช้กล้องของ PC

ในโลก sim ไม่มีคน เดโมกลุ่ม MediaPipe จึงต้องดูกล้องจริง — ใส่ `source:=webcam` แล้ว launch จะเปิด `webcam_publisher.py` ส่งภาพจากกล้อง PC ขึ้น `/webcam/image_raw` ให้เอง (คนนั่งหน้าโน้ตบุ๊ก หุ่นขยับใน Gazebo)

```bash
# ลง mediapipe ก่อน (ไม่มีใน rosdep) — ไฟล์นี้ตรึง numpy<2 ไว้ด้วย เพราะ cv_bridge ของ Jazzy ต้องใช้ numpy 1.x
pip install --user --break-system-packages -r src/simulations/rospider_gazebo/requirements-mediapipe.txt
ros2 launch rospider_gazebo vision_demo.launch.py demo:=hand_gesture source:=webcam
ros2 launch rospider_gazebo vision_demo.launch.py demo:=hand_gesture source:=webcam device:=2   # เลือกกล้อง
```

ถ้าไม่ได้เปิด Gazebo เลย (ดูแค่หน้าต่างเฉยๆ) ให้ใส่ `use_sim_time:=false` ด้วย ไม่งั้น node จะรอ `/clock` ที่ไม่มีวันมา

### ปรับค่า

ค่าเริ่มต้นทั้งหมดอยู่ใน `config/vision_demos.yaml` (แยกเป็นบล็อกต่อเดโม) ค่าที่ใช้บ่อยสั่งทับจาก command line ได้เลย:

```bash
ros2 launch rospider_gazebo vision_demo.launch.py demo:=apriltag_track target_tag:=2 stop_distance:=0.5
ros2 launch rospider_gazebo vision_demo.launch.py demo:=color_position detector:=yolo
ros2 launch rospider_gazebo vision_demo.launch.py demo:=kcf_track tracker:=csrt
```

ค่าที่เป็น list (`colors`, `roi`, `shapes`) แก้ได้เฉพาะในไฟล์ yaml เพราะ argument ของ launch เป็นข้อความล้วน

### ตั้งระยะพื้นก่อนใช้เดโม depth

`prevent_falling`, `cross_bridge`, `object_volume`, `object_classification` ตัดสินจาก "พื้นอยู่ไกลเท่าไร" ซึ่งขึ้นกับท่าแขนตอนเปิดโลก วัดครั้งเดียวด้วย `debug:=true` — มันจะเฉลี่ย 50 เฟรมแล้ว log ค่าออกมา จากนั้นเอาค่านั้นใส่กลับ:

```bash
ros2 launch rospider_gazebo vision_demo.launch.py demo:=prevent_falling debug:=true
# [prevent_falling]: floor is 0.372 m away -- pass plane_distance:=0.372 next time to skip this
ros2 launch rospider_gazebo vision_demo.launch.py demo:=prevent_falling plane_distance:=0.372
```

> หน่วยไม่เหมือนกันระหว่างเดโม: `prevent_falling` / `cross_bridge` ใช้ **เมตร** ส่วน `object_volume` / `object_classification` ใช้ **มิลลิเมตร** — เป็นแบบนี้ในต้นฉบับของ Hiwonder อยู่แล้ว เลยไม่แก้

### หน้าต่างวาดเหมือนต้นฉบับทุกจุด

ภาพในหน้าต่างตอนทำงานปกติคือ draw call ชุดเดียวกับสคริปต์ต้นฉบับ: รูปทรง สี (แปลงเป็น BGR ให้ตรงกับที่ต้นฉบับแสดงหลัง `RGB2BGR`) ฟอนต์ ตำแหน่ง และ**ชื่อหน้าต่าง** (`image`, `result`, `depth_color_map`, `Object Classification (ROI Mode)`, ...) FPS ขึ้นเฉพาะเดโมที่ต้นฉบับเรียก `show_fps` (`face_track`) ส่วนสถานะที่มีเฉพาะใน sim — รอ topic, calibrate พื้น, เป้าหาย, ยังไม่ได้เลือกกรอบ — จะพิมพ์ใน terminal แบบที่ต้นฉบับ `print` ไม่วาดทับภาพ จึงเทียบกับภาพในคู่มือ Hiwonder ได้ตรง ๆ และไม่ต้องเรียนรู้ป้ายที่หุ่นจริงไม่มี

### เรื่องที่ต่างจากหุ่นจริง (ตั้งใจให้ต่าง)

- **คำสั่ง servo** — สคริปต์ยังเขียนด้วย pulse แบบ Hiwonder (`(19, 500), (20, 750), ...`) แล้ว `rospider_gazebo/servo_map.py` แปลงเป็นเรเดียนส่งเข้า `arm_controller` / `gripper_controller` / `leg_controller` ตารางแปลงลอกมาจาก `driver/servo_controller/config/servo_controller.yaml` ตรงๆ (servo หมุน 240° ต่อ 1000 pulse, `init` คือ 0 เรเดียน, ข้อที่ `min` > `max` คือหมุนกลับทาง)
- **ไม่มีเสียง** — เดโมที่หุ่นจริงร้องบี๊บ (`color_recognition`, `hand_gesture`, `pose_control`) เงียบไป ดูได้จาก log ใน terminal
- **ไม่มี action group** — `hand_gesture` ของจริงเล่นท่าที่อัดไว้ใน `/home/ubuntu/software/actionset_editor/ActionGroups` ใน sim จับคู่ท่ามือกับการเคลื่อนที่สั้นๆ ผ่าน `/controller/cmd_vel` แทน (ดูตาราง `MOVES` ในไฟล์)
- **`face_track` ไม่ใช้ IK** — ของจริงคำนวณความสูงข้อมือผ่าน service `arm_kinematics` (ไลบรารี ARM) ใน sim กล้องอยู่บน `link4` อยู่แล้ว เลยสั่ง servo 19 (หัน) กับ servo 22 (ก้ม/เงย) ตรงๆ ผลลัพธ์ที่ตาเห็นเหมือนกัน
- **`pose_control` ไปแย่ง `leg_controller` กับ `sim_gait`** — อย่าสั่งหุ่นเดินระหว่างที่กำลังเลียนแบบท่า ไม่งั้นสองตัวจะแย่งกันสั่งขาหน้า
- **`apriltag_track` กับ behaviours ของ `apriltag_detect` ขับหุ่นทั้งคู่** — เปิดทีละอย่าง
- **`kcf_track` อาจไม่ได้ใช้ KCF จริง** — OpenCV 5 (ที่ pip ลงให้) ไม่มี KCF และ CSRT แล้ว โค้ดจะไล่หา KCF → CSRT → MIL แล้ว log บอกว่าได้ตัวไหน
- **`ar_view` ไม่มีโมเดล `.obj` ติดมา** — ไฟล์ของ Hiwonder อยู่ใน `src/example/example/opencv_example/include/3d_model/` ซึ่ง package `example` ไม่ได้ install ให้ ถ้าจะใช้ต้องชี้ path เอง พร้อมใส่ scale ที่ต้นฉบับใช้ (bicycle 50, fox 4, chair 400, cow 0.4, wolf 0.6):

```bash
ros2 launch rospider_gazebo vision_demo.launch.py demo:=ar_view \
  model_path:=$PWD/src/example/example/opencv_example/include/3d_model/fox.obj model_scale:=4
```

### ที่ไม่ได้ย้ายมา

- `rgbd_example/track_and_grab.py` — เป็นเดโม "เห็นแล้วหยิบ" ซึ่ง sim มี `pick_place.launch.py` ทำหน้าที่นี้อยู่แล้ว (และทำได้ครบกว่า เพราะมี IK ปิดรูปของตัวเอง)
- ส่วนที่ `object_classification` ของจริงหยิบของไปจำแนกใส่ถาด — เหลือแค่ตรวจจับและรายงาน เหตุผลเดียวกัน
- `opencv_example/color_detect_node.py`, `apriltag_recognition.py` — ตัวตรวจจับ ไม่ใช่หน้าต่างเดโม; sim มี `color_detect.py` / `apriltag_detect.py` ของตัวเองที่ publish topic และ message เดียวกันอยู่แล้ว (ดูข้อ 7 และข้อ 8)

## รันหลายตัวพร้อมกัน

ตั้งค่าคนละชุดในแต่ละ terminal ไม่ให้ชนกัน:

```bash
export ROS_DOMAIN_ID=11 GZ_PARTITION=sim11
```

## ปัญหาที่พบบ่อย

- **Gazebo ปิดเองทันที / log มี `OpenGL 3.3 is not supported` หรือ `eglInitialize failed`** — ไดรเวอร์ GPU มีปัญหา (เช่น NVIDIA kernel module กับ library คนละเวอร์ชันหลังอัปเดต ให้ reboot) โหมด `gui:=false` ใช้ EGL headless ซึ่งต้องการไดรเวอร์ GPU ที่ใช้งานได้
- **หุ่นเคลื่อนช้ามาก** — ดู real time factor มุมขวาล่างของ Gazebo ถ้าต่ำกว่า ~0.5 เครื่องทำงานไม่ทัน (ปกติควรใกล้ 1.0)
- **RViz ขึ้น `Message Filter dropping message ... earlier than all the data in the transform cache` หนึ่งครั้งตอนเริ่ม และ `GLSL link result` / `active samplers ...`** — ไม่มีผล ข้อความแรกเกิดเพราะ scan แรกมาถึงก่อน TF (หุ่นจริงก็เป็น) ข้อความ GLSL เป็นคำเตือนของไดรเวอร์กราฟิก
- **`slam.launch.py` / `navigation.launch.py` ของ package `slam` / `navigation` (Hiwonder) ใช้กับ sim ไม่ได้** แม้มี `sim:=true` เพราะยังเปิด driver ของหุ่นจริง และเขียนไว้สำหรับ ROS 2 Humble — ใช้ของ `rospider_gazebo` แทน
- **node ที่ต้องใช้ `controller` / `kinematics` ของ Hiwonder** (เช่น self balancing, body control, perform actions) รันบน PC ไม่ได้ เพราะต้องใช้ `kinematics.so` ของ ARM
- **เดโม MediaPipe ขึ้น `No module named mediapipe`** — `rosdep` ไม่ได้ลงให้ ต้องลงเองจาก `requirements-mediapipe.txt` (ดูข้อ 11)
- ขาและแขนใน sim ไม่มี collision (ตัดออกเพื่อให้ physics เร็ว) — ขาทะลุสิ่งกีดขวางได้ ตัวหุ่นชนกำแพงผ่านกล่อง collision ใต้ลำตัว

