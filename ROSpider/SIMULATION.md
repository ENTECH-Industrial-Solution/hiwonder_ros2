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

หุ่นยืนอยู่กับที่ ก้มแขนมองหาลูกบาศก์สีบนแท่นด้วยกล้อง RGB-D ที่ติดอยู่บนแขน แล้วหยิบไปวางเรียงกันทีละลูกที่อีกจุดหนึ่งหน้าหุ่น การตรวจจับสีใช้ OpenCV แยกสีในปริภูมิ HSV (`scripts/color_detect.py`, ค่าปรับที่ `config/color_detect.yaml`) แล้วส่งผลออกทาง `/yolo/object_detect` (`interfaces/ObjectsInfo`) — หัวข้อและชนิดข้อความเดียวกับ `yolo_node.py` ของหุ่นจริง จึงเปลี่ยนไปใช้ YOLO จริงภายหลังได้โดยไม่ต้องแก้โค้ดส่วนหยิบ-วาง (`scripts/pick_and_place.py`)

ไม่ใช้ MoveIt ในเส้นทางหยิบ-วาง: คุมด้วย IK ปิดรูปเอง (`arm_ik.py`) แทน เพราะ `robot_moveit_config` ตั้ง `position_only_ik: true` (คุมได้แค่ตำแหน่งปลายมือ ไม่คุมทิศทาง) แต่การก้มลงหยิบต้องคุมมุมมือด้วย ส่วนการ "จับ" ลูกบาศก์ใช้ `DetachableJoint` เชื่อมลูกบาศก์เข้ากับ `link5` (มือจับ) แทนแรงเสียดทานจริง

รันทีละสี:

```bash
ros2 launch rospider_gazebo pick_place.launch.py auto_start:=false
ros2 service call /pick_and_place/start interfaces/srv/SetString "{data: 'red'}"
ros2 service call /pick_and_place/stop std_srvs/srv/Trigger
```

### วางเรียงแถว ไม่ซ้อนกัน

ช่องวางทั้งสามอยู่ที่ (0.160, +0.07, 0.135), (0.160, 0.00, 0.135), (0.160, -0.07, 0.135) ใน `base_footprint` (`config/pick_place.yaml`, `drop_slots`) เติมตามลำดับที่หยิบสำเร็จ ไม่ใช่ตำแหน่งคงที่ต่อสี ที่วางเรียงแทนที่จะซ้อนสามชั้นเพราะแขนสั้นเกินไป: ที่ความสูงซ้อนชั้นที่สาม มีแค่มุมก้ม 50-60 องศาที่ยังแก้ IK ได้ และมุมป้านขนาดนั้นต้องกวาดลูกบาศก์ที่ถืออยู่ผ่านลูกบาศก์ที่วางไว้แล้วในแนวราบ ผลวัดจริง: ลูกบาศก์แต่ละลูกห่างจากช่องของตัวเองไม่เกิน ~0.9 ซม.

### กล้องเห็นลูกบาศก์สีเดียวกันสองลูก

`/yolo/object_detect` รายงานวัตถุ 6 ชิ้น ไม่ใช่ 3 ชิ้น: `worlds/rospider_room.sdf` มีลูกบาศก์ตกแต่ง (visual-only ไม่มี collision) สีแดง/เขียว/น้ำเงินอยู่ที่ x=0.55 อยู่ก่อนแล้ว เป็นสีและขนาด (5 ซม.) เดียวกับลูกบาศก์ที่หยิบได้จริงที่ x=0.235 ทุกประการ `pick_and_place.py` เลือกกล่องที่ใหญ่ที่สุดต่อสีก่อน แล้วยืนยันด้วย IK ว่าจุดนั้นแขนเอื้อมถึงจริง — พฤติกรรมนี้เกิดบนหุ่นจริงด้วยเช่นกัน ใครอ่าน `/yolo/object_detect` ต่อจากนี้ต้องรู้ไว้

### ต้อง detach ก่อนสั่งแขน

Gazebo เชื่อมลูกบาศก์ทั้งสามเข้ากับมือจับ (`link5`) ทันทีตอนสแปวน์ (ก่อนโค้ดฝั่งเราสั่งอะไรเลย) `pick_and_place.py` จึงสั่ง detach ทั้งสามสีซ้ำด้วยตัวจับเวลาตอนเริ่มโหนด (ราว 3 วิ) และค้างสถานะ `IDLE` ไม่สั่งแขนจนกว่าขั้นนี้จะจบ — ข้ามขั้นนี้ไม่ได้ ไม่งั้นคำสั่งแขนแรกจะลากทั้งฉาก (แท่น+ลูกบาศก์) หลุดจากจุดตั้ง ใครเขียนโหนดอื่นที่ขยับแขนตัวนี้ต้องเจอเรื่องเดียวกัน

### ปรับค่า

- ช่วงสี HSV: `config/color_detect.yaml` ดูผลได้จากภาพ `/color_detect/image_result` ใน RViz
- ท่ามอง (`look_pose`), มุมเข้าหยิบ, ตำแหน่งช่องวาง (`drop_slots`): `config/pick_place.yaml` — `grasp_z_offset` ไม่ใช่แค่ครึ่งความสูงลูกบาศก์ เพราะจากมุมมองก้มชัน กล่องที่ตรวจจับได้คลุมทั้งหน้าบนและหน้าหน้า (foreshortened) จุดศูนย์กลางกล่องจึงตกที่ขอบบนใกล้กล้อง ไม่ใช่กึ่งกลางหน้าบน ต้องชดเชยด้วยค่านี้
- ตำแหน่งแท่นและลูกบาศก์: ตัวแปร `SCENE` ใน `launch/pick_place.launch.py`
  ถ้าย้ายตำแหน่ง ต้องแก้ `LAYOUT` ใน `test/test_arm_ik.py` ให้ตรงกัน แล้วรัน
  `colcon test --packages-select rospider_gazebo` เพื่อยืนยันว่ายังอยู่ในระยะที่แขนเอื้อมถึง

### ก่อนรัน

ถ้ารันซ้ำหลายรอบ เช็กก่อนว่าไม่มี `sim_gait.py`, `robot_state_publisher` หรือ `gz sim` ค้างจากรอบก่อน (เคยทำให้ `arm_controller` มา activate ช้าจนหยิบไม่ทันแล้ว timeout ทุกสี) และถ้าเพิ่งรันคำสั่ง ROS CLI สั้น ๆ ไปหลายครั้งติดกัน ให้ลอง `rm -f /dev/shm/fastrtps_*` ถ้า node ใหม่หา node อื่นไม่เจอ (shared-memory ค้างจาก process ที่ตายไปแล้ว)

### ตรวจสอบว่าทำงานถูกต้อง

1. แท่นไม่ทับขาหุ่นและไม่ซ้อน `sim_skid_link` ลูกบาศก์วางนิ่งบนแท่น
2. `/color_detect/image_result` วาดกรอบครบทุกสี (สองกรอบต่อสี — ลูกใกล้ที่หยิบได้กับลูกตกแต่งที่ไกลออกไป) จากท่ามอง (`joint4 = -1.55`)
3. ตำแหน่งที่ล็อกไว้ (ดูใน log ของ `pick_and_place`) ห่างจากตำแหน่ง spawn จริงไม่เกิน ~1 ซม.
4. `/grasp/<สี>/attach` ทำให้ลูกบาศก์ติดมือ และ `detach` ทำให้ตก
5. ลูกบาศก์ทั้งสามลูกไปอยู่ในช่องวางของตัวเอง (`drop_slots`) ห่างจากจุดกึ่งกลางช่องไม่เกิน 3 ซม. (วัดจริงซ้ำสองรอบได้ ~0.9 ซม.)

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
- ขาและแขนใน sim ไม่มี collision (ตัดออกเพื่อให้ physics เร็ว) — ขาทะลุสิ่งกีดขวางได้ ตัวหุ่นชนกำแพงผ่านกล่อง collision ใต้ลำตัว
