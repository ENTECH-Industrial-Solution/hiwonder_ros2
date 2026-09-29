# ROSpider Simulation

## 1. ภาพรวม

**ROSpider** คือหุ่นแมงมุม 6 ขาของ Hiwonder มีแขนกลกับกล้อง depth ติดอยู่ที่ปลายแขน และมี LiDAR

เอกสารนี้อธิบาย **simulation บน PC** (package `rospider_gazebo`) ที่ Entech เพิ่มเข้ามา ใช้สอนในเวิร์กช็อปโดยไม่ต้องมีหุ่นจริง

- ใช้ได้บน Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic
- ชื่อ topic ตรงกับหุ่นจริง (`/controller/cmd_vel`, `/scan`, `/odom`, `/depth_cam/...`)
- ท่าเดินใน sim เป็นแบบจำลอง: ขาก้าวให้ดู แต่ตัวหุ่นถูกเลื่อนไปตาม `cmd_vel` โดยตรง เพราะโค้ดเดินของ Hiwonder (`kinematics.so`) เป็น binary ของ ARM ที่รันบน PC ไม่ได้

### ติดตั้งและ build (ครั้งแรก)

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

ทุก terminal ที่จะใช้ ต้องรันสามบรรทัดนี้ก่อน:

```bash
source /opt/ros/jazzy/setup.bash
source ~/entech_hiwonder_ros2_ws/ROSpider/install/local_setup.bash
export need_compile=True
```

ขับหุ่นด้วยคีย์บอร์ด ใช้ได้กับทุกหัวข้อ:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/controller/cmd_vel
```

## 2. โครงสร้างไฟล์

```
ROSpider/                  colcon workspace (build จากในโฟลเดอร์นี้)
├── SIMULATION.md          ← ไฟล์นี้
├── README.md              README ต้นฉบับของ Hiwonder (หุ่นจริง)
├── maps/                  แผนที่ที่ทำเอง (.yaml/.pgm = 2D, .db = 3D)
└── src/
    ├── driver/ app/ example/ slam/ navigation/ ...   โค้ดของ Hiwonder (หุ่นจริง)
    └── simulations/
        ├── rospider_description/   URDF / โมเดลหุ่น
        ├── robot_moveit_config/    ค่า MoveIt ของแขน
        └── rospider_gazebo/        ← simulation ทั้งหมดอยู่ที่นี่
            ├── launch/             ไฟล์สำหรับ ros2 launch (หนึ่งไฟล์ต่อหนึ่งหัวข้อ)
            ├── scripts/            node ต่าง ๆ (ท่าเดิน, ตรวจจับสี/AprilTag, หยิบของ, เดโม)
            ├── rospider_gazebo/    โค้ด Python ที่ใช้ร่วมกัน (IK, PID, ...)
            ├── config/             ค่าปรับตั้ง (.yaml) เช่นช่วงสี, Nav2, RTAB-Map
            ├── worlds/ models/     ฉากใน Gazebo และวัตถุ (ลูกบาศก์, แท่น, ป้าย AprilTag)
            ├── maps/               แผนที่ห้อง sim ที่ทำไว้ให้แล้ว
            ├── urdf/ rviz/         URDF ส่วนของ sim และหน้าจอ RViz
            ├── test/               unit test
            └── tools/              สคริปต์ช่วย (เก็บ dataset/เทรน YOLO, สร้างพื้นผิว)
```

## 3. หัวข้อในเวิร์กช็อป

| หัวข้อ | คำสั่งหลัก |
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

วางแผนการเคลื่อนที่ของแขนกลด้วย MoveIt ใน RViz เลือก planning group `arm` หรือ `gripper` ลาก marker ไปยังท่าที่ต้องการ แล้วกด **Plan & Execute**

```bash
ros2 launch robot_moveit_config demo.launch.py      # แขนจำลอง ไม่มี Gazebo
ros2 launch rospider_gazebo moveit.launch.py        # แขนใน Gazebo (กล้องบนแขนขยับตาม)
```

### Gazebo Simulation

เปิดหุ่นในห้องจำลองพร้อมเซนเซอร์ครบ (LiDAR, กล้อง depth, IMU, odometry) แล้วขับด้วย teleop

```bash
ros2 launch rospider_gazebo gazebo.launch.py
```

ตัวเลือกที่ใช้บ่อย:
- `arm_pose:=horizontal` ให้กล้องมองตรงไปข้างหน้า
- `world:=<ชื่อ>` เปลี่ยนห้อง ใส่แค่ชื่อไฟล์ใน `worlds/` ได้ เช่น `world:=slam_challenge` (ใช้ได้กับทุก launch ที่เปิด Gazebo)
- `gui:=false` ไม่เปิดหน้าต่าง Gazebo

ดูภาพกล้องได้ด้วย `ros2 run rqt_image_view rqt_image_view`

### SLAM Mapping

ทำแผนที่ 2D ด้วย LiDAR (slam_toolbox) มีขั้นตอนดังนี้:

1. เปิด SLAM:
   ```bash
   ros2 launch rospider_gazebo slam.launch.py
   ```
2. ขับหุ่นด้วย teleop ให้ทั่วห้อง
3. เซฟแผนที่โดยรันจากโฟลเดอร์ `ROSpider`:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f maps/room1 --ros-args -p use_sim_time:=true
   ```

ถ้าจะใช้แผนที่นี้นำทาง ให้รัน `ros2 launch rospider_gazebo navigation.launch.py map:=room1` แล้วกด **2D Goal Pose** ใน RViz (`map:=` หาใน `ROSpider/maps/` ก่อน แล้วค่อยหาใน `maps/` ของโฟลเดอร์ที่รันคำสั่ง)

**โจทย์: ซ่อมค่าให้ผ่านด่าน** (10–15 นาที)

ห้องโจทย์มีสองห้องเชื่อมกันด้วยประตู ค่า SLAM ที่ให้มาตั้งผิดไว้ 3 จาก 6 ค่า หาให้เจอแล้วแก้จนผ่านครบ 3 ด่าน

```bash
ros2 launch rospider_gazebo slam_challenge.launch.py
```

1. แก้ค่าในไฟล์โจทย์ `src/simulations/rospider_gazebo/config/slam_challenge.yaml` (launch พิมพ์ path ให้ทุกครั้ง) แล้วปิด-เปิด launch ใหม่ — ต้อง build ด้วย `--symlink-install` ไม่งั้นแก้แล้วต้อง `colcon build` ใหม่
2. ขับให้ทั่วทั้งสองห้อง แล้วเซฟแผนที่: `ros2 run nav2_map_server map_saver_cli -f maps/<ชื่อ> --ros-args -p use_sim_time:=true`
3. ตรวจ: `ros2 run rospider_gazebo check_slam.py <ชื่อ>` บอกผลทีละด่านพร้อมคำใบ้

ด่าน: 1 มีแผนที่ · 2 สำรวจครอบคลุม ≥ 90% · 3 แผนที่ละเอียด (ช่องประตูเปิด ผนังไม่หนาเกินจริง) — อยากเริ่มใหม่: `git checkout -- src/simulations/rospider_gazebo/config/slam_challenge.yaml`

นำทางในห้องโจทย์ด้วยแผนที่ที่เซฟ: `ros2 launch rospider_gazebo navigation.launch.py world:=slam_challenge map:=<ชื่อ>`

**โจทย์ Nav2: ให้หุ่นวิ่งครบเส้นทาง** (10–15 นาที)

ใช้ห้องโจทย์เดิมกับแผนที่เฉลย ค่า Nav2 ตั้งผิดไว้ 3 จาก 6 ค่า แก้จนหุ่นวิ่งครบ 3 จุด (มุมห้อง A → ผ่านประตู ทางเดิน อ้อมทรงกระบอก ไปมุมซ้ายบนห้อง B → กลับจุดเริ่ม) ภายใน 300 วินาที

```bash
ros2 launch rospider_gazebo nav_challenge.launch.py      # map:=<ชื่อ> ใช้แผนที่ของตัวเอง
ros2 run rospider_gazebo check_nav.py                    # ตัวตรวจสั่งวิ่งเองแล้วบอกผลทีละด่าน
```

แก้ค่าใน `src/simulations/rospider_gazebo/config/nav_challenge.yaml` แล้วปิด-เปิด launch ใหม่ก่อนตรวจทุกครั้ง — ด่าน: 1 ถึงเป้าหมายแรก · 2 ผ่านประตูไปห้อง B · 3 วิ่งครบทันเวลา (ค่าผิดบางค่าจะโผล่ให้เห็นหลังแก้ค่าอื่นแล้ว ต้องแก้ครบทั้ง 3 ค่าถึงจะผ่านครบ)

### RTAB-VSLAM 3D Mapping

ทำแผนที่ 3D ด้วย RTAB-Map

```bash
ros2 launch rospider_gazebo vslam.launch.py map:=room_vslam           # ใช้กล้องอย่างเดียว
ros2 launch rospider_gazebo rtabmap_slam.launch.py map:=room1         # ใช้กล้อง + LiDAR
```

1. ขับหุ่นด้วย teleop ให้ทั่วห้อง
2. กด **Ctrl+C** แล้วรอจนขึ้น `Saving database/long-term memory...done!` แผนที่จะถูกเซฟเป็น `.db` ลงใน `ROSpider/maps/` (ของ V-SLAM อยู่ใน `maps/vslam/`)
3. โหลดแผนที่กลับมานำทาง:
   ```bash
   ros2 launch rospider_gazebo vslam.launch.py localization:=true map:=room_vslam
   ```

ข้อควรระวัง: การรันโหมดทำแผนที่ด้วยชื่อเดิมจะเขียนทับไฟล์เดิม (โหมดนำทางใช้สำเนาของแผนที่ ไฟล์เดิมไม่ถูกแก้)

**โจทย์ V-SLAM: สร้างแผนที่ด้วยกล้อง** (15–20 นาที)

ใช้ห้องโจทย์เดิม (ผนังแต่ละด้านมีลายไม่ซ้ำกัน กล้องจะได้จำที่ได้) ค่าในไฟล์โจทย์ตั้งผิดไว้ 3 จาก 5 ค่า

```bash
ros2 launch rospider_gazebo vslam_challenge.launch.py map:=myvslam
```

1. แก้ค่าใน `src/simulations/rospider_gazebo/config/vslam_challenge.yaml` แล้วปิด-เปิด launch ใหม่
2. ขับด้วย teleop ให้ทั่วทั้งสองห้อง แล้วกลับมาที่จุดเริ่มและหันไปทางเดิม (หุ่นต้องกลับมาเห็นภาพเดิมถึงจะเกิด loop closure)
3. กด **Ctrl+C** รอจนปิดเสร็จ แผนที่ถูกบันทึกตอนนี้
4. ตรวจ: `ros2 run rospider_gazebo check_vslam.py myvslam`

ด่าน: 1 มีแผนที่และเห็นผนัง · 2 สำรวจครอบคลุม ≥ 90% · 3 แผนที่ไม่เบี้ยว (จำที่เดิมได้ ไม่จำผิดที่ ผนังไม่ซ้อน)

**โจทย์นำทางด้วย V-SLAM** (10 นาที) ใช้แผนที่ของตัวเองจากโจทย์ข้างบน ค่าตั้งผิดไว้ 2 จาก 6 ค่า เส้นทางและตัวตรวจเดียวกับโจทย์ Nav2

```bash
ros2 launch rospider_gazebo vslam_nav_challenge.launch.py map:=myvslam
ros2 run rospider_gazebo check_nav.py
```

แก้ค่าใน `src/simulations/rospider_gazebo/config/vslam_nav_challenge.yaml` หุ่นต้องเริ่มที่จุดเดียวกับตอนเริ่มทำแผนที่ (จุดเกิด)

วิทยากร: แผนที่เฉลยสร้างด้วย `vslam_challenge.launch.py map:=vslam_answer params:=<path ของ config/vslam_challenge_solved.yaml>` แล้วรัน `python3 src/simulations/rospider_gazebo/tools/drive_route.py` (ขับเส้นทางเดิมให้เองประมาณ 5 นาที) ไฟล์เฉลยของโจทย์นำทางคือ `config/vslam_nav_challenge_solved.yaml`

### Color Threshold Adjustment

จูนช่วงสีในปริภูมิ LAB ด้วยหน้าต่าง LAB_Tool แบบเดียวกับที่ใช้บนหุ่นจริง

```bash
ros2 launch rospider_gazebo depth_camera.launch.py   # terminal 1: เปิดกล้อง (Gazebo)
ros2 launch rospider_gazebo lab_tool.launch.py       # terminal 2: เปิดหน้าต่างจูน
```

1. เลือกสีใน `Color list`
2. เลื่อน slider `L` `A` `B` จนช่อง mask เห็นแค่วัตถุ
3. กด **Save** ค่าจะถูกเก็บที่ `~/.ros/color_detect_tuned.json` และ terminal จะพิมพ์บล็อก YAML ไว้ให้วางลงใน `config/color_detect.yaml`

**โจทย์: จูนสีให้เจอแค่กล่องที่ต้องการ** (10 นาที)

ฉากมีกล่องแดง เขียว น้ำเงิน กล่องน้ำเงินอีกกล่องอยู่ในเงา และกล่องสีส้มที่คล้ายสีแดง ช่วงสีที่ให้มาตั้งผิดไว้ทั้ง 3 สี

```bash
ros2 launch rospider_gazebo color_challenge.launch.py   # เปิดฉาก + หน้าต่าง LAB_Tool
ros2 run rospider_gazebo check_color.py                 # ตรวจจากภาพกล้อง บอกผลทีละด่าน
```

จูนใน LAB_Tool แล้วรันตัวตรวจได้เลยไม่ต้องปิด launch (ตัวตรวจใช้ค่าที่หน้าต่างแสดงอยู่ตอนนั้น) อย่าลืมกด **Save** (เก็บที่ `~/.ros/color_challenge_tuned.json` ทับค่าในไฟล์โจทย์ `config/color_challenge.yaml`) ไม่งั้นปิด launch แล้วค่าหาย ห้ามขับหุ่นระหว่างทำโจทย์ ตัวตรวจจะไม่ยอมตรวจถ้ากล้องไม่ได้เห็นฉากตามตำแหน่งเริ่มต้น — ด่าน: 1 เจอครบทุกสี · 2 ไม่จับของหลอก · 3 เจอแม้อยู่ในเงา — อยากเริ่มใหม่: ลบ `~/.ros/color_challenge_tuned.json` แล้วปิด-เปิด launch ใหม่

### Color Tracking

คลิกที่วัตถุในภาพ แล้วหุ่นจะเดินตามก้อนสีนั้น

```bash
ros2 launch rospider_gazebo gazebo.launch.py            # terminal 1
ros2 launch rospider_gazebo object_tracking.launch.py   # terminal 2
```

คลิกซ้ายที่วัตถุในหน้าต่าง `image` หุ่นจะเก็บสีใต้เมาส์ แล้วเดินตามให้วัตถุอยู่ที่จุดสีเหลือง ถ้าจับสีได้กว้างหรือแคบเกินไป ให้ปรับด้วย `threshold:=0.3`

**โจทย์: เดินไปหยุดหน้าลูกบอลสีแดง** (10 นาที)

ฉากมีลูกบอลสีแดง (ต้องไปหา) กับลูกบอลสีส้ม (ห้ามไปหา) หุ่นต้องหยุดตรงหน้าลูกบอลแดงที่ระยะ 0.43–0.52 ม. ภายใน 40 วินาที ค่าตั้งผิดไว้ 3 จาก 4 ค่า

```bash
ros2 launch rospider_gazebo track_challenge.launch.py   # ฉาก + หน้าต่าง image (หุ่นยังไม่เดิน)
ros2 run rospider_gazebo check_track.py                 # ตัวตรวจคลิกเลือกสีลูกบอลแดงและสั่งเดินให้เอง
```

แก้ค่าใน `src/simulations/rospider_gazebo/config/track_challenge.yaml` แล้วปิด-เปิด launch ใหม่ก่อนตรวจทุกครั้ง (คลิกในหน้าต่างเพื่อดูว่าสีจับอะไรได้ แต่หุ่นจะไม่เดินจนกว่าตัวตรวจสั่ง) — ด่าน: 1 เดินไปหาลูกบอลสีแดง · 2 หยุดหน้าลูกบอลพอดี · 3 ทันเวลา

### AprilTag Tag Tracking

หุ่นเดินเข้าหาป้าย AprilTag ที่เลือก แล้วหยุดที่ระยะที่ตั้งไว้ ในห้อง sim มีป้ายหมายเลข 1 ติดไว้ให้แล้ว

```bash
ros2 launch rospider_gazebo gazebo.launch.py                                  # terminal 1
ros2 launch rospider_gazebo apriltag_track.launch.py target_tag:=1 stop_distance:=0.35   # terminal 2
```

**โจทย์: เดินไปหยุดหน้าป้ายหมายเลข 2** (10 นาที)

ฉากมีป้าย 3 ป้าย หุ่นต้องเดินไปหยุดหน้าป้ายหมายเลข 2 ในระยะที่แขนหยิบของบนแท่นได้ (กล้องห่างป้ายประมาณ 0.30–0.40 ม.) ภายใน 40 วินาที ค่าตั้งผิดไว้ 3 จาก 4 ค่า

```bash
ros2 launch rospider_gazebo apriltag_challenge.launch.py   # ฉาก + หน้าต่าง image (หุ่นยังไม่เดิน)
ros2 run rospider_gazebo check_tag.py                      # ตัวตรวจสั่งเดินเองแล้วบอกผลทีละด่าน
```

แก้ค่าใน `src/simulations/rospider_gazebo/config/apriltag_challenge.yaml` แล้วปิด-เปิด launch ใหม่ก่อนตรวจทุกครั้ง — ด่าน: 1 ไปหาป้ายหมายเลข 2 · 2 หยุดระยะพอดี · 3 ทันเวลา

### Autonomous Line Following

หุ่นเดินตามเส้นบนพื้น launch นี้เปิด Gazebo ให้เอง ในฉากที่มีเส้นดำเป็นลูป และหุ่นเกิดบนเส้นพอดี

```bash
ros2 launch rospider_gazebo line_following.launch.py
```

คลิกซ้ายที่เส้นในหน้าต่าง `image` แล้วหุ่นจะเริ่มเดิน ถ้ามีสิ่งกีดขวางอยู่ใกล้กว่า 0.4 ม. หุ่นจะหยุดรอ วิ่งครบหนึ่งรอบใช้เวลาประมาณ 2 นาที

**โจทย์: ให้หุ่นเกาะเส้นครบรอบทันเวลา** (10 นาที)

ค่าของตัวเกาะเส้นตั้งผิดไว้ 2 จาก 4 ค่า แก้จนหุ่นวิ่งครบรอบภายใน 200 วินาที

```bash
ros2 launch rospider_gazebo line_challenge.launch.py   # ฉากเส้น + หน้าต่าง image
ros2 run rospider_gazebo check_line.py                 # ตัวตรวจเลือกสีเส้นและสั่งวิ่งให้เอง
```

แก้ค่าใน `src/simulations/rospider_gazebo/config/line_challenge.yaml` แล้วปิด-เปิด launch ใหม่ก่อนตรวจทุกครั้ง (หุ่นต้องเริ่มที่จุดเกิดบนเส้น) — ด่าน: 1 เกาะเส้นได้ 1/4 รอบ · 2 วิ่งครบรอบ · 3 ครบรอบทันเวลา

### 3D Vision: Object Grasping

หุ่นติดตามก้อนสีด้วยกล้องบนแขน แล้วหยิบขึ้นมา launch นี้เปิดทุกอย่างให้เอง (Gazebo, แท่นวาง, ลูกบาศก์ และ node หยิบ)

```bash
ros2 launch rospider_gazebo track_and_grab.launch.py
```

ในหน้าต่างควบคุม กดปุ่มสี (red / green / blue) เพื่อสั่งหยิบ ตัวเลือกในหน้าต่าง:
- **pick here**: หยิบก้อนที่อยู่ตรงหน้า (ก้อนเขียว)
- **walk to it**: เดินไปหาก้อนสีนั้นก่อนแล้วค่อยหยิบ (ก้อนแดงและน้ำเงินวางอยู่ด้านข้าง)
- **place: auto**: วางลงข้างตัวทันทีที่หยิบได้
- **place: by button**: ถือไว้จนกว่าจะกดปุ่ม **Place**

**โจทย์: หยิบบล็อกสีน้ำเงินไปวางบนแผ่นสีเหลือง** (15 นาที)

บล็อกสีน้ำเงินอยู่บนแท่นทางขวาของหุ่น ไกลเกินแขนเอื้อม หุ่นต้องเดินไปหยิบ แล้ววางลงบนแผ่นสีเหลืองที่พื้น ค่าตั้งผิดไว้ทั้ง 3 ค่า

```bash
ros2 launch rospider_gazebo grasp_challenge.launch.py   # ฉาก + หน้าต่าง track_and_grab
ros2 run rospider_gazebo check_grasp.py                 # ตัวตรวจสั่งหยิบบล็อกสีน้ำเงินให้เอง (ราว 1 นาที)
```

แก้ค่าใน `src/simulations/rospider_gazebo/config/grasp_challenge.yaml` แล้วปิด-เปิด launch ใหม่ก่อนตรวจทุกครั้ง (หุ่นและบล็อกต้องอยู่ที่เดิม) — ด่าน: 1 เดินไปถึงบล็อก · 2 หยิบขึ้นมา · 3 วางบนแผ่นสีเหลือง

### 3D Vision: Shape Recognition

ใช้ภาพ depth แยกรูปทรงของวัตถุ (ทรงกลม ทรงกระบอก กล่อง) และบอกสี ใน sim ทำได้แค่ตรวจจับและรายงานผล ยังไม่หยิบไปแยกใส่ถาด

```bash
ros2 launch rospider_gazebo pick_place.launch.py auto_start:=false    # terminal 1: ฉากที่มีของวาง
ros2 launch rospider_gazebo object_classification.launch.py           # terminal 2
```

ถ้าตรวจไม่เจอ ให้วัดระยะพื้นหนึ่งครั้งด้วย `debug:=true` แล้วนำค่าที่ log บอกไปใส่ใน `plane_distance:=...` (หน่วยเป็นมิลลิเมตร)

**โจทย์: ให้หุ่นหาลูกบอลให้เจอ** (10 นาที)

บนพื้นใต้กล้องมีลูกบอล (ทรงกลม) กล่อง และทรงกระบอก หุ่นต้องแยกวัตถุออกจากพื้น เห็นครบทั้ง 3 ชิ้นพร้อมเรียกชื่อรูปทรงถูก แล้วเลือกลูกบอล (กรอบสีแดงในหน้าต่าง `depth`) ค่าตั้งผิดไว้ทั้ง 3 ค่า

```bash
ros2 launch rospider_gazebo shape_challenge.launch.py   # ฉาก + หน้าต่าง depth
ros2 run rospider_gazebo check_shape.py                 # ตรวจจากผลที่หุ่นรายงาน บอกผลทีละด่าน
```

แก้ค่าใน `src/simulations/rospider_gazebo/config/shape_challenge.yaml` แล้วปิด-เปิด launch ใหม่ก่อนตรวจทุกครั้ง (หุ่นไม่ต้องเดิน) — ด่าน: 1 แยกวัตถุออกจากพื้น · 2 เห็นครบและเรียกชื่อถูก · 3 เลือกลูกบอล

## ปัญหาที่พบบ่อย

- **launch ขึ้น `KeyError: 'need_compile'`**: ลืม `export need_compile=True`
- **Gazebo ปิดเองทันที หรือขึ้น `eglInitialize failed`**: ไดรเวอร์ GPU มีปัญหา ลอง reboot
- **หุ่นเคลื่อนช้ามาก**: ดู real time factor มุมขวาล่างของ Gazebo ถ้าต่ำกว่า ~0.5 แปลว่าเครื่องทำงานไม่ทัน
- **node หากันไม่เจอหลังรันหลายรอบ**: ปิดทุกอย่าง แล้วรัน `rm -f /dev/shm/fastrtps_*`
- **รันหลายคนในเครือข่ายเดียวกัน**: ให้แต่ละเครื่องตั้ง `export ROS_DOMAIN_ID=<เลขไม่ซ้ำกัน>`
