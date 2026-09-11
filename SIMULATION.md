# ROSpider Simulation

วิธีรัน ROSpider แบบ simulation ล้วนๆ บน PC (ไม่ต่อหุ่นจริง) — ทดสอบบน Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic

## จำลองอะไรได้บ้าง

| ต้องการ | ใช้ | หมายเหตุ |
|---|---|---|
| ดูโมเดล/ขยับข้อต่อด้วย slider | `rospider_description display.launch.py` | RViz อย่างเดียว ไม่มี physics |
| วางแผนแขนกลแบบไม่ใช้ Gazebo | `robot_moveit_config demo.launch.py` | mock hardware |
| หุ่นในสภาพแวดล้อม + เซนเซอร์ | `rospider_gazebo gazebo.launch.py` | LiDAR, depth camera, IMU, odom |
| ทำแผนที่ (SLAM) | `rospider_gazebo slam.launch.py` | slam_toolbox + ค่า `slam/config/slam.yaml` |
| นำทาง (Nav2) | `rospider_gazebo navigation.launch.py` | ใช้แผนที่ห้อง sim ที่ทำไว้แล้ว |
| แขนกล MoveIt ใน Gazebo | `rospider_gazebo moveit.launch.py` | สั่งแขน/gripper ผ่าน MoveIt |

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
argument: `world:=<path.sdf>`, `x:=` `y:=` `yaw:=` (จุดเกิด), `gui:=false` (ไม่เปิดหน้าต่าง Gazebo — ดูหัวข้อปัญหาที่พบบ่อย)

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

## 4. SLAM (ทำแผนที่)

```bash
ros2 launch rospider_gazebo slam.launch.py
```

เปิด Gazebo + slam_toolbox + RViz (`slam/rviz/slam.rviz`) แล้วขับหุ่นด้วย teleop ให้ทั่วห้อง จากนั้นบันทึกแผนที่:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map --ros-args -p use_sim_time:=true
```

## 5. Navigation

```bash
ros2 launch rospider_gazebo navigation.launch.py
```

ใช้แผนที่ `maps/rospider_room.yaml` ที่ทำจากห้อง sim ไว้แล้ว และตั้งตำแหน่งเริ่มต้นให้อัตโนมัติ (หุ่นเกิดที่ origin ของแผนที่) — กด **2D Goal Pose** ใน RViz เพื่อสั่งเดิน หรือ:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.9, y: 0.3}, orientation: {w: 1.0}}}}"
```

ใช้แผนที่อื่น: `map:=/abs/path/my_map.yaml` (ถ้าหุ่นไม่ได้เกิดที่ origin ของแผนที่ ให้ตั้งตำแหน่งด้วย **2D Pose Estimate**)

ความเร็วและ footprint มาจาก config ของหุ่นจริง (`navigation/config`) — สูงสุด 0.05 ม./วิ. หุ่นจึงเดินช้า (เป้าหมายห่าง ~1 ม. ใช้เวลาราว 1 นาที) ต่างจากค่าของ Hiwonder 2 ค่าใน DWB คือ `xy_goal_tolerance` และ `sim_time` เพราะค่าเดิมทำให้หุ่นหยุดก่อนถึงเป้าหมายและ goal ไม่จบ (รายละเอียดที่หัวไฟล์ `src/simulations/rospider_gazebo/config/nav2_params.yaml`)

ลองค่า Nav2 อื่นโดยไม่ต้อง build ใหม่: `params_file:=/abs/path/my_nav2_params.yaml`

## 6. MoveIt ใน Gazebo

```bash
ros2 launch rospider_gazebo moveit.launch.py
```

RViz จะเปิดแท็บ MotionPlanning — เลือก planning group `arm` หรือ `gripper` แล้ว Plan & Execute แขนใน Gazebo จะขยับตาม และภาพกล้อง (ติดอยู่บนแขน) เปลี่ยนตาม

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
