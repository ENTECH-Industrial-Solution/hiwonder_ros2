# Entech × Hiwonder ROS 2

ไทย | [English](README.en.md)

รวมโค้ด ROS 2 ของหุ่นยนต์ Hiwonder ที่ Entech ใช้สอนและพัฒนาต่อ โดยแยก **หนึ่งโฟลเดอร์ต่อหนึ่งรุ่นหุ่น** และแต่ละโฟลเดอร์เป็น colcon workspace ของตัวเอง จะมีรุ่นอื่นเพิ่มเข้ามาเรื่อย ๆ

<p align="center">
  <img src="ROSpider/sources/01.png" alt="ROSpider" width="480"/>
</p>

## หุ่นยนต์ในรีโปนี้

| รุ่น | หุ่น | หุ่นจริง | Simulation (PC) | เอกสาร |
|---|---|---|---|---|
| [ROSpider](ROSpider/) | หุ่นแมงมุม 6 ขา (18 servo) + แขนกล, กล้อง depth, LiDAR | Jetson, Ubuntu 22.04, ROS 2 Humble | Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic | [README](ROSpider/README.md) (sim + โจทย์เวิร์กช็อป), [REAL_ROBOT](ROSpider/REAL_ROBOT.md) (หุ่นจริง), [Hiwonder docs](https://docs.hiwonder.com/projects/ROSpider/en/jetson-orin-nano-version/) |

## โครงสร้าง

```
entech_hiwonder_ros2_ws/
├── README.md          ← ไฟล์นี้
├── README.en.md       ไฟล์นี้ฉบับภาษาอังกฤษ
├── CLAUDE.md          บันทึกเทคนิคสำหรับนักพัฒนา (ภาษาอังกฤษ)
├── docs/              spec และแผนงานของฟีเจอร์ที่ทำเพิ่ม
└── ROSpider/          colcon workspace ของ ROSpider
```

## กติกาสำคัญ

- **build จากในโฟลเดอร์ของรุ่นนั้นเท่านั้น** ห้ามรัน `colcon build` ที่โฟลเดอร์นี้ เพราะหุ่นแต่ละรุ่นของ Hiwonder ใช้ชื่อ package ซ้ำกัน (`app`, `bringup`, `controller`, ...) และ colcon ไม่ยอมให้ชื่อซ้ำ
- **อย่า source `install/` ของสองรุ่นใน terminal เดียวกัน**
- โค้ดของแต่ละรุ่นดึงมาจากรีโปของ Hiwonder ด้วย `git subtree` (เก็บประวัติเดิมไว้) ส่วนที่ Entech เพิ่มอยู่ในรีโปนี้

```bash
cd ~/entech_hiwonder_ros2_ws/ROSpider
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## อัปเดตโค้ดจาก Hiwonder

```bash
git subtree pull --prefix=ROSpider hiwonder Jetson_Nano_ROS2
```

remote `hiwonder` คือ https://github.com/Hiwonder/ROSpider.git

## เพิ่มหุ่นรุ่นใหม่

1. `git remote add <ชื่อ> <URL รีโป Hiwonder>`
2. `git subtree add --prefix=<ชื่อรุ่น> <ชื่อ> <branch>`
3. เพิ่มแถวในตาราง "หุ่นยนต์ในรีโปนี้" ด้านบน (ทั้งสองภาษา)
