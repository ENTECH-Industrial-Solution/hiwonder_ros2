# เตรียมเครื่องก่อนเข้าอบรม ROSpider

ไทย | [English](SETUP.en.md)

เอกสารนี้สำหรับผู้เข้าอบรม ทำให้เสร็จ**ก่อนวันอบรม**ตามลำดับหัวข้อ 1–5 เพราะการดาวน์โหลดและ build ใช้เวลานาน (ราว 30–60 นาที แล้วแต่อินเทอร์เน็ตและเครื่อง) ถ้าติดตรงไหนให้ดูหัวข้อ 7 หรือติดต่อวิทยากรก่อนวันอบรม

## 1. เลือกวิธีตามเครื่องของคุณ

ซิมในอบรมใช้ **Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic** เท่านั้น ถ้าเครื่องไม่ใช่ Ubuntu 24.04 ก็**ไม่ต้องลง OS ใหม่** ให้สร้าง Ubuntu 24.04 ขึ้นมาข้างในเครื่องเดิมแทน

| เครื่องของคุณ | วิธี | ไปที่ |
|---|---|---|
| Ubuntu 24.04 | ติดตั้งตรงบนเครื่อง | หัวข้อ 2A |
| Ubuntu 22.04, 20.04 หรือ Linux อื่น (มี ROS 2 Humble อยู่แล้วก็ได้) | container ด้วย distrobox | หัวข้อ 2B |
| Windows 10/11 | WSL2 | หัวข้อ 2C |
| macOS | ไม่รองรับ (Gazebo ใน VM ไม่มี GPU ซิมจะช้ามาก) | ติดต่อวิทยากร |

สเปกที่แนะนำ: RAM 8 GB ขึ้นไป (16 GB จะลื่นกว่า), พื้นที่ว่างอย่างน้อย 20 GB, การ์ดจอ Intel/AMD/NVIDIA ที่ลงไดรเวอร์แล้ว

## 2. เตรียม Ubuntu 24.04

### 2A. เครื่องเป็น Ubuntu 24.04 อยู่แล้ว

ไม่ต้องเตรียมอะไร ไปหัวข้อ 3 ได้เลย

### 2B. Ubuntu เวอร์ชันอื่น: ใช้ distrobox

distrobox สร้าง Ubuntu 24.04 เป็น container บนเครื่องเดิม หน้าต่าง Gazebo/RViz ขึ้นบนจอปกติและใช้การ์ดจอได้ ROS ที่มีอยู่บนเครื่อง (เช่น Humble) จะไม่ถูกแตะ

1. ติดตั้ง podman กับ distrobox (ครั้งเดียว):

   ```bash
   sudo apt install podman curl
   curl -s https://raw.githubusercontent.com/89luca89/distrobox/main/install | sudo sh
   ```

2. สร้าง container ชื่อ `jazzy` ที่มี home แยกเป็นของตัวเอง:

   ```bash
   distrobox create --name jazzy --image ubuntu:24.04 --home ~/jazzy-home
   ```

   ถ้าการ์ดจอเป็น NVIDIA ให้เติม `--nvidia` ต่อท้ายคำสั่ง

3. เข้า container (ครั้งแรกจะเตรียมตัวเองสักพัก):

   ```bash
   distrobox enter jazzy
   ```

4. ข้างใน container ให้คัดลอกไฟล์ตั้งค่า shell พื้นฐานมาไว้ใน home ใหม่ (ไม่อย่างนั้น terminal อาจไม่อ่าน `~/.bashrc`):

   ```bash
   cp -n /etc/skel/.bashrc /etc/skel/.profile ~/
   ```

**ทำไมต้องแยก home (`--home ~/jazzy-home`)**: ถ้าไม่แยก container จะใช้ `~/.bashrc` ไฟล์เดียวกับเครื่องจริง บรรทัดที่หัวข้อ 4 เพิ่มเข้าไปจะทำให้ terminal ปกติของเครื่องโหลด workspace ที่ build ด้วย Jazzy ทับ ROS ของเครื่อง แล้วงานอื่นที่ใช้ ROS อยู่จะพัง

จากนี้ไป**ทุกคำสั่งในหัวข้อ 3–5 ให้พิมพ์ใน container** (ใน terminal ที่สั่ง `distrobox enter jazzy` แล้ว)

### 2C. Windows: ใช้ WSL2

1. เปิด PowerShell แบบ **Run as administrator** แล้วสั่ง:

   ```powershell
   wsl --install -d Ubuntu-24.04
   ```

2. รีสตาร์ทเครื่องถ้าระบบขอ แล้วเปิดแอป **Ubuntu 24.04** จากเมนู Start ตั้งชื่อผู้ใช้และรหัสผ่านตามที่ถาม
3. ถ้าเคยลง WSL ไว้นานแล้ว ให้สั่ง `wsl --update` ใน PowerShell ก่อน หน้าต่าง Gazebo/RViz ต้องใช้ WSLg ที่มากับ WSL รุ่นใหม่

จากนี้ไปทุกคำสั่งในหัวข้อ 3–5 ให้พิมพ์ในหน้าต่าง Ubuntu 24.04

## 3. ติดตั้ง ROS 2 Jazzy

ย่อมาจาก[คู่มือติดตั้งของ ROS](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) ถ้าคำสั่งข้างล่างใช้ไม่ได้ ให้ทำตามคู่มือนั้นแทน

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

## 4. ดาวน์โหลดและ build workspace

```bash
sudo apt install -y git python3-rosdep python3-colcon-common-extensions
sudo rosdep init   # ถ้าขึ้นว่ามีไฟล์อยู่แล้วก็ไม่เป็นไร
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

`rosdep install` จะลง Gazebo, Nav2, slam_toolbox, RTAB-Map, MoveIt และอื่น ๆ ให้เอง ขั้นนี้นานที่สุด

ถ้า `colcon build` ค้างหรือเครื่องอืดจน RAM เต็ม ให้ build ทีละแพ็กเกจแทน: `colcon build --symlink-install --parallel-workers 1`

เสร็จแล้ว**ปิด terminal ทั้งหมดแล้วเปิดใหม่** (ถ้าใช้ distrobox ให้ `distrobox enter jazzy` ใหม่ด้วย)

## 5. ทดสอบ

1. เช็กว่า terminal โหลดค่าครบ:

   ```bash
   echo $ROS_DISTRO   # ต้องได้ jazzy
   echo $GZ_IP        # ต้องได้ 127.0.0.1
   ```

2. เปิดซิม (terminal ที่ 1):

   ```bash
   ros2 launch rospider_gazebo gazebo.launch.py
   ```

   หน้าต่าง Gazebo ต้องขึ้น มีหุ่นแมงมุมยืนอยู่ในห้อง

3. ขับหุ่น (terminal ที่ 2, ถ้าใช้ distrobox ต้อง `distrobox enter jazzy` ก่อน):

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/controller/cmd_vel
   ```

   กด `i` แล้วหุ่นต้องเดินไปข้างหน้า กด `k` เพื่อหยุด

4. ดูตัวเลข **real time factor** มุมขวาล่างของ Gazebo ระหว่างหุ่นเดิน ถ้าต่ำกว่า ~0.5 แปลว่าเครื่องช้าเกินไปสำหรับบางโจทย์ ให้แจ้งวิทยากรล่วงหน้า

5. (ไม่บังคับ) ทดสอบชุดที่หนักขึ้น ปิดของเดิมด้วย Ctrl+C ก่อน แล้วรัน `ros2 launch rospider_gazebo slam_challenge.launch.py` หน้าต่าง RViz ต้องขึ้นตามมาในไม่กี่วินาที

ผ่านครบทั้ง 5 ข้อแปลว่าเครื่องพร้อมแล้ว

## 6. วันอบรม: เปิด terminal ยังไง

- **Ubuntu 24.04 และ WSL2**: เปิด terminal แล้วใช้คำสั่งในเอกสารอบรมได้เลย
- **distrobox**: ทุก terminal ใหม่ต้องเข้า container ก่อน แล้วค่อยรันคำสั่ง

  ```bash
  distrobox enter jazzy
  cd ~/hiwonder_ros2/ROSpider
  ros2 launch rospider_gazebo slam_challenge.launch.py
  ```

  โจทย์ที่ใช้สอง terminal (เช่น launch ใน terminal หนึ่ง แล้ว `ros2 run rospider_gazebo check_slam.py` ในอีก terminal) ก็ต้อง `distrobox enter jazzy` ในทั้งสอง terminal ถ้าลืม จะเจอ `Package 'rospider_gazebo' not found` หรือ `ros2: command not found`

  ทางลัด: ใส่ `alias jz='distrobox enter jazzy'` ไว้ใน `~/.bashrc` **ของเครื่องจริง** แล้วพิมพ์แค่ `jz`

- **แก้ไฟล์โจทย์ด้วย editor บนเครื่อง** (เช่น VS Code): ไฟล์อยู่ที่

  | วิธี | path ของโฟลเดอร์ ROSpider เวลาเปิดจากเครื่อง |
  |---|---|
  | Ubuntu 24.04 | `~/hiwonder_ros2/ROSpider` |
  | distrobox | `~/jazzy-home/hiwonder_ros2/ROSpider` (ใน container คือ `~/hiwonder_ros2/ROSpider` ไฟล์เดียวกัน) |
  | WSL2 | ใน File Explorer พิมพ์ `\\wsl$\Ubuntu-24.04\home\<ชื่อผู้ใช้>\hiwonder_ros2\ROSpider` หรือสั่ง `code .` ในหน้าต่าง Ubuntu |

  แก้ไฟล์ `.yaml` แล้วรันใหม่ได้เลย ไม่ต้อง build ซ้ำ

- **อย่ารัน ROS ของเครื่องจริงไปพร้อมกับซิม** (กรณี distrobox) เพราะ container ใช้เครือข่ายเดียวกับเครื่อง

## 7. ปัญหาที่พบบ่อยตอนติดตั้ง

- **`Package 'rospider_gazebo' not found`**: terminal นี้ยังไม่ได้โหลด workspace ถ้าใช้ distrobox ให้เช็กว่าอยู่ใน container (`echo $ROS_DISTRO` ต้องได้ `jazzy`) ถ้าอยู่แล้ว ให้เช็กว่า `colcon build` ผ่านโดยไม่มี `Failed` และ `~/.bashrc` มีสองบรรทัด `source ~/hiwonder_ros2/ROSpider/...`
- **distrobox: `echo $ROS_DISTRO` ว่าง ทั้งที่อยู่ใน container แล้ว**: terminal ไม่ได้อ่าน `~/.bashrc` ทำขั้นที่ 4 ของหัวข้อ 2B (`cp -n /etc/skel/.bashrc /etc/skel/.profile ~/`) แล้วสั่ง `exit` และ `distrobox enter jazzy` ใหม่ (`cp -n` ไม่เขียนทับไฟล์ที่มีอยู่ บรรทัดของ workspace ใน `~/.bashrc` จึงไม่หาย)
- **`rosdep install` ขึ้นว่าหา key บางตัวไม่เจอ**: ถ้าเป็นแพ็กเกจของหุ่นจริง (เช่นเสียง, ไดรเวอร์กล้อง/LiDAR) ไม่เป็นไร ซิมไม่ได้ใช้ ขอแค่ `colcon build` ผ่าน
- **Gazebo ปิดเองทันที หรือขึ้น `eglInitialize failed`**: ไดรเวอร์การ์ดจอมีปัญหา ลอง reboot, กรณี distrobox กับการ์ด NVIDIA ให้เช็กว่าสร้าง container ด้วย `--nvidia`
- **WSL2: หน้าต่าง Gazebo/RViz ไม่ขึ้น**: สั่ง `wsl --update` ใน PowerShell แล้ว `wsl --shutdown` แล้วเปิด Ubuntu ใหม่
- ปัญหาตอนใช้งานซิม ดูหัวข้อ "ปัญหาที่พบบ่อย" ใน [README](README.md)

## 8. เช็กลิสต์ก่อนวันอบรม

- [ ] `echo $ROS_DISTRO` ได้ `jazzy`
- [ ] `gazebo.launch.py` เปิดได้ และหุ่นเดินตาม teleop
- [ ] real time factor ไม่ต่ำกว่า ~0.5
- [ ] รู้ว่าจะเปิดไฟล์ในโฟลเดอร์ `ROSpider` ด้วย editor ยังไง
- [ ] (distrobox) จำได้ว่าทุก terminal ต้อง `distrobox enter jazzy` ก่อน
- [ ] ก่อนวันอบรมสั่ง `cd ~/hiwonder_ros2 && git pull` แล้ว `cd ROSpider && colcon build --symlink-install` อีกรอบ เผื่อมีโจทย์อัปเดต

ถ้าไม่ใช้แล้ว วิธีถอนการติดตั้งอยู่ในหัวข้อ "ถอนการติดตั้ง" ของ [README](README.md) กรณี distrobox ลบทั้งหมดได้ด้วย `distrobox rm jazzy` แล้ว `rm -rf ~/jazzy-home`
