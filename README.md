# nav2-parking-robot

ระบบนำทางและจอดรถอัตโนมัติสำหรับรถต้นแบบ Ackermann steering (ROS 2 + Nav2 + ESP32)

ผู้จัดทำ: ภูริภัทร ฤทธิ์สาร  อาจารย์ที่ปรึกษา: อาจารย์วทัญญู ชัยยะ

## เกี่ยวกับระบบ

รถต้นแบบใช้กลไกบังคับเลี้ยวแบบ Ackermann (เลี้ยวด้วยล้อหน้า เหมือนรถยนต์ทั่วไป) จึงหมุนอยู่กับที่ไม่ได้ ต้องวางเส้นทางแบบมีรัศมีเลี้ยว ระบบจึงใช้ Nav2 ร่วมกับ SLAM แผนที่ และสคริปต์จอดรถ (parking maneuver) ของตัวเอง

## เทคโนโลยีที่ใช้

- ROS 2 (Jazzy)
- Nav2 (Navigation stack)
- SLAM Toolbox
- robot_localization (EKF)
- twist_mux / velocity_smoother
- ESP32 (micro-ROS) ควบคุมเซอร์โวเลี้ยวและมอเตอร์
- YDLIDAR

## โครงสร้างโฟลเดอร์

โค้ดทั้งหมดอยู่ใน `src.zip` (แตกแล้วอยู่ที่ `src/tes/`):

| โฟลเดอร์ | เนื้อหา |
|---|---|
| `launch/` | ไฟล์ launch ของระบบ |
| `config/` | พารามิเตอร์ Nav2, SLAM, EKF, twist_mux ฯลฯ |
| `scripts/` | สคริปต์ Python (odom bridge, parking maneuver, spot detector ฯลฯ) |
| `urdf/`, `meshes/` | โมเดลหุ่นยนต์ |
| `maps/` | แผนที่ลานจอดรถ |
| `worlds/`, `rviz/`, `behavior_trees/` | world จำลอง, RViz, behavior tree |

## แพ็กเกจที่ต้องติดตั้งเพิ่ม

แพ็กเกจเหล่านี้เป็นของผู้อื่น ไม่ได้รวมอยู่ใน repo นี้ ให้ clone ลงใน `src` ของ workspace:

- [micro-ROS-Agent](https://github.com/micro-ROS/micro-ROS-Agent)
- [micro_ros_msgs](https://github.com/micro-ROS/micro_ros_msgs)
- [ydlidar_ros2_driver](https://github.com/YDLIDAR/ydlidar_ros2_driver)

## วิธีติดตั้งและรัน

ติดตั้งและ build:

```bash
git clone https://github.com/msxicollab/nav2-parking-robot.git
unzip nav2-parking-robot/src.zip -d ~/ros2_ws
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

คำสั่งรัน:

```bash
# 1. ระบบหลัก
ros2 launch tes car1_core.launch.py

# 2. ขับด้วยจอยสติ๊ก
ros2 launch tes car1_manual_control.launch.py

# 3. ตรวจหาช่องจอด (parking spot detector)
ros2 launch tes car1_parking_detector.launch.py

# 4. จอดอัตโนมัติ
ros2 launch tes car1_parking_auto.launch.py
```

## ระบบตรวจหาช่องจอด (Parking Spot Detector)

ใช้ `scripts/parking_spot_detector.py` ผ่าน `launch/car1_parking_detector.launch.py` เพื่อตรวจหาช่องจอดและส่งตำแหน่งให้ระบบจอดอัตโนมัติใช้ต่อ

## หมายเหตุ

โค้ดนี้เผยแพร่เพื่อการศึกษา สงวนลิขสิทธิ์ ดูรายละเอียดในไฟล์ LICENSE
