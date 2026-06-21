# 驱动模块（`robot_driver/`）

本目录是工作空间内 **驱动** 大类，**本身不是 ROS 包**（根目录无 `package.xml`）。  
各子包提供硬件 SDK 或传感器 ROS 节点；整机运动控制由 **`robot_control/robot_platform`** 链接 `motor`、`chassis`、`scara_arm` 并对外发布 ROS 接口。

## 子包与接口一览

| 子包 | 类型 | 对外接口 | 说明 |
|------|------|----------|------|
| **`motor`** | C++ 库 | **无 ROS** | PD42 步进闭环：`CanInterface`、`Pd42Motor`、`pd42_protocol` |
| **`chassis`** | C++ 库 | **无 ROS** | 前驱差速底盘：`DiffDriveChassis` |
| **`scara_arm`** | C++ 库 + 示例节点 | **无 ROS**（库）；示例 `scara_arm_joints_repl` | SCARA 三轴：`ArmJoint`、`RobotArm`、动作录制/播放 |
| **`wit_ros2_imu`** | ROS 2 Python 节点 | 见下表 | WIT 串口 IMU |
| **`oradar_lidar`** | ROS 2 C++ 节点 | 见下表 | Oradar MS200 2D 激光雷达 |

---

### `motor` — PD42 / SocketCAN

**不提供 ROS 话题/服务**；供 `chassis`、`scara_arm`、`robot_platform` 链接。

| 接口 | 说明 |
|------|------|
| `CanInterface` | SocketCAN 收发、发送队列、≥2 ms 帧间隔 |
| `Pd42Motor` | 单轴：`initialize`、`set_mode`、`set_speed`（0xF1）、`set_absolute_position`（0xF2）、`set_torque`（0xF0）、`rpm`/`pos`/`arrived_flag`（0x30）等 |
| `pd42_protocol.hpp` | 组帧/解析（含速度环 PID 0x26/0x73 等） |

调试：`ros2 run motor pd42_motor_repl`。详见 [`motor/README.md`](motor/README.md)。

---

### `chassis` — 差速底盘 SDK

**不提供 ROS 话题/服务**；由 `robot_platform` 封装。

| C++ API | 说明 |
|---------|------|
| `DiffDriveChassis::set_twist(v, ω)` | 速度模式；内部 300 ms 看门狗 |
| `DiffDriveChassis::move(distance_m, speed_mps)` | 阻塞直线（MOVE 模式） |
| `DiffDriveChassis::span(angle_rad, omega_radps)` | 阻塞原地转（MOVE 模式） |
| `set_drive_mode(kTwist / kMove)` | 模式切换 |
| `inject_imu_yaw(rad)` | 由 platform 注入 IMU 航向 |

参数：`config/chassis.yaml`（轮距、减速比、电机 ID、`encoder_pulses_per_rev` 等）。

---

### `scara_arm` — SCARA 机械臂 SDK

**库无 ROS**；录制/播放逻辑在 SDK 内完成。

| C++ API | 说明 |
|---------|------|
| `RobotArm::calibrate()` | Z → J2 → J1 碰停标定 |
| `RobotArm::set_position(z, j1, j2)` | 三轴通信位置（须已标定） |
| `RobotArm::start_motion_recording(name)` | J1/J2 零力矩拖动，10 Hz 采样 |
| `RobotArm::finish_motion_recording()` | 写 CSV（7 列）到 `recordings/` |
| `RobotArm::play_motion_file(path, params, …)` | 读 CSV、预处理、首点到位门控、段定时下发 |
| `RobotArm::is_motion_recording()` / `is_motion_playing()` | 状态查询 |

参数：`config/scara_arm.yaml`。ROS 侧录制/播放见 **`robot_platform`**。详见 [`scara_arm/README.md`](scara_arm/README.md)。

示例：`ros2 run scara_arm scara_arm_joints_repl`（仅 J1/J2 调试，非整臂）。

---

### `wit_ros2_imu` — IMU 驱动

| 方向 | 名称 | 类型 | 说明 |
|------|------|------|------|
| 发布 | `/imu/data_raw` | `sensor_msgs/msg/Imu` | 原始 IMU；`frame_id` 多为 `imu_link` |

| 可执行文件 | 说明 |
|------------|------|
| `wit_ros2_imu` | 串口节点；默认端口 `/dev/imu_usb`（需 `bind_usb.sh`） |

Launch：`ros2 launch wit_ros2_imu rviz_and_imu.launch.py`。  
整机中由 `robot_bringup` / `robot_platform` 订阅该话题并发布 `map→base_link` TF。

---

### `oradar_lidar` — MS200 激光雷达

| 方向 | 名称（默认） | 类型 | 说明 |
|------|----------------|------|------|
| 发布 | `/scan` | `sensor_msgs/msg/LaserScan` | 2D 扫描；`frame_id` 默认 `laser_frame` |
| 发布 | `/point_cloud` | `sensor_msgs/msg/PointCloud2` | 点云节点可选 |
| TF | `base_link` → `laser_frame` | 静态 TF | `ms200_scan.launch.py` 内发布（高度可改） |

| 可执行文件 | 说明 |
|------------|------|
| `oradar_scan` | LaserScan |
| `oradar_pointcloud` | PointCloud2 |

Launch：`ros2 launch oradar_lidar ms200_scan.launch.py`。详见 [`oradar_lidar/README.md`](oradar_lidar/README.md)。

---

## 编译示例

```bash
cd ~/robot_ws
colcon build --packages-select motor chassis scara_arm wit_ros2_imu oradar_lidar
source install/setup.bash
```

整机节点还需：`robot_interfaces`、`robot_platform`（见 **`robot_control/README.md`**）。
