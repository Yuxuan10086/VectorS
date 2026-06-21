# 控制模块（`robot_control/`）

本目录是工作空间内 **控制** 大类，**本身不是 ROS 包**（根目录无 `package.xml`）。  
子包提供 **ROS 接口定义**、**整机硬件节点** 与 **Web 手动控制**；底层 SDK 在 **`robot_driver/`**。

## 子包与接口一览

| 子包 | 角色 | ROS 接口 |
|------|------|----------|
| **`robot_interfaces`** | 类型定义（无节点） | `.msg` / `.srv` / `.action` |
| **`robot_platform`** | 整机硬件节点 `platform_node` | 话题、服务、Action、TF |
| **`manual_control`** | Web 控制面板 | 作为客户端调用 platform 接口；HTTP/WebSocket |

---

## `robot_interfaces` — 接口定义包

**不运行节点**；其他包 `depend` 本包即可获得消息类型。  
实现方主要为 **`robot_platform`**，消费方如 **`manual_control`**。

### 消息 `MotorState`

用于 `/motor/get_state` 响应体（完整电机系统参数快照）。

### 服务

| 类型 | 典型服务名 | Request | Response |
|------|------------|---------|----------|
| `SetDriveMode` | `/chassis/set_mode` | `uint8 mode`（0=TWIST，1=MOVE） | `success`, `message` |
| `GetMotorState` | `/motor/get_state` | `motor_id` | `MotorState state` |
| `GetMotorSpeedLoopPid` | `/motor/get_speed_loop_pid` | `motor_id` | `success`, `p`, `i`, `d`, `message` |
| `SetMotorSpeedLoopPid` | `/motor/set_speed_loop_pid` | `motor_id`, `p`, `i`, `d` | `success`, `p`, `i`, `d`, `message` |
| `StartArmMotionRecording` | `/arm/start_motion_recording` | `action_name` | `success`, `message` |
| `FinishArmMotionRecording` | `/arm/finish_motion_recording` | （空） | `success`, `message` |

### Action

| 类型 | 典型名 | Goal | Feedback | Result |
|------|--------|------|----------|--------|
| `ChassisMove` | `/chassis/move` | `distance_m`, `speed_mps` | `distance_remaining_m` | `success` |
| `ChassisSpan` | `/chassis/span` | `angle_rad`, `omega_radps` | `angle_remaining_rad` | `success` |
| `ArmCalibrate` | `/arm/calibrate` | （空） | `completed_axis`, `progress`, `z_done`, `j1_done`, `j2_done`, `current_step` | `success`, `message` |
| `ArmPlayMotion` | `/arm/play_motion` | `file_path`（CSV 绝对路径） | `phase`, `progress`, `index`, `total`, `message` | `success`, `message` |

详见 [`robot_interfaces/README.md`](robot_interfaces/README.md)。

---

## `robot_platform` — 整机节点

**节点**：`platform_node`（`ros2 launch robot_platform platform.launch.py` 或 `robot_bringup` 入口）。

依赖 `motor` + `chassis` + `scara_arm`；参数来自 `chassis.yaml`、`scara_arm.yaml`、`platform_topics.yaml`。

### 订阅（默认话题名可参数覆盖）

| 话题 | 类型 | 行为 |
|------|------|------|
| `/chassis/cmd_vel` | `geometry_msgs/msg/Twist` | `linear.x`、`angular.z` → 底盘速度；TWIST 模式；~300 ms 无指令则停车 |
| `/arm/joint_command` | `sensor_msgs/msg/JointState` | `z`/`j1`/`j2` 逻辑行程 → `RobotArm::set_position`；**录制/播放中忽略** |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 注入航向；首次收到后对齐 map 零点 |

### 服务

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/chassis/set_mode` | `SetDriveMode` | TWIST / MOVE 切换 |
| `/motor/get_state` | `GetMotorState` | 读指定电机系统参数 |
| `/motor/get_speed_loop_pid` | `GetMotorSpeedLoopPid` | 读速度环 PID |
| `/motor/set_speed_loop_pid` | `SetMotorSpeedLoopPid` | 写速度环 PID |
| `/imu/reset_zero` | `std_srvs/Trigger` | 当前朝向设为 map 航向 0° |
| `/arm/start_motion_recording` | `StartArmMotionRecording` | 开始 J1/J2 示教录制 |
| `/arm/finish_motion_recording` | `FinishArmMotionRecording` | 结束录制并写 CSV |

### Action

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/chassis/move` | `ChassisMove` | 仅 MOVE 模式；阻塞直线 |
| `/chassis/span` | `ChassisSpan` | 仅 MOVE 模式；阻塞转向 |
| `/arm/calibrate` | `ArmCalibrate` | 三轴碰停标定 + 进度反馈 |
| `/arm/play_motion` | `ArmPlayMotion` | SDK 播放 CSV 轨迹 |

### TF

| 变换 | 频率 | 说明 |
|------|------|------|
| `map` → `base_link` | 7 Hz | 位置由 move/span 成功累加；航向由 IMU − offset |

更多细节见 [`robot_platform/README.md`](robot_platform/README.md)。

---

## `manual_control` — Web 控制面板

**节点**：`web_control_panel`（FastAPI + WebSocket + ROS 2）。

| 方向 | 接口 | 说明 |
|------|------|------|
| HTTP | `http://<host>:8080/` | 主控制页 |
| HTTP | `http://<host>:8080/monitor` | 电机监控 / PID |
| WebSocket | `/ws` | 摇杆、机械臂、模式、Action、动作录制/播放/删除 |
| WebSocket | `/ws/monitor` | 电机状态轮询、PID 读写 |

### 作为 ROS 客户端（默认名与 platform 一致）

| 方向 | 名称 | 类型 |
|------|------|------|
| 发布 | `/chassis/cmd_vel` | `Twist` |
| 发布 | `/arm/joint_command` | `JointState` |
| 订阅 | `/imu/data_raw` | `Imu`（车身朝向显示） |
| 服务客户端 | `/chassis/set_mode` | `SetDriveMode` |
| 服务客户端 | `/arm/start_motion_recording`、`/arm/finish_motion_recording` | 录制 |
| 服务客户端 | `/motor/get_state`、`/motor/get_speed_loop_pid`、`/motor/set_speed_loop_pid` | 监控页 |
| Action 客户端 | `/chassis/move`、`/chassis/span` | 阻塞移动 |
| Action 客户端 | `/arm/calibrate` | 标定 |
| Action 客户端 | `/arm/play_motion` | 动作播放（goal 为 CSV 路径） |

启动：`ros2 launch manual_control web_panel.launch.py`。详见 [`manual_control/README.md`](manual_control/README.md)。

---

## 编译与启动示例

```bash
cd ~/robot_ws
colcon build --packages-select motor chassis scara_arm robot_interfaces robot_platform manual_control
source install/setup.bash

# 整机（含 IMU 时推荐 bringup）
ros2 launch robot_bringup imu_platform_bringup.launch.py

# Web 面板（另一终端）
ros2 launch manual_control web_panel.launch.py
```
