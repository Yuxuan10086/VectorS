# robot_platform

整机硬件装配包：拥有一个 **`robot_driver::CanInterface`**，并实例化：

- **`chassis::DiffDriveChassis`**（参数见 **`chassis/config/chassis.yaml`**）
- **`scara_arm::RobotArm`**（参数见 **`scara_arm/config/scara_arm.yaml`**）

C++ 类 **`robot_platform::Platform`** 在 `open()` 时从 `rclcpp::Node` 读取上述参数，不在本包内重复定义默认值。

## 依赖

`motor`、`chassis`、`scara_arm`（均为独立 ament 包，与目录位置无关）。

## 编译与运行

完整启动（推荐）—— 同时启动 IMU 驱动 + 静态 TF（base_link → imu_link） + platform 节点：

```bash
cd ~/robot_ws
colcon build --packages-select motor chassis scara_arm robot_platform robot_bringup wit_ros2_imu
source install/setup.bash

# 首次使用或 IMU 设备重插拔后，先绑定串口设备（创建 /dev/imu_usb 软链接）
# （脚本位于 wit_ros2_imu/bind_usb.sh，需 root 权限）
sudo ~/robot_ws/src/robot_driver/wit_ros2_imu/bind_usb.sh

ros2 launch robot_bringup imu_platform_bringup.launch.py
```

**带参数启动示例**（更多参数说明见 `robot_bringup/README.md`）：

```bash
# 开启机械臂自动标定 + 自定义 IMU 安装高度
ros2 launch robot_bringup imu_platform_bringup.launch.py \
    arm_auto_calibrate:=true \
    imu_z:=0.15
```

单独只启动 platform（需手动启动 IMU 和 TF，可用于调试）：

```bash
ros2 launch robot_platform platform.launch.py
```

**前提条件**：
- IMU 硬件已正确插入，且 `bind_usb.sh` 已成功执行（或 udev 规则已生效）。
- `chassis.yaml` 中 `encoder_pulses_per_rev: 52100`（每圈脉冲，与 `pos()` 0x2A 单位一致，电机直连车轮）。

`platform_node` 在硬件 `open()` 后订阅下列话题（默认值见 **`config/platform_topics.yaml`**，可用参数覆盖）：

| 参数 | 默认话题 | 消息 | 行为 |
|------|----------|------|------|
| `chassis_cmd_vel_topic` | `/chassis/cmd_vel` | `geometry_msgs/msg/Twist` | `linear.x`、`angular.z` → `DiffDriveChassis::set_twist`；停止发布约 300ms 后底盘看门狗停车 |
| `arm_joint_command_topic` | `/arm/joint_command` | `sensor_msgs/msg/JointState` | 解析三轴 `position` → `RobotArm::set_position`（须先标定，`reachable_span_*` 有效） |
|| `imu_topic` | `/imu/data_raw` | `sensor_msgs/msg/Imu` | 把最新 Yaw 注入 Platform（内部同时更新底盘 SDK + 维护 IMU 零点 offset）；启动后首次 IMU 到达会自动 reset_zero 使初始位姿与 map 重合 |

**JointState 约定**：`name` 与 `arm_joint_names`（默认 `z` / `j1` / `j2`）逐项匹配取 `position`；若 `name` 为空且 `position` 至少 3 个，则按 `[z, j1, j2]` 顺序使用。

```bash
# 底盘
ros2 topic pub /chassis/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"

# 机械臂（标定后，单位与 scara_arm.yaml 中 span_* 一致）
ros2 topic pub /arm/joint_command sensor_msgs/msg/JointState \
  "{name: ['z','j1','j2'], position: [50.0, 90.0, 90.0]}"
```

## 底盘模式切换服务

`platform_node` 提供 ROS 服务用于在运行时切换底盘驱动模式：

- **服务名**：`/chassis/set_mode`
- **类型**：`robot_interfaces/srv/SetDriveMode`
- **常量**：
  - `TWIST=0`：速度控制模式（`set_twist` 有效，启用 300ms 看门狗保护）
  - `MOVE=1`：阻塞移动模式（`move` / `span` 有效，无看门狗）

```bash
# 切换到 MOVE 模式（阻塞移动）
ros2 service call /chassis/set_mode robot_interfaces/srv/SetDriveMode "{mode: 1}"

# 切换回 TWIST 模式（带看门狗的速度控制）
ros2 service call /chassis/set_mode robot_interfaces/srv/SetDriveMode "{mode: 0}"
```

服务返回：
- `success`：是否切换成功
- `message`：详细说明

**注意**：构造函数默认启动为 `kMove` 模式，`robot_platform` 启动后会立即切换到 `kTwist`（见 `platform.cpp`）。

**启动自动标定已移除**：`arm_auto_calibrate` 参数及启动时自动调用已删除。请改用新的 **`/arm/calibrate` Action**（带实时进度 Feedback，每个轴完成都会推送消息，前端有按钮和三轴状态显示）。

## 底盘阻塞移动 Action

当底盘处于 `kMove` 模式时，`platform_node` 提供两个 ROS Action 用于执行阻塞式移动命令（对应 `chassis::DiffDriveChassis` 的 `move` / `span`）：

- **ChassisMove**：`/chassis/move`
  - Goal：`distance_m`（米）、`speed_mps`（米/秒）
  - Feedback：`distance_remaining_m`（剩余命令距离，与 Goal 同单位）
  - Result：`success`（bool）

- **ChassisSpan**：`/chassis/span`
  - Goal：`angle_rad`（弧度）、`omega_radps`（弧度/秒）
  - Feedback：`angle_remaining_rad`（剩余命令角度，与 Goal 同单位）
  - Result：`success`（bool）

**重要**：这两个 Action 的 Goal Callback 会严格检查当前模式，只有处于 `kMove` 时才 ACCEPT，否则直接 REJECT。请先通过 `/chassis/set_mode` 服务切换到 MOVE 模式。

```bash
# 1. 先切到 MOVE 模式
ros2 service call /chassis/set_mode robot_interfaces/srv/SetDriveMode "{mode: 1}"

# 2. 发送直线移动 Action（示例：前进 1 米，速度 0.3 m/s）
ros2 action send_goal /chassis/move robot_interfaces/action/ChassisMove "{distance_m: 1.0, speed_mps: 0.3}"

# 3. 发送原地转向 Action（示例：顺时针转 90 度，角速度 0.5 rad/s）
ros2 action send_goal /chassis/span robot_interfaces/action/ChassisSpan "{angle_rad: -1.57, omega_radps: 0.5}"
```

**当前状态**：`move` / `span` 已实现真实 20ms 周期控制闭环（依赖 IMU yaw 注入）。

## IMU 零点重置服务与 map->base_link TF

`platform_node` 额外提供：

- **服务**：`/imu/reset_zero` （类型 `std_srvs/srv/Trigger`）

  调用后立即把 Platform 内部的 `imu_yaw_offset` 设为**当前原始 IMU 偏航角**，使得此后发布的 `map->base_link` 的 yaw = 0（当前物理朝向成为 map 坐标系的正方向）。

  位置（x,y）保持不变，仅重置航向参考。

  ```bash
  ros2 service call /imu/reset_zero std_srvs/srv/Trigger "{}"
  ```

- **TF 发布**：以 **7Hz** 持续广播 `map -> base_link` 变换

  - 初始：首次收到 IMU 数据后自动 `reset_imu_zero()`，保证 `map->base_link` 初始为单位位姿（x=0,y=0,yaw=0），与 map 重合。
  - 姿态：**始终** 由 `当前 IMU yaw - offset` 唯一确定（不受 move/span 历史影响，实时跟随 IMU）。
  - 位置：仅当 `/chassis/move` 或 `/chassis/span` Action **成功返回** 时更新：
    - move：按启动瞬间的 map yaw 方向，累加 `distance_m * (cos, sin)`
    - span：仅改变朝向（位置不变，yaw 由后续 IMU 实时刷新）
  - twist 模式下的自由遥控不会更新位置（无距离语义）。

  可用 RViz 或 `ros2 run tf2_ros tf2_echo map base_link` 查看实时位姿。

这样设计使得：用户可在任意时刻通过重置服务“告诉”系统“现在这个朝向就是 map 的 0 度”，之后所有 move/span 的位姿累加都在这个对齐后的 map 系下进行，同时 TF 始终输出最新 IMU 修正后的精确朝向。

## 在上层代码中使用

```cpp
#include "robot_platform/platform.hpp"

// node 已通过 launch 加载 chassis.yaml + scara_arm.yaml
robot_platform::Platform platform(*node);
if (!platform.open()) { /* ... */ }
// 可选：显式切换模式（kTwist 带看门狗，kMove 无看门狗）
platform.chassis().set_mode(chassis::DriveMode::kMove);
platform.arm().calibrate();
platform.close();
```
