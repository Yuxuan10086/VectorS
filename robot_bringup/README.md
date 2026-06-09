# robot_bringup

整机启动包（bringup），提供一键启动真实机器人所需的所有核心节点。

目前包含两个便捷入口（均基于核心的 `robot_platform/launch/platform.launch.py`）：

- **`imu_platform_bringup.launch.py`**：IMU + Platform 核心启动（无额外 UI，推荐用于真实机器人运行）
- **`manual_platform_bringup.launch.py`**：IMU + Platform + **最新的网页控制器后台**（FastAPI WebSocket 版，适合手动调试/操作）

## 编译

```bash
cd ~/robot_ws
colcon build --packages-select robot_bringup robot_platform wit_ros2_imu chassis scara_arm motor
source install/setup.bash
```

## 前置准备（重要！）

1. **IMU 设备绑定**（每次插拔或重启后建议执行一次）：

   ```bash
   sudo ~/robot_ws/src/robot_driver/wit_ros2_imu/bind_usb.sh
   ```

   执行成功后应能看到 `/dev/imu_usb` 软链接指向实际串口设备。

2. 确保 CAN 总线已正确启动且底盘电机可正常通信（`robot_platform` 启动时会尝试打开 CAN）。

## 启动命令

### 核心启动（推荐，无 UI）

```bash
ros2 launch robot_bringup imu_platform_bringup.launch.py
```

或直接使用 platform 包的 launch（效果完全相同）：

```bash
ros2 launch robot_platform platform.launch.py
```

以上两者都会启动：
- WIT IMU 驱动（默认 `/dev/imu_usb`，9600 波特率，发布 `/imu/data_raw`）
- `base_link` → `imu_link` 静态 TF（默认 z=0.12m，无旋转）
- `robot_platform` 节点（底盘 + 机械臂）

### 带手动网页控制的启动（推荐用于调试）

```bash
ros2 launch robot_bringup manual_platform_bringup.launch.py
```

此启动会额外启动**最新的网页控制器后台**（`web_control_panel`）。
启动后浏览器访问 `http://<机器人或开发机IP>:8080` 即可使用现代网页控制面板。

### 带参数执行（常用覆盖示例）

```bash
# 1. 开启机械臂自动标定
ros2 launch robot_bringup imu_platform_bringup.launch.py \
    arm_auto_calibrate:=true

# 2. 指定不同的 IMU 串口和波特率
ros2 launch robot_bringup imu_platform_bringup.launch.py \
    imu_port:=/dev/ttyUSB1 \
    imu_baud:=115200

# 3. 自定义 IMU 安装位姿（平移 + 旋转）
# 格式：x y z yaw pitch roll（米 + 弧度），相对于 base_link
ros2 launch robot_bringup imu_platform_bringup.launch.py \
    imu_x:=0.05 \
    imu_y:=0.0 \
    imu_z:=0.18 \
    imu_yaw:=1.57 \
    imu_pitch:=0.0 \
    imu_roll:=0.0

# 4. 组合多个参数
ros2 launch robot_bringup imu_platform_bringup.launch.py \
    arm_auto_calibrate:=true \
    imu_port:=/dev/imu_usb \
    imu_z:=0.15 \
    imu_yaw:=3.1416
```

**参数说明**（均可通过命令行 `xxx:=value` 覆盖）：

| 参数名                | 默认值         | 说明 |
|-----------------------|----------------|------|
| `imu_port`            | `/dev/imu_usb` | IMU 串口设备 |
| `imu_baud`            | `9600`         | IMU 波特率 |
| ~~arm_auto_calibrate~~ | 已移除         | 启动自动标定已取消，改用 `/arm/calibrate` Action（带实时进度） |
| `imu_x/y/z`           | `0/0/0.12`     | IMU 相对 base_link 的安装位置（米） |
| `imu_yaw/pitch/roll`  | `0/0/0`        | IMU 相对 base_link 的安装旋转（弧度） |

## 自定义 IMU 安装位置

最灵活的方式是直接在 launch 文件中修改 `static_transform_publisher` 的默认值，或者通过上面所示的 launch 参数从命令行覆盖。

修改 launch 文件后需要重新 `colcon build` 该包。

## 常见问题

- **无法打开 /dev/imu_usb**：请先执行 `bind_usb.sh`，并检查 `ls -l /dev/imu_usb` 是否存在。
- **IMU 数据不更新或 Yaw 漂移**：确认 IMU 已正确安装且 TF 方向正确；可用 `ros2 topic echo /imu/data_raw` 快速验证。
- **想单独启动 IMU 或 platform**：可参考 `wit_ros2_imu` 或 `robot_platform` 包内的独立 launch 文件。

## 后续扩展

本包预留用于添加更多 bringup launch，例如：
- 带激光雷达/相机/导航的完整感知启动
- 带手动控制面板的调试启动
- 带任务/行为树的完整应用启动

欢迎贡献！
