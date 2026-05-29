# manual_control

`robot_control` 子包，提供**两种手动控制方式**（可并存）：

- **网页控制面板**（推荐）：基于 FastAPI + WebSocket 的现代化浏览器 UI，支持虚拟摇杆、滑块、模式切换、阻塞移动 Action 进度反馈和急停。
- **键盘节点**（保留）：原有终端键盘控制（方向键 + WASD）。

两者均发布与 `robot_platform` 兼容的话题和服务。

## 推荐：网页控制面板

### 启动命令

```bash
cd ~/robot_ws

# 1. 编译（**必须同时构建 robot_platform**，因为网页面板依赖其自定义 .srv/.action 接口）
colcon build --packages-select robot_platform manual_control

# 2. 环境
source install/setup.bash

# 3. 启动 web 面板（默认 0.0.0.0:8080）
ros2 launch manual_control web_panel.launch.py
```

> **重要**：如果之前只单独构建过 `manual_control`，请务必执行上面的完整命令重新构建 `robot_platform` + `manual_control`。否则启动时会因为找不到 `robot_platform` 的 typesupport 库而报错。

浏览器打开：

- 本机：http://localhost:8080
- 局域网其他设备（手机/平板/另一电脑）：http://<机器人IP>:8080

**推荐配合启动顺序**（推荐新终端分别运行）：

```bash
# 终端1：整机硬件（必须先启动）
ros2 launch robot_platform platform.launch.py

# 终端2（可选）：避障
ros2 launch scan_safety simple_scan_safety.launch.py

# 终端3：网页控制面板
ros2 launch manual_control web_panel.launch.py web_port:=8081
```

### Launch 参数（常用）

```bash
ros2 launch manual_control web_panel.launch.py \
  web_port:=8081 \
  web_host:=0.0.0.0 \
  chassis_cmd_vel_topic:=/chassis/cmd_vel \
  arm_joint_command_topic:=/arm/joint_command \
  linear_speed_x:=0.20 \
  angular_speed_z:=0.8
```

### 面板功能

- **底盘**：虚拟摇杆（拖动）+ WASD 风格方向按钮 + 线/角速度上限滑块。按住发送，松手自动归零（服务端 watchdog 保护）。
- **机械臂**：三个实时滑块 + ±1 / ±5 步进按钮，支持直接数值设置。
- **模式切换**：一键切换 `TWIST`（速度 + 看门狗） / `MOVE`（阻塞移动 Action）。
- **阻塞移动**（仅 MOVE 模式）：
  - `ChassisMove`：发送距离 + 速度，实时剩余距离进度条 + 反馈。
  - `ChassisSpan`：发送角度 + 角速度，实时剩余角度进度条。
- **大红色 E-STOP**：一键停止底盘 + 取消所有进行中的 Action。
- 实时状态、日志区、键盘快捷键（WASD + 空格急停）。

> 提示：使用前请确保 `robot_platform` 已经成功启动并完成初始化。

## 原键盘节点（保留）

仍可使用原有键盘控制（适合无显示器场景）：

```bash
ros2 run manual_control manual_control_node
# 或
ros2 launch manual_control manual_control.launch.py
```

> 注意：必须在**真实终端**（TTY）中运行，stdin 需为交互式。

## 话题与接口（两者共用）

- 发布 `/chassis/cmd_vel` (`geometry_msgs/msg/Twist`)
- 发布 `/arm/joint_command` (`sensor_msgs/msg/JointState`)
- 调用 `/chassis/set_mode` (`robot_interfaces/srv/SetDriveMode`)
- Action `/chassis/move` (`robot_interfaces/action/ChassisMove`)
- Action `/chassis/span` (`robot_interfaces/action/ChassisSpan`)

默认话题名与 `robot_platform` 一致，可通过参数覆盖。

## 依赖

- ROS 2 包依赖已在 `package.xml` 中声明（含 `robot_platform` + 轻量接口包 `robot_interfaces`）。
- Python Web 依赖（FastAPI + Uvicorn）在 `setup.py` 的 `install_requires` 中声明，`colcon build` 时会自动安装（推荐在构建前 `pip install --upgrade pip`）。

如遇到依赖问题，可手动：

```bash
pip install "fastapi>=0.110" "uvicorn[standard]>=0.30"
```

## 常见问题

- 浏览器无法访问：确认防火墙放行端口，`web_host` 是否为 `0.0.0.0`。
- 启动时报 “librobot_platform__rosidl_typesupport_*.so 找不到” 或 “Type support not from this implementation”：**必须同时构建** `robot_platform` 和 `manual_control`（见上方启动命令）。单独 `colcon build --packages-select manual_control` 是不够的。
- 控制无反应：检查 `robot_platform` 是否运行，模式是否正确（MOVE 模式下 Twist 可能受限）。
- Action 一直 rejected：先通过面板或命令切换到 MOVE 模式。
- 想同时用键盘和网页：两个节点可同时运行（注意不要同时激烈操作同一自由度）。

---

原键盘说明（简要）保留供参考：

- 机械臂：方向键控制 z/j1，`,` `.` 控制 j2
- 底盘：WASD 速度控制，松键超时归零
- 话题与上面一致

完整键盘参数见源码 `manual_control_node.py`。
