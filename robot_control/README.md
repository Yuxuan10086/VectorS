# 控制模块（`robot_control/`）

本目录是 **工作空间内「控制」大类**，**本身不是 ROS 包**（根目录无 `package.xml`）。  
具体功能拆成 **多个并列子功能包**，按需 `colcon build` 对应包名。

## 子功能包一览

| 子目录（ROS 包名） | 说明 |
|-------------------|------|
| **`robot_platform`** | 整机硬件装配：拥有一个 `CanInterface`，实例化 **`chassis::DiffDriveChassis`** 与 **`scara_arm::RobotArm`**。参数来自 `chassis/config/chassis.yaml` + `scara_arm/config/scara_arm.yaml`（launch 一次加载，键不重复）。 |
| **`scan_safety`** | 基于 `LaserScan` 的简单避障：`cmd_vel_raw` + `/scan` → `cmd_vel`。详见包内 `README.md`。 |
| *（预留）* | 速度平滑、`twist_mux`、任务节点等。 |

机械臂与底盘 **SDK** 在 **`robot_driver/scara_arm`**、**`robot_driver/chassis`**。

## 编译示例

```bash
cd ~/robot_ws
colcon build --packages-select motor chassis scara_arm robot_platform scan_safety
source install/setup.bash
```

启动整机硬件节点（需 CAN 已 up）：

```bash
ros2 launch robot_platform platform.launch.py
```

启动避障：

```bash
ros2 launch scan_safety simple_scan_safety.launch.py
```
