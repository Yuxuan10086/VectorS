# arm_collision_calib

机械臂 **碰撞限位标定**：三轴电机 ID 与碰停电流由 **配置文件** 给出，运行时通过 **`find_package(robot_driver)`** 链接工作空间内 **`robot_driver`** 包（无绝对路径，随 workspace 移植）。

核心逻辑为 **`ArmJoint`**（构造时 initialize / 关限位 / 清零；**`bump`** 内设力矩模式）与 **`RobotArm`**（构造时打开 CAN 并创建电机与关节；**`calibrate`**），不依赖 ROS 日志；构造失败由 **`std::exception`** 打出原因。

## 流程概要

1. **`RobotArm` 构造**：打开 CAN → 三电机 → 三 **`ArmJoint`**；每个关节构造时 **initialize → 关限位 → zero_position → 力矩清零**。  
2. **`bump`**：对该电机 **设力矩模式**，再恒力矩碰停并记录位置。  
3. 关节1 **正向** 力矩直至转速稳定为低速（碰停），读位置  
4. 关节2 **正向** 碰停 → 读位置；**反向** 碰停 → 读位置  
5. 由两次碰撞位置减去 **`limit_margin_units`** 得到关节2 **左/右限位原点**，下发 **0x90/0x98** 并 **开启限位（0x99）**  
6. 关节1 **反向** 碰停 → 读位置  
7. 与步骤 3 的位置配对，同样加冗余后设置 **关节1** 限位并开启  

Z 轴仅参与初始化与清零（不参与碰停）；若需 Z 向标定可在此基础上扩展。

## 依赖

- 同 workspace 已编译 **`robot_driver`**（SocketCAN）  
- Linux **CAN** 接口（默认 `can0`）

## 编译

```bash
cd ~/robot_ws
colcon build --packages-select robot_driver arm_collision_calib
source install/setup.bash
```

## 配置

编辑安装目录或源码下 **`config/arm_collision.yaml`**：

| 参数 | 含义 |
|------|------|
| `motor_*_id` | Z / 关节1 / 关节2 从机地址 |
| `torque_*_ma` | 各轴碰停探测电流（mA，0~3000） |
| `limit_margin_units` | 限位相对碰撞面向内收缩（脉冲，51200 一圈） |
| `speed_stop_threshold_rpm` | 判定堵停的 \|转速\| 上限 |
| `stall_stable_ms` | 低速需连续保持的时间 |
| `seek_timeout_s` | 单次碰停最长等待 |

## 运行

**必须**通过 `--params-file`（或 launch）加载参数；若不加载配置文件，节点会因缺少参数立即退出（代码内不设默认值）。

参数文件：

```bash
ros2 run arm_collision_calib collision_calib_node --ros-args --params-file $(ros2 pkg prefix arm_collision_calib)/share/arm_collision_calib/config/arm_collision.yaml
```

或 launch：

```bash
ros2 launch arm_collision_calib arm_collision_calib.launch.py
```

**安全提示**：碰停前确认关节行程内无障碍物与人；电流过大可能损坏结构，请先从小到大试。
