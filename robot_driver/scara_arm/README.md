# scara_arm

**SCARA 机械臂控制包**：基于 **`motor`**（Pd42 / SocketCAN）的 **`ArmJoint`** / **`RobotArm`**，提供碰停限位标定、脉冲段与逻辑幅度 **`span`** 映射、**`set_position`** 等；库代码无 ROS 依赖。

示例可执行文件（均由 **`colcon build`** 安装到 `lib/scara_arm/`，通过 **`ros2 run scara_arm <名>`** 启动）：

| 可执行名 | 源码 | 作用 |
|---------|------|------|
| **`scara_arm_joints_repl`** | `examples/scara_arm_joints_repl.cpp` | 读 ROS 参数后构造 **J1/J2** 关节与电机；**不上电自动标定**；终端支持 **关节命令**（`j1` / `j2`）与 **直连电机**（`motor_j1_id` / `motor_j2_id`）。**不含 Z 轴与 `RobotArm`**，用于单轴/双轴调试。 |

整臂标定与运动请使用 **`robot_platform`**（`Platform` + ROS Action/Service）。

**`colcon build`** 后提供静态库 **`scara_arm::scara_arm`**（`ArmJoint` / `RobotArm`），供 **`robot_platform`** 等包 `find_package(scara_arm)` 链接。

**`set_position(span)`**：逻辑 **`span ∈ [0, span_max]`**（YAML `span_*`）线性对应脉冲 **`[0, H]`**；因两侧各收缩 **`limit_margin_units`**，**实际可达 span 子区间**由 **`(margin/H)·span_max`** 与名义 **`span_max`** 对称推出。**`ArmJoint::set_limits()`** 仅返回该单侧 span 裕量；**整臂可达区间由 `RobotArm` 在标定后写入公有成员** `reachable_span_min_z` / `reachable_span_max_z`（及 J1/J2 同名）。越界返回失败，**不做静默夹紧**。

## 流程概要（标定）

1. **外部** 创建并 **`CanInterface::open()`** 后，用 **`ScaraArmParams`**（三轴各一 **`ArmJointParams`** + 标定扭矩）构造 **`RobotArm(can, params)`**（可与底盘**共用同一** `CanInterface`）；**`RobotArm` 不拥有、不关闭** CAN。随后各轴 **`ArmJoint(motor, joint_params)`**：**initialize → 关限位 → 位置模式**，**不在构造时清零脉冲**（零点在反向 bump 末 **`set_zero_position`**）。参数类型定义在 **`arm_joint.hpp`**（`ArmJointParams`）与 **`robot_arm.hpp`**（`ScaraArmParams`）。  
2. **`bump`**：速度模式 + 相电流判到位。**反向**：到位后清零。**正向**：记录 **`stop_fwd_`**。  
3. **`set_limits`**：以 **`H`**（正向碰停脉冲）为标度，驱动限位脉冲为 **`[margin, H-margin]`**；**返回**单侧 span 裕量 **`(margin/H)·span_max`**。**`RobotArm::calibrate()`** 据此与各轴名义 **`span_max`** 写入公有成员 **`reachable_span_min_*` / `reachable_span_max_*`**。  
4. **`calibrate()`**：整臂标定流程（**先 Z 碰停与限位**，再 **J2 / J1** 等；细节以 `robot_arm.cpp` 为准）；由 **`robot_platform`** 的 `/arm/calibrate` Action 触发。

## 依赖

- 同 workspace 已编译 **`motor`**  
- Linux **CAN**（SocketCAN；**网卡名**在 **`motor`** 的 `src/can_interface.cpp` 内常量 **`kCanSocketIfName`**，与 YAML 无关）

## 编译

```bash
cd ~/robot_ws
colcon build --packages-select motor scara_arm
source install/setup.bash
```

## 配置

编辑 **`src/robot_driver/scara_arm/config/scara_arm.yaml`**（源码）。  
**注意**：`ros2 pkg prefix scara_arm` 指向 **install** 目录；仅改源码下的 YAML 而不执行 **`colcon build`**（安装步骤）时，`share/scara_arm/config/` 里的副本**不会更新**。要让「改文件 → 重启节点即生效」而又不想每次安装：

- **推荐**：启动前指定源码 YAML（绝对路径或相对于当前 shell `cwd` 的路径）：
  ```bash
  ros2 run scara_arm scara_arm_joints_repl --ros-args --params-file ~/robot_ws/src/robot_driver/scara_arm/config/scara_arm.yaml
  ```
- **环境变量**（便于 launch）：若设置 **`SCARA_ARM_PARAMS_FILE`** 为上述 YAML 的绝对路径，则 **`ros2 launch scara_arm scara_arm_joints_repl.launch.py`** 会优先加载该文件（launch 内解析顺序见 `launch/scara_arm_joints_repl.launch.py`）。
- **Launch 参数**：`params_file:=/path/to/scara_arm.yaml` 优先级最高。
- 若仍希望继续用 **`$(ros2 pkg prefix scara_arm)/share/...`**，则在每次改 YAML 后执行 **`colcon build --packages-select scara_arm`**（无需清编译，仅刷新 install 文件）。

| 参数 | 含义 |
|------|------|
| `motor_*_id` | Z / 关节1 / 关节2 从机地址（**均须非 0**）；**`scara_arm_joints_repl`** 仅使用 `motor_j1_id` / `motor_j2_id`。 |
| `torque_z_up_ma` / `torque_z_down_ma` | Z 标定 **`bump(0)` / `bump(1)`** 相电流阈值（mA）；常见对应上升 / 下降，与丝杆方向一致时请按实机核对 |
| `torque_j1_ma` / `torque_j2_ma` | J1/J2 碰停 **相电流到位阈值**（mA，0~3000）；**`RobotArm::calibrate()`** 与 **`scara_arm_joints_repl`** 的 bump 使用 |
| `stall_current_*_ma` | 各轴 **常态** 堵转保护电流（mA，0~3000）；**`Pd42Motor` 构造**等 |
| `bump_speed_rpm_*` | 碰停标定 **`set_speed`** 目标转速（rpm）；加速度档复用 **`position_accel_*`** |
| `position_speed_rpm_*` | 各关节 **`set_absolute_position`** 目标转速（rpm） |
| `position_accel_*` | **`set_absolute_position`** / 碰停 **`set_speed`** 加速度档（0~255） |
| `limit_margin_units` | 固件行程 **下限侧脉冲**（与正向碰停点上沿构成区间）；标定前请保证 **反向 bump 先执行并已清零** |
| `span_joint1` / `span_joint2` / `span_z` | 各轴逻辑行程上限（用户单位自定） |
| （碰停轮询） | 相电流采样周期与寻边超时见 **`arm_joint.hpp`** 中 **`kBumpStallPollPeriodMs`**、**`kBumpStallSeekTimeoutS`**，不由 YAML 配置 |

## 运行

**必须用 `ros2 run` 直连终端**：`ros2 launch` 启动的子进程 **stdin 通常不是 TTY**，读不到键盘输入，交互命令不会执行。

### `scara_arm_joints_repl`：J1/J2 关节 + 直连电机

首词为 **`j1` / `j2`** 接 `bump()`、`move(span)` 等；或以 **`motor_j1_id` / `motor_j2_id`** 为行首直连电机命令。参数表见上（**不要求** `motor_z_id` 等 Z 轴键，但可与完整 YAML 共存，未用键可保留）。

```bash
ros2 run scara_arm scara_arm_joints_repl --ros-args --params-file ~/robot_ws/src/robot_driver/scara_arm/config/scara_arm.yaml
```

### 仅想加载 YAML、不依赖本机键盘时

可用 launch（若节点在无 TTY 下会打印说明并退出，属预期行为）：

```bash
ros2 launch scara_arm scara_arm_joints_repl.launch.py
```

**安全提示**：碰停前确认行程内无障碍物与人；电流请从小到大试。
