# motor

ROS 2 功能包：PD42（正点原子协议）步进闭环驱动的 **SocketCAN 收发** 与 **应用层 SDK**，供上层节点或其它包链接使用。

## 依赖与前置条件

- Linux **SocketCAN**（如 `can0`），波特率等与驱动一致（常用 **500 kbit/s**）。
- 使用前对 **`CanInterface`** 调用 **`open()`**；进程结束前 **`close()`**（析构也会关闭）。
- 协议细节与注意事项见同目录 **`doc/电机手册提取.md`**；CAN 帧拆分与队列行为见 **`doc/can_interface_README.md`**。

## 链接方式（CMake）

```cmake
find_package(motor REQUIRED)
# 若通过 ament 导出目标名不同，以包内 CMake 导出为准；当前构建产物为静态库 robot_driver_pd42
target_link_libraries(your_target PRIVATE motor::robot_driver_pd42)
target_include_directories(your_target PRIVATE ${motor_INCLUDE_DIRS})  # 或依赖 ament 传递的头文件路径
```

具体以工作空间 **`colcon build`** 后 **`install`** 布局为准；也可在同一 workspace 里 **`target_link_libraries(... robot_driver_pd42)`** 并 **`target_include_directories(... include)`**。

## 类与职责

| 组件 | 作用 |
|------|------|
| **`CanInterface`** | 扩展帧发送（下行 ID 固定 **0x1001**）、按路由键组帧接收、发送队列与 ≥2 ms 指令间隔。 |
| **`pd42_protocol.hpp`** | 自定义串口/CAN 载荷：组帧、校验、`default_can_eff_id`、读应答解析等（无 I/O）；含 **0xF0 / 0x90 / 0x98 / 0x99** 等组帧。 |
| **`Pd42Motor`** | 面向单轴：构造时下发 **0x6C + 0x04**；其余指令经私有 **`send_cmd`** 发送并阻塞收应答（应答错误字节非成功则失败）。 |

## `Pd42Motor` 使用步骤（推荐顺序）

1. **`CanInterface can; can.open();`**（绑定网卡名见 `src/can_interface.cpp` 内 `kCanSocketIfName`）
2. **`Pd42Motor motor(can, motor_id [, stall_current_ma]);`**  
   - `motor_id` 与协议帧内 **从机地址**一致（1–255）；与上行 CAN 扩展帧路由 **`default_can_eff_id(addr)`**（低 4 位）一致。  
   - 第三参数为构造后 **`enable_stall_protection`** 用的堵转电流（mA），省略时默认 **2000**。
3. **`motor.initialize()`** — 清状态 **0xFB** + 使能 **0xFA**。
4. 按工况 **`motor.set_mode(Pd42CommMode::kPosition / kSpeed / kTorque)`**。  
   - **内部会先失能再使能再发 0x62**（规避驱动内部目标位置异常）。
5. 速度模式：**`set_speed(...)`**（**0xF1**）；位置模式：**`set_absolute_position(...)`**（**0xF2**）；力矩模式：**`set_torque(reverse, current_ma)`**（**0xF0**，电流单位 **mA**，范围 **0~3000**）。  
   - **三类运动指令均不负责切换模式**：须先 **`set_mode`** 切到对应 **`kPosition` / `kSpeed` / `kTorque`**；SDK 用 **`comm_mode()`** 记录最近一次成功的 **`set_mode`**。模式不符则 **`error_code()==0xFA`**，不下发该帧。  
   - 位置与限位原点单位：**51200 为一圈**（与读位置 **0x2A** 一致）。  
   - **绝对位置帧中的方向位**（`set_absolute_position` 末参 **`reverse`**，载荷中为 **0** 与 **1** 两档）：**取正向（`reverse == false`，即方向位为 0）时，驱动器按最短、最直观的角位移轨迹趋近目标位置**；**取反向（`reverse == true`，即方向位为 1）时，等效于刻意沿另一旋转方向绕行，角行程显著增大**（可类比为舍近求远）。上层**不必**依据「当前位置读数与目标脉冲谁大谁小」自行推断应发正向还是反向；**默认采用正向即可得到符合直觉的走法**；仅在确有绕远、多圈等工艺需求时，再显式选用反向。

6. （可选）左右限位：**`set_limit_origins(left, right)`** 依次下发左 **0x90**、右 **0x98**；**`set_limit_sw(enable)`** 为 **0x99**（开启后行程受限于已设置的左右原点）。
7. **急停：`stop()`** — 先发 **0xFC**，成功后立即 **`clear_status()`（0xFB）**，避免驱动长期耗转发烫。
8. **读反馈（阻塞）：** **`rpm()`**（0x29）、**`pos()`**（0x2A）、**`arrived_flag()`**（0x30，到位标志 0/1），成功为 `optional` 有值，失败查 **`error_code()`**。  
   - 角度/位置计数清零：**`set_zero_position()`**（**0xF8**）。

## 返回值与 `error_code()`

- **控制类**（`initialize`、`set_speed`、`set_torque`、`set_mode`、`set_limit_origins`、`set_limit_sw`、`set_zero_position` 等）：`bool`，失败时 **`error_code()`** 非 0。  
  - **0xE1–0xE6**：手册协议错误（应答数据首字节）。  
  - **0xFF**：等待应答超时；**0xFE**：入队失败；**0xFC**：应答不匹配或解析失败；**0xFB**：下发帧过短（SDK 自检）。  
  - **0xFA**：当前 **`comm_mode()`** 与指令不匹配（例如未先 **`set_mode(kSpeed)`** 就 **`set_speed`**，或未 **`set_mode(kTorque)`** 就 **`set_torque`** / **`set_absolute_position`**）。  
  - 手册 **0xFB** 功能码（清状态）与 SDK 哨兵 **0xFB** 含义不同，请以上下文区分。
- **读取类**（`rpm`、`pos`）：`std::optional`，无值时表示失败，同样看 **`error_code()`**。

## 交互示例可执行文件

```bash
ros2 run motor pd42_motor_repl
```

指令为短名 REPL（`init()`、`spd(100)`、`rpm()` 等），源码见 **`example/pd42_motor_repl.cpp`**。用于单电机协议调试；差速底盘请使用 **`chassis`** 包的 **`DiffDriveChassis`**（由 **`robot_platform`** 集成）。

## 不在本 SDK 内封装的内容

- **ROS 2 节点 / 话题 / 服务**：由上层包编写，仅链接本库即可。
- **协议里已有组帧、但未通过 `Pd42Motor` 暴露的指令**：可直接 **`#include <robot_driver/pd42_protocol.hpp>`**，自行 **`can.send(...)` + 自管应答**（谨慎处理与 `Pd42Motor` 共享同一 `CanInterface` 时的收发顺序）。

### 后续：底盘电机 / 关节电机（继承，可选）

- **`Pd42Motor`**：保留完整 **`set_mode` / `set_speed` / `set_absolute_position`**，作为 **通用协议门面**。  
- **`ChassisMotor`（示例名）**：公有继承 **`Pd42Motor`**，对外只暴露 **`set_speed`、`rpm`、`stop`、`initialize`** 等；内部 **`set_mode(kSpeed)`** 固定在初始化路径，**不提供或禁用位置模式 API**（可用 `private using` / 不转发 `set_absolute_position`，或在文档约定勿调用）。  
- **`JointMotor`（示例名）**：同样继承 **`Pd42Motor`**，固定 **`set_mode(kPosition)`**，对外 **`set_absolute_position`、`pos`**，弱化或隐藏 **`set_speed`**。  

构造仍可 **`JointMotor(CanInterface &, uint8_t id) : Pd42Motor(...)`**；是否再加一层「底盘驱动」聚合四个轮子属上层 ROS 节点职责，不必塞进 `robot_driver`。
