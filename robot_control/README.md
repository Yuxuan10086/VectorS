# 控制模块（`robot_control/`）

本目录是 **工作空间内「控制」大类**，**本身不是 ROS 包**（根目录无 `package.xml`）。  
具体功能拆成 **多个并列子功能包**，按需 `colcon build` 对应包名。

根目录下 **只应保留** 本 `README.md` 与各子功能包文件夹（如 `scan_safety/`）；**不要**在模块根下放 `src/`、`include/`、`resource/` 等——那是单个 ROS 包模板残留；若曾误创建，可直接删除。

## 子功能包一览

| 子目录（ROS 包名） | 说明 |
|-------------------|------|
| **`scan_safety`** | 基于二维激光 `LaserScan` 的 **简单反应式避障**：`cmd_vel_raw` + `/scan` → `cmd_vel`。详见包内 `README.md`。 |
| **`scara_arm`** | **SCARA 机械臂控制**（Pd42/CAN）：关节 **`ArmJoint`**、整臂 **`RobotArm`**，碰停限位标定与 `span` 映射。示例可执行文件 **`test`**（J1/J2 关节 + 直连电机终端交互）、**`test2`**（仅构造 **`RobotArm`**，终端只接受 **`calibrate`** 等整臂指令）。详见包内 `README.md`。 |
| *（预留）* | 例如速度平滑、`twist_mux` 封装、后续纯跟踪等，可按同样方式新增并列包。 |

## 编译示例

```bash
cd ~/robot_ws
colcon build --packages-select scan_safety scara_arm
source install/setup.bash
```

启动避障（包 **`scan_safety`**）：

```bash
ros2 launch scan_safety simple_scan_safety.launch.py
```

---

新增子包时：在本目录下新建独立文件夹，内含标准 `package.xml` + 源码；勿把多个节点混成一个巨型包，除非确有强耦合。
