# `robot_ws` 源码说明 · TRT 机器人实验室

本工作空间 `src` 下整机配置、包结构与模块分工。由 **TRT 机器人实验室** 维护；**TRT** 为实验室名称，非机器人型号。

网页版：[`README.html`](README.html)

---

## 1. 机器人功能与参数

移动作业机器人：**车载计算**负责整机软件运行；**移动平台**承载并定位；**SCARA 作业臂**完成取放；**电气系统**统一供电并保护。驱动层由 **`robot_driver`** 模块中的 CAN 电机栈贯通底盘与机械臂。

**结构关系图**见 [`README.html#spec`](README.html#spec) 中的 SVG 示意图。

### 车载计算

运行导航、感知与机械臂等全部 ROS 节点。

| | |
|:--|:--|
| 硬件平台 | Centerm 瘦客户端 |
| 处理器 | Intel J1900 |
| 内存 / 存储 | 4 GB / 32 GB |
| 操作系统 | Ubuntu 22.04 Server |
| 中间件 | ROS 2 Humble |
| 机载网络 | `192.168.66.66`，主机名 `robot` |

### 移动平台

承担场地内行驶与停靠；与作业臂固定在同一机架上。

| | |
|:--|:--|
| 机架 | 铝型材，约 **450 mm × 450 mm** |
| 驱动 | 前驱差速 |
| 后轮 | 从动全向轮 |

### SCARA 作业臂

安装在移动平台上，面向货架完成底层取放。

| | |
|:--|:--|
| 构型 | 三轴 SCARA，工作高度约 **350 mm** |
| Z 轴 | 丝杆 |
| 平面关节 | 同步带传动 |

### 驱动与通信

移动平台与机械臂共 **5** 台 **正点原子 PD42** 步进闭环，**FOC**，**CAN** 扩展帧；协议与接口见 **`robot_driver/motor`**。

### 电气与安全

为底盘电机、机械臂电机及主控分配电源，并具备独立保护与急停。

| | |
|:--|:--|
| 动力电池 | **24 V**，**2550 mAh** |
| 工作电流 | 持续约 **6 A**，峰值约 **10 A** |
| 电机支路 | 每路 **5 A** 独立保险丝 |
| 主控供电 | 电池经降压模块供电 |
| 安全 | 急停开关 |

---

## 2. 模块与包结构

### 2.0 目录说明

- **`robot_bringup/`、`robot_perception/`、`robot_tasks/`**：ROS 功能包，根目录含 `package.xml`。
- **`robot_driver/`、`robot_control/`**：模块目录，根目录无 `package.xml`，下挂多个子功能包。

### 2.1 顶层与模块对照

| 名称 | 角色 |
|------|------|
| **`robot_driver/`** | 驱动模块目录 |
| **`robot_control/`** | 控制模块目录，见 **`robot_control/README.md`** |
| **`robot_bringup`** | 预留：整机 launch |
| **`robot_perception`** | 预留：感知 |
| **`robot_tasks`** | 预留：任务逻辑 |

**`robot_control/` 子包**

| 子目录 | ROS 包名 | 说明 |
|--------|----------|------|
| `robot_interfaces/` | `robot_interfaces` | 自定义 msg/srv/action，见 **`robot_control/README.md`** |
| `robot_platform/` | `robot_platform` | 整机硬件节点，见 **`robot_control/README.md`** |
| `manual_control/` | `manual_control` | Web 控制面板，见包内 `README.md` |

**`robot_driver/` 子包**

| 子目录 | ROS 包名 | 说明 |
|--------|----------|------|
| `motor/` | `motor` | PD42 / SocketCAN，见包内 `README.md` |
| `chassis/` | `chassis` | 前驱差速底盘 SDK，见 `config/chassis.yaml` |
| `scara_arm/` | `scara_arm` | SCARA 机械臂 SDK，见包内 `README.md` |
| `oradar_lidar/` | `oradar_lidar` | MS200 激光雷达驱动，见包内 `README.md` |

### 2.2 `robot_driver` 子包

- 模块说明：**`robot_driver/README.md`**（各包接口总表）
- **`motor`**：CAN 与 Pd42 协议（C++ 库，无 ROS）
- **`chassis`**、**`scara_arm`**：设备 SDK，供 `robot_platform` 链接
- **`wit_ros2_imu`**：IMU 节点，发布 `/imu/data_raw`
- **`oradar_lidar`**：激光雷达，发布 `/scan` 等

### 2.3 `robot_control` 子包

- 模块说明：**`robot_control/README.md`**（各包 ROS 接口总表）
- **`robot_interfaces`**：接口定义
- **`robot_platform`**：`platform_node`（底盘 + 机械臂 + IMU + TF）
- **`manual_control`**：Web 控制面板

---

## 3. 编译与环境

```bash
cd ~/robot_ws
colcon build
source install/setup.bash
```

---

## 4. 技术方案

**TRT 机器人实验室** 参赛机器人采用分层对准：固定路线粗定位 → 激光精调 → 视觉精调 → 机械自适应兜底。物块贴 **AprilTag**，仅作业**底层**。

| 层级 | 内容 |
|------|------|
| **路线** | 沿预设固定路线行驶，到达面向目标物块的停靠位 |
| **底盘底层** | **IMU** 与**底盘 SDK** 融合，只参与底层运动控制：根据实际运动与速度指令的误差做修正 |
| **激光** | 二维激光调整与柜台（货架）的**距离**与**平行度** |
| **视觉** | **单目相机**、无深度；识别 AprilTag，解算物块位姿，修正**横向**位置 |
| **机械** | SCARA 与机构自身的**自适应**为最后一层兜底，吸收残余误差 |

**数据流概要**：`cmd_vel` 经底盘闭环执行 → 到点后激光对齐柜面 → 相机对 tag 横向微调 → 机械臂抓取/放置。

---

## 5. 竞赛任务说明

场地 **400 cm × 400 cm**，A～F 六区；核心区域须保留。平面示意、立体结构与得分见 **[`README.html`](README.html)** 第 5 节。

**取货层**：双层货架，底层约 25–30 cm、高层约 65–75 cm。本队末端高度 **≤40 cm**，仅做底层取放；B/C 巷道 **130 cm**。

**任务**：赛前声明自主/人工；抽 **2** 张取货号；**2** 块物块全部入 **A 区**；可选 **D 区** +20 分；结束入 **F 区**；自主运行不得人工操控。

**计分**：调试 **40 min**，单次 **10 min**；技术 **80** + 报告 **20**；拿货/放置各 **20**/件，封顶 **80**，出界 **−10**/件；D 区 **+20**；违规 **−5**/次；未进 F **−5**；系数：自主导航 **1.5**、算法 **1.2**、人工介入 **1.0**、全程人工 **0.75**。细则以赛项群为准。

---

## 6. 作者与单位

**TRT 机器人实验室**，**河北工程大学数理科学与工程学院**

- **队长**：张宇轩  
- **成员**：陶相晨、张佳怡、李晶晶、王运迎、王正宇  

---

各子包细节见对应目录 `README.md`。
