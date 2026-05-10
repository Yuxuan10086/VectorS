# `robot_ws` 源码说明（TRT 机器人）

本文描述本工作空间 `src` 下整机配置、包结构与模块分工。

---

## 1. 机器人功能与参数（概要）

| 项目 | 说明 |
|------|------|
| **主控** | Intel **J1900**，**4 GB** 内存，**32 GB** 存储 |
| **形态** | **Centerm 瘦客户端** 硬件平台 |
| **系统** | **Ubuntu 22.04 Server** |
| **中间件** | **ROS 2**（建议 **Humble**，与 22.04 配套） |
| **网络（开发）** | 固定 IP **`192.168.66.66`**，主机名 **`robot`** |
| **底盘** | **前驱差速**，后轮为从动 **全向轮** |
| **框架** | 铝型材，外形约 **450 mm × 450 mm** |
| **机械臂** | **三轴 SCARA**，高度约 **350 mm**；**Z 轴丝杆**；两串联平面关节 **同步带** |
| **电机** | 共 **5** 台 **正点原子 PD42** 步进闭环，**FOC**，**CAN**（扩展帧协议），详见 `robot_driver` |
| **电源** | **24 V** 电池；持续约 **6 A**，最大约 **10 A**；标称 **2550 mAh** |
| **保护** | 各路电机 **5 A 独立保险丝**；主控经 **降压稳压** 由电池供电；具备 **急停开关** |

---

## 2. 模块与包结构

### 2.0 目录说明

- **`robot_driver/`、`robot_bringup/`、`robot_perception/`、`robot_tasks/`**：目录名对应 ROS **功能包名**（根目录含 `package.xml`）。
- **`robot_control/`**：仅为 **控制大类目录**（根目录**无** `package.xml`），其下再建 **多个子功能包**（如避障、速度仲裁等），各子包独立编译。

### 2.1 顶层与模块对照

| 名称 | 角色 |
|------|------|
| **`robot_driver`** | 底层驱动与协议库（电机 CAN / 雷达串口等），供上层链接或启动其自带节点。 |
| **`robot_control/`** | 控制 **模块文件夹**；子包见下及 **`robot_control/README.md`**。 |
| **`robot_bringup`** | （预留）整机启动、参数与模式组合 launch。 |
| **`robot_perception`** | （预留）感知算法与其它传感器处理。 |
| **`robot_tasks`** | （预留）高层任务与业务逻辑。 |

**`robot_control/` 下子包（当前）**

| 子目录 | ROS 包名 | 说明 |
|--------|----------|------|
| `scan_safety/` | `scan_safety` | 基于 LaserScan 的简单避障（`simple_scan_safety` 节点），见该目录 `README.md`。 |

### 2.2 `robot_driver`（摘录）

- **`motor`（包名 `robot_driver`）**  
  PD42 正点原子协议、**SocketCAN** 收发与应用层封装；差速示例与 REPL 示例见该目录 **`README.md`**（含 `pd42_cycle_example`、`pd42_test2` 等运行方式）。

- **`oradar_lidar`（包名 `oradar_lidar`）**  
  Oradar **MS200** 二维激光雷达 ROS 2 驱动：发布 **LaserScan** / **PointCloud2**，launch 与 SLAM/导航衔接说明见包内 **`README.md`**。

### 2.3 `robot_control` 子包（摘录）

- 模块说明与后续如何新增子包：见 **`robot_control/README.md`**。  
- 避障子包 **`scan_safety`**：订阅 **`cmd_vel_raw`** 与 **`/scan`**，发布 **`cmd_vel`**，参数 **`enabled`** 可直通。详见 **`robot_control/scan_safety/README.md`**。

---

## 3. 编译与环境

```bash
cd ~/robot_ws   # 或你的工作空间根目录
colcon build
source install/setup.bash
```

---

## 4. 作者与单位

**TRT 机器人实验室**，隶属于 **河北工程大学数理科学与工程学院**。

- **队长**：张宇轩  
- **成员**：陶相晨、张佳怡、李晶晶、王运迎、王正宇  

---

*文档随仓库迭代更新；各子包细节以对应目录内 `README.md` 为准。*
