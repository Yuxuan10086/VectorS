# `oradar_lidar` — Oradar MS200 激光雷达（ROS 2）

本功能包在 ROS 2 下发布 **LaserScan**（`oradar_scan`）或 **PointCloud2**（`oradar_pointcloud`），适用于 Oradar MS200 系列 2D 机械旋转激光雷达。

源码目录名与 ROS 包名一致：`oradar_lidar`（激光雷达常用英文名 **LiDAR**）。

## 环境与编译

- **ROS 2**：Foxy 及以上（本工作空间实测 Humble）。
- **编译**：确保 `CMakeLists.txt` 顶部 `COMPILE_METHOD` 为 **`COLCON`**（默认已是）；构建时使用仓库里的 `package_ros2.xml` 会在 CMake 配置阶段复制为 `package.xml`。

```bash
cd ~/robot_ws   # 你的 workspace 根目录
colcon build --packages-select oradar_lidar
source install/setup.bash
```

## 串口权限与参数

1. 查看设备节点：`ls -l /dev/ttyUSB*`（或实际枚举到的串口）。
2. 将用户加入 `dialout` 组或使用 udev 规则固定别名（例如 `/dev/oradar`），避免反复 `chmod`。

常用参数（可在对应 launch 里改）：

| 参数 | 含义 |
|------|------|
| `port_name` | 串口路径，默认 `/dev/ttyUSB0` |
| `baudrate` | 波特率，默认 `230400` |
| `frame_id` | 雷达坐标系名，LaserScan 默认 `laser_frame` |
| `scan_topic` | LaserScan 话题名，默认 **`scan`**（与 SLAM / Nav2 惯例一致） |
| `cloud_topic` | PointCloud2 话题名，默认 **`point_cloud`** |
| `range_min` / `range_max` | 有效测距范围（米） |

## 启动雷达节点（ROS 2）

```bash
# 发布 LaserScan（默认含 base_link → laser_frame 静态 TF，高度 0.18 m 可按实车修改）
ros2 launch oradar_lidar ms200_scan.launch.py

# 同上 + RViz2
ros2 launch oradar_lidar ms200_scan_view.launch.py

# 发布 PointCloud2
ros2 launch oradar_lidar ms200_pointcloud.launch.py
```

节点发布的话题：

- **LaserScan**：默认 **`/scan`**，`frame_id` 默认 **`laser_frame`**。
- **静态 TF**（`ms200_scan.launch.py`）：`base_link` → `laser_frame`（平移 z=0.18，可按安装高度改 launch）。

若你已有整车 URDF/robot_state_publisher，请避免重复发布冲突的 `base_link`→雷达 TF：可自建 launch 只启动 `oradar_scan`，并在 URDF 里定义雷达关节与 `frame_id` 一致。

---

## 建图（SLAM）

本驱动只负责 **`sensor_msgs/LaserScan`**。建图还需 **里程计**（`nav_msgs/Odometry`，常见 `odom`→`base_link` TF）以及 **`base_link`→`laser_frame`** TF（launch 里静态 TF 或 URDF）。

### 使用 slam_toolbox（推荐，ROS 2 常用）

安装（以 Ubuntu 22.04 / **Humble** 为例）：

```bash
sudo apt install ros-humble-slam-toolbox
```

典型流程：

1. 终端 A：底盘/里程计（发布 `odom`→`base_link`；若无真底盘，可用仿真或键盘遥控栈代替）。
2. 终端 B：`ros2 launch oradar_lidar ms200_scan.launch.py`（确认 `/scan`、`base_link`→`laser_frame`）。
3. 终端 C：异步在线建图，例如：

```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml
```

若默认参数里扫描话题不是 `/scan`，可用 **remap** 或在 yaml 里改 `scan_topic`。

保存地图（示例，具体以 slam_toolbox 文档为准）：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

---

## 定位与导航

### 已知地图定位（AMCL + map_server）

1. 启动地图服务器与 AMCL（常用 **Nav2** 自带 bringup，或单独启动 `map_server` + `amcl`）。
2. 确保 **`map`→`odom`→`base_link`→`laser_frame`** TF 链完整：AMCL 根据 `/scan` 与地图修正 `map`→`odom`。
3. 雷达侧仍使用：`ros2 launch oradar_lidar ms200_scan.launch.py`，保证 **`frame_id`** 与代价地图 / AMCL 配置中的激光层一致（常为 `laser_frame` 或在 URDF 中与配置统一）。

### 纯激光建图式定位

也可使用 **slam_toolbox 定位模式**（localization）在已有栅格地图上运行，同样需要里程计与 TF；配置参见 `slam_toolbox` 官方文档。

---

## 与导航栈（Nav2）对接要点

- **扫描话题**：默认 **`/scan`**，一般无需 remap。
- **坐标系**：代价地图通常订阅 `odom`/`map` 与激光；请保证 `robot_description` 或静态 TF 中 **`base_link` 与雷达 `frame_id` 关系正确**。
- **频率与延时**：若 Nav2 报 TF 或扫描超时，检查串口带宽、电机转速 `motor_speed` 与上位机负载。

---

## ROS 1 说明（可选）

若需在 ROS 1 使用：将 `CMakeLists.txt` 中 `COMPILE_METHOD` 改为 **`CATKIN`**，并用 `package_ros1.xml` 覆盖为 `package.xml` 后 `catkin_make`。ROS 1 / ROS 2 **不建议**混装在开发者同一环境里。

更多英文说明见仓库内 `README_en.md`。
