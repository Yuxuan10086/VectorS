# `scan_safety`

**控制模块子包**：基于 `sensor_msgs/LaserScan` 的 **简单反应式避障**，订阅上层速度与雷达扫描，发布滤波后的 `cmd_vel`。可通过参数 **`enabled`** 关闭实现直通，便于与日后 Nav2 等切换。

## 节点：`simple_scan_safety`

- **订阅**
  - `scan_topic`（默认 **`scan`**）：与 `oradar_lidar` 等驱动一致。
  - `cmd_vel_in`（默认 **`cmd_vel_raw`**）：上层期望速度（遥控应对准该话题）。
- **发布**
  - `cmd_vel_out`（默认 **`cmd_vel`**）：底盘驱动应对准该话题。
  - `enabled_topic`（默认 **`avoidance_enabled`**，`publish_enabled_state` 为 true 时）：当前避障逻辑是否启用（`std_msgs/Bool`）。

### 行为与参数

在 **`forward_angle_rad`** 为中心、半宽 **`forward_half_angle_rad`** 的扇区内取最近有效距离；若小于 **`min_distance_m`**，则按 **`allow_reverse_when_blocked`** / **`allow_rotation_when_blocked`** 限制速度。

若 **`require_scan_before_move`** 为 true，在未收到激光前输出零速度。

常用参数见源码 `simple_scan_safety.py` 中 `declare_parameter`；运行时：`ros2 param set /simple_scan_safety enabled false`

## Launch

```bash
ros2 launch scan_safety simple_scan_safety.launch.py
```

## 典型串联

1. `ros2 launch oradar_lidar ms200_scan.launch.py`
2. `ros2 launch scan_safety simple_scan_safety.launch.py`
3. 遥控 remap：`cmd_vel` → `cmd_vel_raw`，例如  
   `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_raw`

## 编译

```bash
colcon build --packages-select scan_safety
```
