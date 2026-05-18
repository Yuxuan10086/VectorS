# robot_platform

整机硬件装配包：拥有一个 **`robot_driver::CanInterface`**，并实例化：

- **`chassis::DiffDriveChassis`**（参数见 **`chassis/config/chassis.yaml`**）
- **`scara_arm::RobotArm`**（参数见 **`scara_arm/config/scara_arm.yaml`**）

C++ 类 **`robot_platform::Platform`** 在 `open()` 时从 `rclcpp::Node` 读取上述参数，不在本包内重复定义默认值。

## 依赖

`motor`、`chassis`、`scara_arm`（均为独立 ament 包，与目录位置无关）。

## 编译与运行

```bash
cd ~/robot_ws
colcon build --packages-select motor chassis scara_arm robot_platform
source install/setup.bash
ros2 launch robot_platform platform.launch.py
```

## 在上层代码中使用

```cpp
#include "robot_platform/platform.hpp"

// node 已通过 launch 加载 chassis.yaml + scara_arm.yaml
robot_platform::Platform platform(*node);
if (!platform.open()) { /* ... */ }
platform.chassis().stop();
platform.arm().calibrate();
platform.close();
```
