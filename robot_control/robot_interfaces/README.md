# robot_interfaces

机器人系统自定义 ROS 2 接口定义包（纯接口包，轻量）。

## 包含的接口

### 服务

- **SetDriveMode**（`/chassis/set_mode`）
  - 常量：`TWIST=0`、`MOVE=1`
  - Request: `uint8 mode`
  - Response: `bool success` + `string message`

### Action

- **ChassisMove**（`/chassis/move`）
  - 仅在 `kMove` 模式下有效
  - Goal: `distance_m`（米）、`speed_mps`（米/秒）
  - Feedback: `distance_remaining_m`
  - Result: `success`（bool）

- **ChassisSpan**（`/chassis/span`）
  - 仅在 `kMove` 模式下有效
  - Goal: `angle_rad`（弧度）、`omega_radps`（弧度/秒）
  - Feedback: `angle_remaining_rad`
  - Result: `success`（bool）

## 使用方式

其他包只需在 `package.xml` 中添加：

```xml
<depend>robot_interfaces</depend>
```

或对于 Python 节点：

```xml
<exec_depend>robot_interfaces</exec_depend>
```

然后即可 `from robot_interfaces.srv import SetDriveMode` 或 `from robot_interfaces.action import ChassisMove` 等。

实现方（robot_platform）提供服务器，消费方（manual_control、行为树节点等）只需依赖本包即可获得类型定义，完全解耦。

## 维护

接口定义发生变化时，只需在此包修改并重新构建，所有依赖方自动获得新类型。
