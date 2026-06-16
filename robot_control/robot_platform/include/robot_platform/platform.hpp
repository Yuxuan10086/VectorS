#ifndef ROBOT_PLATFORM__PLATFORM_HPP_
#define ROBOT_PLATFORM__PLATFORM_HPP_

#include <atomic>
#include <memory>

#include "chassis/diff_drive_chassis.hpp"
#include "robot_driver/can_interface.hpp"
#include "scara_arm/robot_arm.hpp"

namespace rclcpp
{
class Node;
}  // namespace rclcpp

namespace robot_platform
{

/** 整机硬件装配：拥有一个 CanInterface，并持有底盘与机械臂 SDK 对象 */
class Platform
{
public:
  explicit Platform(rclcpp::Node & node);
  ~Platform();

  Platform(const Platform &) = delete;
  Platform & operator=(const Platform &) = delete;

  bool open();
  void close();
  bool is_open() const { return open_; }

  chassis::DiffDriveChassis & chassis();
  scara_arm::RobotArm & arm();

  // IMU 零点 offset：用于 map->base_link 的航向修正
  // reset_imu_zero() 将 offset 设为当前原始偏航角，使当前朝向成为 map 系的 yaw=0
  void update_imu_yaw(double raw_yaw);
  void reset_imu_zero();
  double get_corrected_imu_yaw() const;

private:
  enum class OpenAttemptResult { kOk, kRetry, kFatal };

  bool load_chassis_params(chassis::DiffDriveParams & out);
  bool load_scara_arm_params(scara_arm::ScaraArmParams & out);
  OpenAttemptResult open_once();
  void reset_hardware_state();

  rclcpp::Node & node_;
  robot_driver::CanInterface can_;
  std::unique_ptr<chassis::DiffDriveChassis> chassis_;
  std::unique_ptr<scara_arm::RobotArm> arm_;
  bool open_{false};

  std::atomic<double> current_imu_yaw_raw_{0.0};
  double imu_yaw_offset_{0.0};
};

}  // namespace robot_platform

#endif  // ROBOT_PLATFORM__PLATFORM_HPP_
