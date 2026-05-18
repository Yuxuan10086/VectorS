#ifndef ROBOT_PLATFORM__PLATFORM_HPP_
#define ROBOT_PLATFORM__PLATFORM_HPP_

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

  robot_driver::CanInterface & can();
  chassis::DiffDriveChassis & chassis();
  scara_arm::RobotArm & arm();

private:
  bool load_chassis_params(chassis::DiffDriveParams & out);
  bool create_arm();

  rclcpp::Node & node_;
  robot_driver::CanInterface can_;
  std::unique_ptr<chassis::DiffDriveChassis> chassis_;
  std::unique_ptr<scara_arm::RobotArm> arm_;
  bool open_{false};
};

}  // namespace robot_platform

#endif  // ROBOT_PLATFORM__PLATFORM_HPP_
