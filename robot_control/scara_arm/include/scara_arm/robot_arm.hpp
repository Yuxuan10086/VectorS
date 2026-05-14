// SCARA 机械臂（三关节 Z / J1 / J2）
#ifndef SCARA_ARM__ROBOT_ARM_HPP_
#define SCARA_ARM__ROBOT_ARM_HPP_

#include <cstdint>
#include <memory>

#include "robot_driver/can_interface.hpp"

#include "scara_arm/arm_joint.hpp"

namespace robot_driver
{
class Pd42Motor;
}  // namespace robot_driver

namespace scara_arm
{

class RobotArm
{
public:
  // 须已 open 的共享 Can（可与底盘共用）；id_z / id_j1 / id_j2 均须非 0
  RobotArm(
    robot_driver::CanInterface & can,
    std::uint8_t id_z, std::uint8_t id_j1, std::uint8_t id_j2,
    std::int32_t margin_units,
    double span_z, double span_joint1, double span_joint2,
    std::uint16_t torque_z_up_ma, std::uint16_t torque_z_down_ma,
    std::uint16_t torque_j1_ma, std::uint16_t torque_j2_ma,
    std::uint16_t stall_current_z_ma, std::uint16_t stall_current_j1_ma,
    std::uint16_t stall_current_j2_ma,
    std::uint16_t position_speed_rpm_z, std::uint16_t position_speed_rpm_j1,
    std::uint16_t position_speed_rpm_j2,
    std::uint8_t position_accel_z, std::uint8_t position_accel_j1,
    std::uint8_t position_accel_j2,
    std::uint16_t bump_speed_rpm_z, std::uint16_t bump_speed_rpm_j1,
    std::uint16_t bump_speed_rpm_j2);

  ~RobotArm();

  RobotArm(const RobotArm &) = delete;
  RobotArm & operator=(const RobotArm &) = delete;

  // Z 碰停与限位 → J2 碰停与限位 → J2 半行程 → J1 碰停与限位（细节见 robot_arm.cpp）
  bool calibrate();

  /** 三轴同时 set_position；整臂标定后 span 须在各类 reachable_span_* 内，否则 false */
  bool set_position(double span_z, double span_joint1, double span_joint2);

  /** `calibrate()` 内由各轴 `set_limits()` 单侧裕量与名义 `span_max` 写入；未标定前为 0 */
  double reachable_span_min_z = 0.0;
  double reachable_span_max_z = 0.0;
  double reachable_span_min_j1 = 0.0;
  double reachable_span_max_j1 = 0.0;
  double reachable_span_min_j2 = 0.0;
  double reachable_span_max_j2 = 0.0;

  ArmJoint & joint_z() noexcept { return *jz_; }
  ArmJoint & joint1() noexcept { return *j1_; }
  ArmJoint & joint2() noexcept { return *j2_; }

private:
  robot_driver::CanInterface & can_;
  std::unique_ptr<robot_driver::Pd42Motor> mz_;
  std::unique_ptr<robot_driver::Pd42Motor> m1_;
  std::unique_ptr<robot_driver::Pd42Motor> m2_;
  std::unique_ptr<ArmJoint> jz_;
  std::unique_ptr<ArmJoint> j1_;
  std::unique_ptr<ArmJoint> j2_;
  std::uint16_t torque_z_up_ma_;
  std::uint16_t torque_z_down_ma_;
  std::uint16_t torque_j1_ma_;
  std::uint16_t torque_j2_ma_;
};

}  // namespace scara_arm

#endif  // SCARA_ARM__ROBOT_ARM_HPP_
