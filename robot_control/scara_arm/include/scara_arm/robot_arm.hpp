// SCARA 机械臂（三关节 Z / J1 / J2）
#ifndef SCARA_ARM__ROBOT_ARM_HPP_
#define SCARA_ARM__ROBOT_ARM_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "scara_arm/arm_joint.hpp"
#include "robot_driver/can_interface.hpp"
#include "robot_driver/pd42_motor.hpp"

namespace scara_arm
{

class RobotArm
{
public:
  /** Z / J1 / J2 从机地址均须非 0 */
  RobotArm(
    std::string can_iface,
    std::uint8_t id_z, std::uint8_t id_j1, std::uint8_t id_j2,
    BumpCfg bump, std::int32_t margin_units,
    double span_z, double span_joint1, double span_joint2,
    std::uint16_t torque_z_ma, std::uint16_t torque_j1_ma, std::uint16_t torque_j2_ma,
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

  /**
   * J2：反向 bump（清零）→ 正向 bump → set_limits；J2 span 半行程 →
   * J1：反向 bump（清零）→ 正向 bump → set_limits
   */
  bool calibrate();

  ArmJoint & joint_z() noexcept { return *jz_; }
  ArmJoint & joint1() noexcept { return *j1_; }
  ArmJoint & joint2() noexcept { return *j2_; }

private:
  std::string can_iface_;
  std::unique_ptr<robot_driver::CanInterface> can_;
  std::unique_ptr<robot_driver::Pd42Motor> mz_;
  std::unique_ptr<robot_driver::Pd42Motor> m1_;
  std::unique_ptr<robot_driver::Pd42Motor> m2_;
  std::unique_ptr<ArmJoint> jz_;
  std::unique_ptr<ArmJoint> j1_;
  std::unique_ptr<ArmJoint> j2_;
  std::uint16_t torque_z_ma_;
  std::uint16_t torque_j1_ma_;
  std::uint16_t torque_j2_ma_;
};

}  // namespace scara_arm

#endif  // SCARA_ARM__ROBOT_ARM_HPP_
