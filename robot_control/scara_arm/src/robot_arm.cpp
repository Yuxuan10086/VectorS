#include "scara_arm/robot_arm.hpp"

#include <iostream>
#include <stdexcept>

namespace scara_arm
{

namespace
{
void print_calib_fail(const char * step, const robot_driver::Pd42Motor & m)
{
  std::cerr << "calibrate failed: " << step << " (motor_id=" << static_cast<int>(m.motor_id())
            << ", err=" << static_cast<int>(m.error_code()) << ")\n";
}
}  // namespace

RobotArm::RobotArm(
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
  std::uint16_t bump_speed_rpm_j2)
: can_iface_(std::move(can_iface)),
  torque_z_ma_(torque_z_ma),
  torque_j1_ma_(torque_j1_ma),
  torque_j2_ma_(torque_j2_ma)
{
  can_ = std::make_unique<robot_driver::CanInterface>(can_iface_);
  if (!can_->open()) {
    throw std::runtime_error("RobotArm: CAN open failed");
  }

  mz_ = std::make_unique<robot_driver::Pd42Motor>(*can_, id_z, stall_current_z_ma);
  jz_ = std::make_unique<ArmJoint>(
    *mz_, bump, margin_units, span_z, position_speed_rpm_z, position_accel_z, stall_current_z_ma,
    bump_speed_rpm_z, "Z");

  m1_ = std::make_unique<robot_driver::Pd42Motor>(*can_, id_j1, stall_current_j1_ma);
  m2_ = std::make_unique<robot_driver::Pd42Motor>(*can_, id_j2, stall_current_j2_ma);
  j1_ = std::make_unique<ArmJoint>(
    *m1_, bump, margin_units, span_joint1, position_speed_rpm_j1, position_accel_j1,
    stall_current_j1_ma, bump_speed_rpm_j1, "J1");
  j2_ = std::make_unique<ArmJoint>(
    *m2_, bump, margin_units, span_joint2, position_speed_rpm_j2, position_accel_j2,
    stall_current_j2_ma, bump_speed_rpm_j2, "J2");
}

RobotArm::~RobotArm() = default;

bool RobotArm::calibrate()
{
  // if (!jz_->bump(0, torque_z_ma_)) {
  //   print_calib_fail("Z bump forward", jz_->motor());
  //   return false;
  // }
  // if (!jz_->bump(1, torque_z_ma_)) {
  //   print_calib_fail("Z bump reverse", jz_->motor());
  //   return false;
  // }
  // if (!jz_->set_limits()) {
  //   print_calib_fail("Z set_limits", jz_->motor());
  //   return false;
  // }
  if (!j1_->bump(1, torque_j1_ma_)) {
    print_calib_fail("J1 bump reverse", j1_->motor());
    return false;
  }
  if (!j2_->bump(1, torque_j2_ma_)) {
    print_calib_fail("J2 bump reverse", j2_->motor());
    return false;
  }
  if (!j2_->bump(0, torque_j2_ma_)) {
    print_calib_fail("J2 bump forward", j2_->motor());
    return false;
  }
  if (!j2_->set_limits()) {
    print_calib_fail("J2 set_limits", j2_->motor());
    return false;
  }

  (void)j2_->set_position(j2_->span_max() / 2.0);

  if (!j1_->bump(0, torque_j1_ma_)) {
    print_calib_fail("J1 bump forward", j1_->motor());
    return false;
  }
  if (!j1_->set_limits()) {
    print_calib_fail("J1 set_limits", j1_->motor());
    return false;
  }
  return true;
}

}  // namespace scara_arm
