#include "arm_collision_calib/robot_arm.hpp"

#include <stdexcept>

namespace arm_collision_calib
{

RobotArm::RobotArm(
  std::string can_iface,
  std::uint8_t id_z, std::uint8_t id_j1, std::uint8_t id_j2,
  BumpCfg bump, std::int32_t margin_units,
  std::uint16_t torque_j1_ma, std::uint16_t torque_j2_ma)
: can_iface_(std::move(can_iface)),
  torque_j1_ma_(torque_j1_ma),
  torque_j2_ma_(torque_j2_ma)
{
  can_ = std::make_unique<robot_driver::CanInterface>(can_iface_);
  if (!can_->open()) {
    throw std::runtime_error("RobotArm: CAN open failed");
  }

  mz_ = std::make_unique<robot_driver::Pd42Motor>(*can_, id_z);
  m1_ = std::make_unique<robot_driver::Pd42Motor>(*can_, id_j1);
  m2_ = std::make_unique<robot_driver::Pd42Motor>(*can_, id_j2);

  jz_ = std::make_unique<ArmJoint>(*mz_, bump, margin_units);
  j1_ = std::make_unique<ArmJoint>(*m1_, bump, margin_units);
  j2_ = std::make_unique<ArmJoint>(*m2_, bump, margin_units);
}

RobotArm::~RobotArm() = default;

bool RobotArm::idle_all_joints_()
{
  return jz_->idle() && j1_->idle() && j2_->idle();
}

bool RobotArm::calibrate()
{
  if (!j1_->bump(1, torque_j1_ma_)) {
    return false;
  }
  if (!idle_all_joints_()) {
    return false;
  }
  if (!j2_->bump(1, torque_j2_ma_)) {
    return false;
  }
  if (!idle_all_joints_()) {
    return false;
  }
  if (!j2_->bump(0, torque_j2_ma_)) {
    return false;
  }
  if (!j2_->set_limits()) {
    return false;
  }
  if (!idle_all_joints_()) {
    return false;
  }
  if (!j1_->bump(0, torque_j1_ma_)) {
    return false;
  }
  if (!j1_->set_limits()) {
    return false;
  }
  (void)idle_all_joints_();
  return true;
}

}  // namespace arm_collision_calib
