#include "scara_arm/robot_arm.hpp"

#include "robot_driver/can_interface.hpp"
#include "robot_driver/pd42_motor.hpp"

#include <iostream>
#include <stdexcept>

namespace scara_arm
{

namespace
{
void print_calib_fail(const char * step, const robot_driver::Pd42Motor & m)
{
  std::cerr << "calibrate failed: " << step << " (motor_id=" << static_cast<int>(m.motor_id())
            << ", err=" << static_cast<int>(m.error_code()) << ")";
  if (m.error_code() == 0) {
    std::cerr << " — 无驱动应答错误：多为碰停寻边超时（超时内相电流未超标定阈值）、"
                 "bump_speed_rpm_z=0、或读相电流连续失败；见 arm_joint.cpp stall_seek_by_speed_\n";
  } else {
    std::cerr << '\n' << m.error_hint() << '\n';
  }
}

ArmJointParams with_joint_tag(ArmJointParams p, const char * tag)
{
  if (p.joint_tag == nullptr) {
    p.joint_tag = tag;
  }
  return p;
}
}  // namespace

RobotArm::RobotArm(robot_driver::CanInterface & can, ScaraArmParams params)
: can_(can),
  params_(params),
  torque_z_up_ma_(params.torque_z_up_ma),
  torque_z_down_ma_(params.torque_z_down_ma),
  torque_j1_ma_(params.torque_j1_ma),
  torque_j2_ma_(params.torque_j2_ma)
{
  if (params_.z.motor_id == 0U || params_.j1.motor_id == 0U || params_.j2.motor_id == 0U) {
    throw std::invalid_argument("motor_z_id / motor_j1_id / motor_j2_id 均不可为 0");
  }

  const ArmJointParams z = with_joint_tag(params_.z, "Z");
  const ArmJointParams j1 = with_joint_tag(params_.j1, "J1");
  const ArmJointParams j2 = with_joint_tag(params_.j2, "J2");

  mz_ = std::make_unique<robot_driver::Pd42Motor>(can_, z.motor_id, z.stall_current_ma);
  jz_ = std::make_unique<ArmJoint>(*mz_, z);

  m1_ = std::make_unique<robot_driver::Pd42Motor>(can_, j1.motor_id, j1.stall_current_ma);
  j1_ = std::make_unique<ArmJoint>(*m1_, j1);

  m2_ = std::make_unique<robot_driver::Pd42Motor>(can_, j2.motor_id, j2.stall_current_ma);
  j2_ = std::make_unique<ArmJoint>(*m2_, j2);
}

RobotArm::~RobotArm() = default;

bool RobotArm::calibrate(std::function<void(const std::string & axis)> on_axis_complete)
{
  auto notify = [&](const char * axis) {
    if (on_axis_complete) {
      on_axis_complete(std::string(axis));
    }
  };

  if (!jz_->bump(0, torque_z_up_ma_)) {
    print_calib_fail("Z bump forward", jz_->motor());
    return false;
  }
  if (!jz_->bump(1, torque_z_down_ma_)) {
    print_calib_fail("Z bump reverse", jz_->motor());
    return false;
  }
  if (const auto mz = jz_->set_limits()) {
    reachable_span_min_z = *mz;
    reachable_span_max_z = jz_->span_max() - *mz;
  } else {
    print_calib_fail("Z set_limits", jz_->motor());
    return false;
  }
  if (!jz_->set_position(jz_->span_max() / 2.0)) {
    print_calib_fail("Z set_position", jz_->motor());
    return false;
  }
  notify("z");   // Z 轴标定完成

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
  if (const auto m2 = j2_->set_limits()) {
    reachable_span_min_j2 = *m2;
    reachable_span_max_j2 = j2_->span_max() - *m2;
  } else {
    print_calib_fail("J2 set_limits", j2_->motor());
    return false;
  }

  (void)j2_->set_position(j2_->span_max() / 2.0);
  notify("j2");  // J2 轴标定完成

  if (!j1_->bump(0, torque_j1_ma_)) {
    print_calib_fail("J1 bump forward", j1_->motor());
    return false;
  }
  if (const auto m1 = j1_->set_limits()) {
    reachable_span_min_j1 = *m1;
    reachable_span_max_j1 = j1_->span_max() - *m1;
  } else {
    print_calib_fail("J1 set_limits", j1_->motor());
    return false;
  }
  j1_->set_position(j1_->span_max() / 2.0);
  notify("j1");  // J1 轴标定完成

  return true;
}

bool RobotArm::set_position(double span_z, double span_joint1, double span_joint2)
{
  if (reachable_span_max_z > reachable_span_min_z) {
    if (span_z < reachable_span_min_z || span_z > reachable_span_max_z) {
      return false;
    }
  }
  if (reachable_span_max_j1 > reachable_span_min_j1) {
    if (span_joint1 < reachable_span_min_j1 || span_joint1 > reachable_span_max_j1) {
      return false;
    }
  }
  if (reachable_span_max_j2 > reachable_span_min_j2) {
    if (span_joint2 < reachable_span_min_j2 || span_joint2 > reachable_span_max_j2) {
      return false;
    }
  }
  return jz_->set_position(span_z) && j1_->set_position(span_joint1) && j2_->set_position(span_joint2);
}

}  // namespace scara_arm
