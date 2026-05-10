// 三关节机械臂标定（Z 预留，标定流程仅用 J1/J2）
#ifndef ARM_COLLISION_CALIB__ROBOT_ARM_HPP_
#define ARM_COLLISION_CALIB__ROBOT_ARM_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "arm_collision_calib/arm_joint.hpp"
#include "robot_driver/can_interface.hpp"
#include "robot_driver/pd42_motor.hpp"

namespace arm_collision_calib
{

class RobotArm
{
public:
  RobotArm(
    std::string can_iface,
    std::uint8_t id_z, std::uint8_t id_j1, std::uint8_t id_j2,
    BumpCfg bump, std::int32_t margin_units,
    std::uint16_t torque_j1_ma, std::uint16_t torque_j2_ma);

  ~RobotArm();

  RobotArm(const RobotArm &) = delete;
  RobotArm & operator=(const RobotArm &) = delete;

  /**
   * J1 正向 bump → J2 正/反向 bump → J2 set_limits → J1 反向 bump → J1 set_limits
   * （不使用 Z 轴碰停）
   */
  bool calibrate();

  ArmJoint & joint_z() noexcept { return *jz_; }
  ArmJoint & joint1() noexcept { return *j1_; }
  ArmJoint & joint2() noexcept { return *j2_; }

private:
  bool idle_all_joints_();

  std::string can_iface_;
  std::unique_ptr<robot_driver::CanInterface> can_;
  std::unique_ptr<robot_driver::Pd42Motor> mz_;
  std::unique_ptr<robot_driver::Pd42Motor> m1_;
  std::unique_ptr<robot_driver::Pd42Motor> m2_;
  std::unique_ptr<ArmJoint> jz_;
  std::unique_ptr<ArmJoint> j1_;
  std::unique_ptr<ArmJoint> j2_;
  std::uint16_t torque_j1_ma_;
  std::uint16_t torque_j2_ma_;
};

}  // namespace arm_collision_calib

#endif  // ARM_COLLISION_CALIB__ROBOT_ARM_HPP_
