// 单关节：碰停采样 + 限位下发（无 ROS 依赖）
#ifndef ARM_COLLISION_CALIB__ARM_JOINT_HPP_
#define ARM_COLLISION_CALIB__ARM_JOINT_HPP_

#include <cstdint>
#include <optional>

#include "robot_driver/pd42_motor.hpp"

namespace arm_collision_calib
{

/** 碰停判据 */
struct BumpCfg
{
  double rpm_zero_th{15.0};
  int stable_ms{150};
  int poll_ms{30};
  double timeout_s{120.0};
};

/**
 * 构造时：initialize → 关限位 → zero_position → 力矩清零。
 * idle()：力矩置零。
 * bump(dir)：力矩模式 → 恒力矩碰停并记录位置。
 * set_limits()：两侧碰停经 margin 收缩；若当前位置在区间外则先到较近端点，再写入驱动并开限位。
 */
class ArmJoint
{
public:
  ArmJoint(robot_driver::Pd42Motor & motor, BumpCfg bump, std::int32_t margin_units);

  bool idle();

  /** dir：0 反向 / 1 正向；torque_ma：0~3000 */
  bool bump(std::uint8_t dir, std::uint16_t torque_ma);

  /** 需两侧均已 bump 成功，否则失败 */
  bool set_limits();

  robot_driver::Pd42Motor & motor() noexcept { return motor_; }

private:
  static bool torque_seek_(
    robot_driver::Pd42Motor & m, bool reverse, std::uint16_t torque_ma, const BumpCfg & cfg);

  robot_driver::Pd42Motor & motor_;
  BumpCfg bump_;
  std::int32_t margin_;
  std::optional<std::int32_t> stop_fwd_;
  std::optional<std::int32_t> stop_rev_;
};

}  // namespace arm_collision_calib

#endif  // ARM_COLLISION_CALIB__ARM_JOINT_HPP_
