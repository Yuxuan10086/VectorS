// 单关节：碰停采样 + 限位下发（无 ROS 依赖）
#ifndef SCARA_ARM__ARM_JOINT_HPP_
#define SCARA_ARM__ARM_JOINT_HPP_

#include <cstdint>
#include <iosfwd>
#include <optional>

#include "robot_driver/pd42_motor.hpp"

namespace scara_arm
{

/** 单关节配置：与 ArmJoint 构造入参同形；motor_id 由 RobotArm 建 Pd42Motor 使用 */
struct ArmJointParams
{
  std::uint8_t motor_id{};
  std::int32_t limit_margin_units{};
  double span_max{};
  std::uint16_t position_speed_rpm{};
  std::uint8_t position_accel{};
  std::uint16_t stall_current_ma{};
  std::uint16_t bump_speed_rpm{};
  const char * joint_tag{nullptr};
};

// bump 相电流轮询周期（ms）、碰停寻边总超时（s），stall_seek_by_speed_ 使用
inline constexpr int kBumpStallPollPeriodMs = 30;
inline constexpr double kBumpStallSeekTimeoutS = 25.0;

// 持有一个 Pd42Motor：span 与限位脉冲线性映射；bump/set_limits/set_position 见 cpp
class ArmJoint
{
public:
  ArmJoint(robot_driver::Pd42Motor & motor, const ArmJointParams & params);

  // dir：0 正向 / 1 反向；calibration_stall_ma：碰停相电流阈值 mA；启动后 500ms 内不判到位
  bool bump(std::uint8_t dir, std::uint16_t calibration_stall_ma);

  // 须先完成正向 bump（stop_fwd_）。成功返回单侧 span 裕量 (margin/H)*span_max；对称收缩
  std::optional<double> set_limits();

  void print_calibration_summary(std::ostream & os) const;

  bool set_position(double span);

  double span_max() const noexcept { return span_max_; }
  double span_now() const noexcept { return span_now_; }
  /** YAML `limit_margin_units`，与 set_limits 下发一致 */
  std::int32_t limit_margin_units() const noexcept { return margin_; }
  /** 正向碰停脉冲 H；未正向 bump 时为 nullopt */
  std::optional<std::int32_t> forward_stop_pulse() const noexcept { return stop_fwd_; }

  robot_driver::Pd42Motor & motor() noexcept { return motor_; }

private:
  static bool stall_seek_by_speed_(
    robot_driver::Pd42Motor & m, bool reverse, std::uint16_t bump_speed_rpm, std::uint8_t accel,
    std::uint16_t bump_current_preset_ma);

  const char * joint_tag_{};
  robot_driver::Pd42Motor & motor_;
  std::int32_t margin_;
  std::uint16_t nominal_stall_ma_;
  std::uint16_t bump_speed_rpm_;
  std::uint16_t abs_speed_rpm_;
  std::uint8_t abs_accel_;
  double span_max_;
  double span_now_;
  /** 正向碰停脉冲 H；逻辑 span∈[0,span_max] 对应脉冲 [0,H]，驱动限位为 [margin, H-margin] */
  std::optional<std::int32_t> stop_fwd_;
};

}  // namespace scara_arm

#endif  // SCARA_ARM__ARM_JOINT_HPP_
