// 单关节：碰停采样 + 限位下发（无 ROS 依赖）
#ifndef SCARA_ARM__ARM_JOINT_HPP_
#define SCARA_ARM__ARM_JOINT_HPP_

#include <cstdint>
#include <iosfwd>
#include <optional>

#include "robot_driver/pd42_motor.hpp"

namespace scara_arm
{

/** 碰停判据 */
struct BumpCfg
{
  double rpm_zero_th{15.0};
  int stable_ms{150};
  int poll_ms{30};
  double timeout_s{15.0};
};

/** `set_position` 成功返回后，由 `last_set_position_outcome()` 读取 */
struct SetPositionOutcome
{
  std::int32_t target_pulse{};
  /** false：与反馈差在容差内，未再次下发绝对位置帧 */
  bool sent_to_motor{false};
};

/**
 * 单关节：持有一个 Pd42Motor。
 * span_max：用户侧最大幅度（0…X，物理含义自定）；左限位脉冲 → 0、右限位 → span_max 线性映射。
 * 构造时：initialize → 关限位 → 通信位置模式 → enable_stall_protection（nominal）；**不在此清零角度**。
 * bump()：速度模式；相电流判到位（见 cpp）。**正向 (dir=0)**：到位后仅记录 **stop_fwd_**。**反向 (dir=1)**：到位后 **set_zero_position**（该端为零点）。
 * set_limits()：**下限脉冲 = margin**，**上限脉冲 = stop_fwd_**（正向碰停点）；再 set_limit_origins 与开限位。
 * set_position(span)：依赖 limit_lo_/limit_hi_；映射 pulse ∈ [margin, stop_fwd] ↔ span ∈ [0, span_max]。
 */
class ArmJoint
{
public:
  /**
   * joint_tag：日志用短标；span_max：用户单位最大行程，用于与脉冲限位区间线性对应。
   * abs_speed_rpm / abs_accel：通信位置模式下 **`set_absolute_position`** 用的转速（rpm）与加速度档（0~255）。
   * nominal_stall_ma：与电机构造 **stall_current** 一致；仅构造 / bump 失败 **`restore`** 时再下发堵转参数。
   * bump_speed_rpm：碰停标定 **`set_speed`** 目标转速（rpm）。
   */
  ArmJoint(
    robot_driver::Pd42Motor & motor, BumpCfg bump, std::int32_t margin_units, double span_max,
    std::uint16_t abs_speed_rpm, std::uint8_t abs_accel, std::uint16_t nominal_stall_ma,
    std::uint16_t bump_speed_rpm, const char * joint_tag);

  /** dir：0 正向 / 1 反向；calibration_stall_ma：碰停相电流阈值（mA，0~3000），|I|>阈值即判到位（启动后 500ms 内不判） */
  bool bump(std::uint8_t dir, std::uint16_t calibration_stall_ma);

  /** 需已完成正向 bump（stop_fwd_）；将驱动限位原点设为 [margin, 正向碰停脉冲] */
  bool set_limits();

  /** `set_limits()` 成功后打印碰停点、限位脉冲与 span（可由调用方再次传入流以复查） */
  void print_calibration_summary(std::ostream & os) const;

  /**
   * 将 span 映射到 [limit_lo_,limit_hi_] 脉冲并发绝对位置（须已 set_limits）。
   * 仅当 span∈[0,span_max_] 时成功；越界不夹紧、直接 false。
   * 目标脉冲与反馈相差在容差内则不重复下发（见 cpp 内脉冲容差常量）。
   */
  bool set_position(double span);

  /** 最近一次成功的 `set_position` 映射结果；失败或未调用则为空 */
  std::optional<SetPositionOutcome> last_set_position_outcome() const noexcept
  {
    return last_set_position_outcome_;
  }

  double span_max() const noexcept { return span_max_; }
  double span_now() const noexcept { return span_now_; }

  robot_driver::Pd42Motor & motor() noexcept { return motor_; }

private:
  static bool stall_seek_by_speed_(
    robot_driver::Pd42Motor & m, bool reverse, std::uint16_t bump_speed_rpm, std::uint8_t accel,
    const BumpCfg & cfg, std::uint16_t bump_current_preset_ma);

  const char * joint_tag_{};
  robot_driver::Pd42Motor & motor_;
  BumpCfg bump_;
  std::int32_t margin_;
  std::uint16_t nominal_stall_ma_;
  std::uint16_t bump_speed_rpm_;
  std::uint16_t abs_speed_rpm_;
  std::uint8_t abs_accel_;
  double span_max_;
  double span_now_;
  std::optional<std::int32_t> limit_lo_;
  std::optional<std::int32_t> limit_hi_;
  /** 正向碰停时读到的脉冲（反向 bump 已清零后，作为行程上端） */
  std::optional<std::int32_t> stop_fwd_;

  std::optional<SetPositionOutcome> last_set_position_outcome_;
};

}  // namespace scara_arm

#endif  // SCARA_ARM__ARM_JOINT_HPP_
