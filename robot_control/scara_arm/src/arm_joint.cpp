#include "scara_arm/arm_joint.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace scara_arm
{

namespace
{
/** 判定「目标脉冲已达到」的窗口（脉冲） */
constexpr std::int32_t kPulseRepeatTol = 512;

/** bump：启动后此时间内不判相电流到位（ms） */
constexpr std::chrono::milliseconds kBumpPhaseGraceMs{500};
}  // namespace

namespace
{
std::string joint_fail_msg(
  const char * tag, const char * step, const robot_driver::Pd42Motor & m)
{
  std::ostringstream o;
  o << "ArmJoint " << tag << " " << step << " failed"
    << " motor_id=" << static_cast<int>(m.motor_id())
    << " err=" << static_cast<int>(m.error_code());
  const auto e = m.error_code();
  if (e == 0xFF) {
    o << " (timeout/no reply: check CAN name, bitrate, wiring, slave id)";
  } else if (e == 0xFE) {
    o << " (send queue)";
  } else if (e >= 0xE1 && e <= 0xE6) {
    o << " (manual err 0xE1-0xE6: frame/protocol reject)";
  } else if (e == 0xFA) {
    o << " (comm_mode 与指令不符：须先 set_mode 到位置/速度/力矩对应模式)";
  }
  return o.str();
}
}  // namespace

ArmJoint::ArmJoint(
  robot_driver::Pd42Motor & motor, BumpCfg bump, std::int32_t margin_units, double span_max,
  std::uint16_t abs_speed_rpm, std::uint8_t abs_accel, std::uint16_t nominal_stall_ma,
  std::uint16_t bump_speed_rpm, const char * joint_tag)
: joint_tag_(joint_tag),
  motor_(motor),
  bump_(bump),
  margin_(margin_units),
  nominal_stall_ma_(nominal_stall_ma),
  bump_speed_rpm_(bump_speed_rpm),
  abs_speed_rpm_(abs_speed_rpm),
  abs_accel_(abs_accel),
  span_max_(span_max),
  span_now_(0.0)
{
  if (!motor_.initialize()) {
    throw std::runtime_error(joint_fail_msg(joint_tag, "initialize", motor_));
  }
  if (!motor_.set_limit_sw(false)) {
    throw std::runtime_error(joint_fail_msg(joint_tag, "set_limit_sw(false)", motor_));
  }
  if (!motor_.set_mode(robot_driver::Pd42CommMode::kPosition)) {
    throw std::runtime_error(joint_fail_msg(joint_tag, "set_mode(position)", motor_));
  }
  /** 标定 bump：反向到位后再 set_zero_position，此处保留上电原角度 */
  /** Pd42Motor  ctor 已设堵转，但此后 initialize/set_mode 等可能清 RAM；再下发名义堵转电流 */
  if (!motor_.enable_stall_protection(nominal_stall_ma_)) {
    throw std::runtime_error(joint_fail_msg(joint_tag, "enable_stall_protection(nominal)", motor_));
  }
  /** 默认保持通信位置模式；标定 bump() 内临时速度模式 */
}

bool ArmJoint::set_position(double span)
{
  last_set_position_outcome_.reset();
  if (!limit_lo_ || !limit_hi_ || span_max_ <= 0.0) {
    return false;
  }
  if (*limit_lo_ >= *limit_hi_) {
    return false;
  }
  if (std::isnan(span) || span < 0.0 || span > span_max_) {
    return false;
  }
  const double s = span;
  const double lo_d = static_cast<double>(*limit_lo_);
  const std::int64_t p64 = static_cast<std::int64_t>(
    std::llround(lo_d + (s / span_max_) * (static_cast<double>(*limit_hi_) - lo_d)));
  if (p64 < static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::min()) ||
      p64 > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()))
  {
    return false;
  }
  const std::int32_t pulse = static_cast<std::int32_t>(p64);

  const auto cur_opt = motor_.pos();
  if (!cur_opt) {
    return false;
  }
  const std::int32_t cur = *cur_opt;
  const std::int64_t pd =
    static_cast<std::int64_t>(pulse) - static_cast<std::int64_t>(cur);
  if (pd <= static_cast<std::int64_t>(kPulseRepeatTol) &&
      pd >= -static_cast<std::int64_t>(kPulseRepeatTol))
  {
    span_now_ = s;
    last_set_position_outcome_ = SetPositionOutcome{pulse, false};
    return true;
  }

  if (!motor_.set_absolute_position(
        static_cast<std::uint32_t>(pulse), abs_speed_rpm_, abs_accel_))
  {
    return false;
  }
  span_now_ = s;
  last_set_position_outcome_ = SetPositionOutcome{pulse, true};
  return true;
}

bool ArmJoint::bump(std::uint8_t dir, std::uint16_t calibration_stall_ma)
{
  if (dir > 1U) {
    return false;
  }
  if (bump_speed_rpm_ == 0U) {
    return false;
  }

  std::uint16_t cal_ma = calibration_stall_ma;
  if (cal_ma > 3000U) {
    cal_ma = 3000U;
  }

  auto restore_nominal = [this]() {
    (void)motor_.enable_stall_protection(nominal_stall_ma_);
    (void)motor_.set_mode(robot_driver::Pd42CommMode::kPosition);
  };

  if (!motor_.set_mode(robot_driver::Pd42CommMode::kSpeed)) {
    restore_nominal();
    return false;
  }

  /** dir：0=正向 / 1=反向；与 set_speed 的 reverse 一致（reverse=true 为反转） */
  const bool reverse = (dir == 1U);
  if (!stall_seek_by_speed_(motor_, reverse, bump_speed_rpm_, abs_accel_, bump_, cal_ma)) {
    (void)motor_.set_speed(0.F, reverse, abs_accel_);
    restore_nominal();
    return false;
  }

  if (dir == 0U) {
    const auto p = motor_.pos();
    if (!p) {
      restore_nominal();
      return false;
    }
    stop_fwd_ = *p;
    return motor_.set_mode(robot_driver::Pd42CommMode::kPosition);
  }

  /** 反向碰停 (dir=1)：此处定义零点，后续正向行程为正脉冲区间（避免依赖负绝对位置） */
  if (!motor_.set_mode(robot_driver::Pd42CommMode::kPosition)) {
    restore_nominal();
    return false;
  }
  if (!motor_.set_zero_position()) {
    restore_nominal();
    return false;
  }
  return true;
}

bool ArmJoint::set_limits()
{
  if (!stop_fwd_) {
    return false;
  }
  limit_lo_ = margin_;
  limit_hi_ = *stop_fwd_;
  if (*limit_lo_ >= *limit_hi_) {
    limit_lo_.reset();
    limit_hi_.reset();
    return false;
  }

  const auto cur_opt = motor_.pos();
  if (!cur_opt) {
    limit_lo_.reset();
    limit_hi_.reset();
    return false;
  }
  const std::int32_t cur = *cur_opt;

  if (cur < *limit_lo_ || cur > *limit_hi_) {
    const double span_target = (cur < *limit_lo_) ? 0.0 : span_max_;
    if (!set_position(span_target)) {
      limit_lo_.reset();
      limit_hi_.reset();
      return false;
    }
  }

  if (!(motor_.set_limit_origins(*limit_lo_, *limit_hi_) && motor_.set_limit_sw(true))) {
    limit_lo_.reset();
    limit_hi_.reset();
    return false;
  }

  if (const auto p = motor_.pos()) {
    const double dr =
      static_cast<double>(*limit_hi_) - static_cast<double>(*limit_lo_);
    if (dr > 0.0) {
      span_now_ =
        ((static_cast<double>(*p) - static_cast<double>(*limit_lo_)) / dr) * span_max_;
    }
  }
  print_calibration_summary(std::cout);
  if (!motor_.save_parameters()) {
    return false;
  }
  return true;
}

void ArmJoint::print_calibration_summary(std::ostream & os) const
{
  if (!joint_tag_) {
    return;
  }
  if (!stop_fwd_ || !limit_lo_ || !limit_hi_) {
    os << "[ArmJoint " << joint_tag_ << "] 标定摘要不可用（限位未就绪）\n";
    return;
  }
  const std::int32_t lo = *limit_lo_;
  const std::int32_t hi = *limit_hi_;
  const std::int64_t pulse_span = static_cast<std::int64_t>(hi) - static_cast<std::int64_t>(lo);
  os << "[ArmJoint " << joint_tag_ << "] 标定：正向碰停脉冲=" << *stop_fwd_
     << "；margin(下限)=" << margin_ << " → 限位脉冲 [" << lo << ", " << hi << "]（宽度 " << pulse_span
     << "） span_max=" << span_max_ << " span_now=" << span_now_ << '\n';
}

bool ArmJoint::stall_seek_by_speed_(
  robot_driver::Pd42Motor & m, bool reverse, std::uint16_t bump_speed_rpm, std::uint8_t accel,
  const BumpCfg & cfg, std::uint16_t bump_current_preset_ma)
{
  using clock = std::chrono::steady_clock;
  const clock::time_point t_end = clock::now() +
    std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(cfg.timeout_s));

  if (!m.set_speed(static_cast<float>(bump_speed_rpm), reverse, accel)) {
    return false;
  }

  const clock::time_point t_motion_start = clock::now();

  while (clock::now() < t_end) {
    std::this_thread::sleep_for(std::chrono::milliseconds(cfg.poll_ms));

    const clock::time_point now = clock::now();
    if (now - t_motion_start < kBumpPhaseGraceMs) {
      continue;
    }

    const auto phase_ma = m.phase_current_ma();
    if (!phase_ma.has_value()) {
      continue;
    }

    const std::int32_t preset =
      static_cast<std::int32_t>(bump_current_preset_ma);
    const std::int32_t iabs =
      std::abs(static_cast<std::int32_t>(*phase_ma));

    if (iabs > preset) {
      (void)m.set_speed(0.F, reverse, accel);
      (void)m.clear_status();
      return true;
    }
  }
  (void)m.set_speed(0.F, reverse, accel);
  return false;
}

}  // namespace scara_arm
