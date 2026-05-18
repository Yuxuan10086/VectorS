#include "scara_arm/arm_joint.hpp"

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
constexpr std::int32_t kPulseRepeatTol = 512;  // 与反馈脉冲差在此内视为已到位、不重复下发

constexpr std::chrono::milliseconds kBumpPhaseGraceMs{500};  // bump 启动后此时间内不判相电流到位

std::string joint_fail_msg(
  const char * tag, const char * step, const robot_driver::Pd42Motor & m)
{
  std::ostringstream o;
  o << "ArmJoint " << tag << " " << step << " failed; " << m.error_hint();
  return o.str();
}
}  // namespace

ArmJoint::ArmJoint(
  robot_driver::Pd42Motor & motor, std::int32_t margin_units, double span_max,
  std::uint16_t abs_speed_rpm, std::uint8_t abs_accel, std::uint16_t nominal_stall_ma,
  std::uint16_t bump_speed_rpm, const char * joint_tag)
: joint_tag_(joint_tag),
  motor_(motor),
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
  // initialize/set_mode 后可能清 RAM，再下发名义堵转电流
  if (!motor_.enable_stall_protection(nominal_stall_ma_)) {
    throw std::runtime_error(joint_fail_msg(joint_tag, "enable_stall_protection(nominal)", motor_));
  }
}

bool ArmJoint::set_position(double span)
{
  if (!stop_fwd_ || span_max_ <= 0.0) {
    return false;
  }
  const std::int32_t H = *stop_fwd_;
  if (H <= 2 * margin_) {
    return false;
  }
  if (std::isnan(span) || span < 0.0 || span > span_max_) {
    return false;
  }

  const double Hd = static_cast<double>(H);
  const double span_lo = (static_cast<double>(margin_) / Hd) * span_max_;
  const double span_hi = (static_cast<double>(H - margin_) / Hd) * span_max_;
  if (span < span_lo || span > span_hi) {
    return false;
  }

  const std::int64_t p64 =
    static_cast<std::int64_t>(std::llround((span / span_max_) * Hd));
  if (p64 < static_cast<std::int64_t>(margin_) ||
      p64 > static_cast<std::int64_t>(H) - static_cast<std::int64_t>(margin_))
  {
    return false;
  }
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
    span_now_ = span;
    return true;
  }

  if (!motor_.set_absolute_position(
        static_cast<std::uint32_t>(pulse), abs_speed_rpm_, abs_accel_))
  {
    return false;
  }
  span_now_ = span;
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

  if (!motor_.set_mode(robot_driver::Pd42CommMode::kSpeed)) {
    return false;
  }

  const bool reverse = (dir == 1U);  // 与 set_speed 的 reverse 一致
  if (!stall_seek_by_speed_(motor_, reverse, bump_speed_rpm_, abs_accel_, cal_ma)) {
    (void)motor_.set_speed(0.F, reverse, abs_accel_);
    return false;
  }

  if (dir == 0U) {
    const auto p = motor_.pos();
    if (!p) {
      return false;
    }
    stop_fwd_ = *p;
    return motor_.set_mode(robot_driver::Pd42CommMode::kPosition);
  }

  // dir==1：反向碰停后清零，后续行程为正脉冲区间
  if (!motor_.set_mode(robot_driver::Pd42CommMode::kPosition)) {
    return false;
  }
  if (!motor_.set_zero_position()) {
    return false;
  }
  return true;
}

std::optional<double> ArmJoint::set_limits()
{
  if (!stop_fwd_) {
    return std::nullopt;
  }
  const std::int32_t H = *stop_fwd_;
  if (H <= 2 * margin_) {
    return std::nullopt;
  }

  const std::int32_t lo = margin_;
  const std::int32_t hi = H - margin_;
  const double Hd = static_cast<double>(H);
  const double span_lo = (static_cast<double>(margin_) / Hd) * span_max_;
  const double span_hi = (static_cast<double>(H - margin_) / Hd) * span_max_;

  const auto cur_opt = motor_.pos();
  if (!cur_opt) {
    return std::nullopt;
  }
  const std::int32_t cur = *cur_opt;

  if (cur < lo) {
    if (!set_position(span_lo)) {
      return std::nullopt;
    }
  } else if (cur > hi) {
    if (!set_position(span_hi)) {
      return std::nullopt;
    }
  }

  if (!(motor_.set_limit_origins(lo, hi) && motor_.set_limit_sw(true))) {
    return std::nullopt;
  }

  if (const auto p = motor_.pos()) {
    span_now_ = (static_cast<double>(*p) / Hd) * span_max_;
  }
  print_calibration_summary(std::cout);
  if (!motor_.save_parameters()) {
    return std::nullopt;
  }
  return (static_cast<double>(margin_) / Hd) * span_max_;
}

void ArmJoint::print_calibration_summary(std::ostream & os) const
{
  if (!joint_tag_) {
    return;
  }
  if (!stop_fwd_) {
    os << "[ArmJoint " << joint_tag_ << "] 标定摘要不可用（限位未就绪）\n";
    return;
  }
  const std::int32_t H = *stop_fwd_;
  if (H <= 2 * margin_) {
    os << "[ArmJoint " << joint_tag_ << "] 标定摘要不可用（H<=2*margin）\n";
    return;
  }
  const std::int32_t lo = margin_;
  const std::int32_t hi = H - margin_;
  const std::int64_t pulse_span = static_cast<std::int64_t>(hi) - static_cast<std::int64_t>(lo);
  const double Hd = static_cast<double>(H);
  const double smin = (static_cast<double>(margin_) / Hd) * span_max_;
  const double smax = (static_cast<double>(H - margin_) / Hd) * span_max_;
  os << "[ArmJoint " << joint_tag_ << "] 标定：正向碰停 H=" << H
     << "；驱动限位脉冲 [" << lo << ", " << hi << "]（宽度 " << pulse_span << "）"
     << " 逻辑 span 名义 [0," << span_max_ << "]→脉冲[0," << H << "] 可达 span≈[" << smin << ", " << smax
     << "] span_now=" << span_now_ << '\n';
}

bool ArmJoint::stall_seek_by_speed_(
  robot_driver::Pd42Motor & m, bool reverse, std::uint16_t bump_speed_rpm, std::uint8_t accel,
  std::uint16_t bump_current_preset_ma)
{
  using clock = std::chrono::steady_clock;
  const clock::time_point t_end = clock::now() +
    std::chrono::duration_cast<clock::duration>(
      std::chrono::duration<double>(kBumpStallSeekTimeoutS));

  if (!m.set_speed(static_cast<float>(bump_speed_rpm), reverse, accel)) {
    return false;
  }

  const clock::time_point t_motion_start = clock::now();

  while (clock::now() < t_end) {
    std::this_thread::sleep_for(std::chrono::milliseconds(kBumpStallPollPeriodMs));

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
