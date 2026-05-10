#include "arm_collision_calib/arm_joint.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <thread>

namespace arm_collision_calib
{

namespace
{
constexpr std::uint16_t kTorqueMaMax = 3000U;

constexpr std::uint16_t kRetraceSpeedRpm = 400U;
constexpr std::uint8_t kRetraceAccel = 80U;
constexpr std::int32_t kRetracePosTol = 256;
constexpr double kRetraceTimeoutS = 90.0;
constexpr int kRetracePollMs = 40;

/** 已在 [left,right] 内返回 nullopt；否则返回离当前位置最近的端点 */
std::optional<std::int32_t> nearest_endpoint_if_outside(
  std::int32_t cur, std::int32_t left, std::int32_t right)
{
  if (cur >= left && cur <= right) {
    return std::nullopt;
  }
  if (cur < left) {
    return left;
  }
  return right;
}

bool wait_near_target(
  robot_driver::Pd42Motor & m, std::int32_t target, std::int32_t tol, double timeout_s)
{
  using clock = std::chrono::steady_clock;
  const clock::time_point t_end = clock::now() +
    std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(timeout_s));
  while (clock::now() < t_end) {
    const auto p = m.pos();
    if (!p) {
      return false;
    }
    if (std::abs(static_cast<std::int64_t>(*p) - static_cast<std::int64_t>(target)) <= tol) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kRetracePollMs));
  }
  return false;
}
}  // namespace

ArmJoint::ArmJoint(robot_driver::Pd42Motor & motor, BumpCfg bump, std::int32_t margin_units)
: motor_(motor), bump_(bump), margin_(margin_units)
{
  if (!motor_.initialize()) {
    throw std::runtime_error("ArmJoint: initialize failed");
  }
  if (!motor_.set_limit_sw(false)) {
    throw std::runtime_error("ArmJoint: set_limit_sw(false) failed");
  }
  if (!motor_.zero_position()) {
    throw std::runtime_error("ArmJoint: zero_position failed");
  }
  if (!motor_.set_torque(false, 0)) {
    throw std::runtime_error("ArmJoint: idle torque failed");
  }
}

bool ArmJoint::idle()
{
  return motor_.set_torque(false, 0);
}

bool ArmJoint::bump(std::uint8_t dir, std::uint16_t torque_ma)
{
  if (dir > 1U) {
    return false;
  }
  if (torque_ma > kTorqueMaMax) {
    torque_ma = kTorqueMaMax;
  }

  if (!motor_.set_mode(robot_driver::Pd42CommMode::kTorque)) {
    return false;
  }

  const bool reverse = (dir == 0U);
  if (!torque_seek_(motor_, reverse, torque_ma, bump_)) {
    return false;
  }

  const auto p = motor_.pos();
  if (!p) {
    return false;
  }
  if (dir == 1U) {
    stop_fwd_ = *p;
  } else {
    stop_rev_ = *p;
  }
  return true;
}

bool ArmJoint::set_limits()
{
  if (!stop_fwd_ || !stop_rev_) {
    return false;
  }
  const std::int32_t lo = std::min(*stop_fwd_, *stop_rev_);
  const std::int32_t hi = std::max(*stop_fwd_, *stop_rev_);
  const std::int32_t left = lo + margin_;
  const std::int32_t right = hi - margin_;
  if (left >= right) {
    return false;
  }

  const auto cur_opt = motor_.pos();
  if (!cur_opt) {
    return false;
  }
  const std::int32_t cur = *cur_opt;

  if (const auto ep = nearest_endpoint_if_outside(cur, left, right)) {
    const std::int32_t target = *ep;
    if (!motor_.set_mode(robot_driver::Pd42CommMode::kPosition)) {
      return false;
    }
    const bool reverse = target < cur;
    if (!motor_.set_absolute_position(
          static_cast<std::uint32_t>(target), kRetraceSpeedRpm, kRetraceAccel, reverse))
    {
      return false;
    }
    if (!wait_near_target(motor_, target, kRetracePosTol, kRetraceTimeoutS)) {
      return false;
    }
  }

  return motor_.set_limit_origins(left, right) && motor_.set_limit_sw(true);
}

bool ArmJoint::torque_seek_(
  robot_driver::Pd42Motor & m, bool reverse, std::uint16_t torque_ma, const BumpCfg & cfg)
{
  using clock = std::chrono::steady_clock;
  const clock::time_point t_end = clock::now() +
    std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(cfg.timeout_s));
  clock::time_point stable_begin{};
  bool stable = false;

  while (clock::now() < t_end) {
    if (!m.set_torque(reverse, torque_ma)) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(cfg.poll_ms));
    const auto rv = m.rpm();
    if (!rv) {
      continue;
    }
    const double rpm = static_cast<double>(*rv);
    if (std::abs(rpm) <= cfg.rpm_zero_th) {
      const clock::time_point now = clock::now();
      if (!stable) {
        stable = true;
        stable_begin = now;
      } else if (now - stable_begin >= std::chrono::milliseconds(cfg.stable_ms)) {
        (void)m.set_torque(reverse, 0);
        (void)m.clear_status();
        return true;
      }
    } else {
      stable = false;
    }
  }
  (void)m.set_torque(reverse, 0);
  return false;
}

}  // namespace arm_collision_calib
