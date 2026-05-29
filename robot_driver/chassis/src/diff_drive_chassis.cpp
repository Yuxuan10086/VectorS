#include "chassis/diff_drive_chassis.hpp"

#include <cmath>
#include <thread>

namespace chassis
{

namespace
{
constexpr double kTwistEpsilon = 1e-6; // 速度控制误差容差
constexpr auto kWatchdogPollInterval = std::chrono::milliseconds(50); // 50ms 看门狗轮询间隔

// ===== kMove 模式控制参数（可根据实车调参） =====
constexpr auto kMoveLoopPeriod = std::chrono::milliseconds(20); // 20ms 周期控制

// 直行参数
constexpr double kStraightDecelDistance = 0.30;   // 开始减速的剩余距离 (m)
constexpr double kStraightMinSpeed      = 0.06;   // 最低保底线速度 (m/s)
constexpr double kStraightDecelGain     = 1.6;    // 剩余距离 → 基础速度的比例系数 P1
constexpr double kHeadingCorrectionGain = 1.8;    // 航向误差(rad) → 差速修正(m/s) 的比例系数 P2

// 旋转参数
constexpr double kRotateDecelAngle   = 0.26;      // 开始减速的剩余角度 (rad, ≈15°)
constexpr double kRotateMinOmega     = 0.10;      // 最低保底角速度 (rad/s)
constexpr double kRotateDecelGain    = 3.2;       // 剩余角度 → 角速度的比例系数 P3
constexpr double kRotateAngleTol     = 0.0087;    // 到位容差 (rad, ≈0.5°)

}  // namespace

DiffDriveChassis::DiffDriveChassis(robot_driver::CanInterface & can, DiffDriveParams params)
: can_(can),
  params_(params),
  left_motor_(can, params.left_id),
  right_motor_(can, params.right_id)
{
}

DiffDriveChassis::~DiffDriveChassis()
{
  shutdown_watchdog();

  std::lock_guard<std::mutex> lock(mutex_);
  if (motors_ready_) {
    (void)apply_wheel(left_motor_, 0.0, kLeftForwardRequiresReverse);
    (void)apply_wheel(right_motor_, 0.0, kRightForwardRequiresReverse);
  }
}

bool DiffDriveChassis::apply_wheel(
  robot_driver::Pd42Motor & motor, double v_wheel_ms, bool positive_requires_reverse)
{
  const bool forward = (v_wheel_ms >= 0.0);

  // rpm_from_wheel_ms 直接展开
  float rpm = 0.F;
  const double v = std::abs(v_wheel_ms);
  if (params_.wheel_diameter_m > 0.0) {
    rpm = static_cast<float>(v * 60.0 / (M_PI * params_.wheel_diameter_m));
  }

  const bool reverse = forward ? positive_requires_reverse : !positive_requires_reverse;
  return motor.set_speed(rpm, reverse, params_.accel);
}

bool DiffDriveChassis::send_twist_locked(double linear_x, double omega)
{
  if (!motors_ready_) {
    return false;
  }

  if (std::abs(linear_x - linear_x_) <= kTwistEpsilon &&
    std::abs(omega - omega_) <= kTwistEpsilon) {
    return true;
  }

  const double half = params_.track_m * 0.5;
  const double v_left = linear_x - omega * half;
  const double v_right = linear_x + omega * half;

  if (!apply_wheel(left_motor_, v_left, kLeftForwardRequiresReverse) ||
    !apply_wheel(right_motor_, v_right, kRightForwardRequiresReverse)) {
    return false;
  }

  linear_x_ = linear_x;
  omega_ = omega;
  return true;
}

bool DiffDriveChassis::set_twist(double linear_x, double omega)
{
  // 仅 kTwist 模式允许速度控制
  if (mode_ != DriveMode::kTwist) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  last_cmd_time_ = std::chrono::steady_clock::now();
  return send_twist_locked(linear_x, omega);
}

bool DiffDriveChassis::set_mode(DriveMode mode)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (mode == mode_) {
    return true;
  }

  // 首次或重新初始化电机
  if (!motors_ready_) {
    if (params_.left_id == 0U || params_.right_id == 0U) {
      return false;
    }
    if (params_.wheel_diameter_m <= 0.0 || params_.track_m <= 0.0) {
      return false;
    }

    motors_ready_ = false;
    if (!left_motor_.initialize() || !right_motor_.initialize()) {
      return false;
    }
    if (!left_motor_.set_mode(robot_driver::Pd42CommMode::kSpeed) ||
        !right_motor_.set_mode(robot_driver::Pd42CommMode::kSpeed)) {
      return false;
    }

    motors_ready_ = true;
    linear_x_ = 0.0;
    omega_ = 0.0;
    last_cmd_time_ = std::chrono::steady_clock::now();
  }

  // 根据目标模式启停看门狗
  if (mode == DriveMode::kTwist) {
    if (!watchdog_running_) {
      shutdown_ = false;
      watchdog_thread_ = std::thread(&DiffDriveChassis::watchdog_loop, this);
      watchdog_running_ = true;
    }
  } else {
    // 切换到 move 模式，关闭看门狗
    shutdown_watchdog();
  }

  mode_ = mode;
  return true;
}

void DiffDriveChassis::watchdog_loop()
{
  while (!shutdown_.load(std::memory_order_relaxed)) {
    std::this_thread::sleep_for(kWatchdogPollInterval);

    std::lock_guard<std::mutex> lock(mutex_);
    if (shutdown_.load(std::memory_order_relaxed)) {
      break;
    }
    if (!motors_ready_) {
      continue;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - last_cmd_time_ <= kVelocityCommandTimeout) {
      continue;
    }

    if (std::abs(linear_x_) <= kTwistEpsilon && std::abs(omega_) <= kTwistEpsilon) {
      continue;
    }

    (void)send_twist_locked(0.0, 0.0);
  }
}

void DiffDriveChassis::shutdown_watchdog()
{
  shutdown_.store(true, std::memory_order_relaxed);
  if (watchdog_running_ && watchdog_thread_.joinable()) {
    watchdog_thread_.join();
    watchdog_running_ = false;
  }
}

void DiffDriveChassis::update_imu_yaw(double yaw)
{
  current_yaw_.store(yaw, std::memory_order_relaxed);
}

// ===================== kMove 模式：move / span 实现 =====================

namespace
{
// 内部辅助：脉冲/米（move 距离换算，电机直连车轮，reduction=1）
double pulses_per_meter(const DiffDriveParams & p)
{
  if (p.wheel_diameter_m <= 0.0 || p.encoder_pulses_per_rev <= 0.0) {
    return 0.0;
  }
  const double wheel_circum = M_PI * p.wheel_diameter_m;
  return p.encoder_pulses_per_rev / wheel_circum;
}
} // anonymous

bool DiffDriveChassis::move(double distance_m, double speed_mps)
{
  if (mode_ != DriveMode::kMove || !motors_ready_ || distance_m <= 0.0 || speed_mps <= 0.0) {
    return false;
  }

  const double ppm = pulses_per_meter(params_);
  if (ppm <= 0.0) {
    return false;   // 缺少编码器参数，无法换算
  }

  // 1. 初始化：当前左右轮基准（连续累积距离）
  const auto left_start  = left_motor_.pos().value_or(0);
  const auto right_start = right_motor_.pos().value_or(0);

  // 2. 锁死初始航向（使用此刻最新注入值）
  const double target_yaw = current_yaw_.load(std::memory_order_relaxed);

  double traveled_m = 0.0;

  while (true) {
    // --- 数据采集 ---
    const auto left_now  = left_motor_.pos().value_or(left_start);
    const auto right_now = right_motor_.pos().value_or(right_start);

    const double avg_pulses = (left_now - left_start + right_now - right_start) * 0.5;
    traveled_m = avg_pulses / ppm;

    // --- 终止条件 ---
    if (traveled_m >= distance_m) {
      break;
    }

    // --- 基础前行速度（带减速） ---
    const double remaining = distance_m - traveled_m;
    double base_speed = speed_mps;

    if (remaining <= kStraightDecelDistance) {
      base_speed = std::max(kStraightMinSpeed, kStraightDecelGain * remaining);
    }

    // --- 航向修正（完全依赖外部注入的 current_yaw_） ---
    const double current_yaw = current_yaw_.load(std::memory_order_relaxed);
    const double yaw_err = target_yaw - current_yaw;
    const double correction = kHeadingCorrectionGain * yaw_err;

    // --- 双轮速度合成 ---
    const double v_left  = base_speed - correction;
    const double v_right = base_speed + correction;

    // --- 下发 ---
    (void)apply_wheel(left_motor_,  v_left,  kLeftForwardRequiresReverse);
    (void)apply_wheel(right_motor_, v_right, kRightForwardRequiresReverse);

    // --- 20ms 周期控制 ---
    std::this_thread::sleep_for(kMoveLoopPeriod);
  }

  // 收尾刹车
  (void)apply_wheel(left_motor_,  0.0, kLeftForwardRequiresReverse);
  (void)apply_wheel(right_motor_, 0.0, kRightForwardRequiresReverse);

  return true;
}

bool DiffDriveChassis::span(double angle_rad, double omega_radps)
{
  if (mode_ != DriveMode::kMove || !motors_ready_ || std::abs(angle_rad) < 1e-6 || omega_radps <= 0.0) {
    return false;
  }

  // 1. 读取起始注入角度，计算最短旋转方向和目标
  const double start_yaw = current_yaw_.load(std::memory_order_relaxed);
  // 简单处理：用户传入的就是期望的相对旋转量（正=逆时针）
  const double target_yaw = start_yaw + angle_rad;

  while (true) {
    const double current_yaw = current_yaw_.load(std::memory_order_relaxed);
    double err = target_yaw - current_yaw;

    // 角度误差归一化到 [-π, π]
    while (err >  M_PI) err -= 2.0 * M_PI;
    while (err < -M_PI) err += 2.0 * M_PI;

    const double abs_err = std::abs(err);

    if (abs_err <= kRotateAngleTol) {
      break;   // 到位
    }

    // 旋转速度比例控制（带减速）
    double cmd_omega = (err > 0 ? 1.0 : -1.0) * omega_radps;

    if (abs_err <= kRotateDecelAngle) {
      cmd_omega = (err > 0 ? 1.0 : -1.0) * std::max(kRotateMinOmega, kRotateDecelGain * abs_err);
    }

    // 双轮反向速度
    // 向左（逆时针）: 左轮负，右轮正
    const double v_left  = -cmd_omega * (params_.track_m * 0.5);   // 近似把角速度换算为轮速差
    const double v_right =  cmd_omega * (params_.track_m * 0.5);

    // 由于 apply_wheel 内部会把 m/s 再转 rpm，这里直接把“等效线速度”传进去即可
    // （更精确的做法是把角速度直接映射到 rpm，但为了和现有 apply_wheel 保持一致，用半轴距换算）
    (void)apply_wheel(left_motor_,  v_left,  kLeftForwardRequiresReverse);
    (void)apply_wheel(right_motor_, v_right, kRightForwardRequiresReverse);

    // 20ms 周期
    std::this_thread::sleep_for(kMoveLoopPeriod);
  }

  // 收尾刹车
  (void)apply_wheel(left_motor_,  0.0, kLeftForwardRequiresReverse);
  (void)apply_wheel(right_motor_, 0.0, kRightForwardRequiresReverse);

  return true;
}

}  // namespace chassis
