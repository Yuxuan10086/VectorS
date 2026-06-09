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
  left_motor_(can, params.left_id, robot_driver::Pd42Motor::kDisableStallProtection),
  right_motor_(can, params.right_id, robot_driver::Pd42Motor::kDisableStallProtection)
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
  std::unique_lock<std::mutex> lock(mutex_);

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

  // 根据目标模式启停看门狗（注意：join 必须在不持有 mutex 的情况下进行，否则会与 watchdog_loop 死锁）
  if (mode == DriveMode::kTwist) {
    if (!watchdog_running_) {
      shutdown_ = false;
      watchdog_thread_ = std::thread(&DiffDriveChassis::watchdog_loop, this);
      watchdog_running_ = true;
    }
  } else {
    // 切换到 move 模式，安全关闭看门狗（先把线程对象移出，再释放锁，最后 join）
    std::thread thread_to_join;
    if (watchdog_running_ && watchdog_thread_.joinable()) {
      shutdown_.store(true, std::memory_order_relaxed);
      thread_to_join = std::move(watchdog_thread_);
      watchdog_running_ = false;
    }

    lock.unlock();

    if (thread_to_join.joinable()) {
      thread_to_join.join();
    }

    lock.lock();
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
  // 安全版本：短暂持有锁取出线程对象，然后在无锁情况下 join，避免与 watchdog_loop 死锁
  std::thread thread_to_join;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    shutdown_.store(true, std::memory_order_relaxed);
    if (watchdog_running_ && watchdog_thread_.joinable()) {
      thread_to_join = std::move(watchdog_thread_);
      watchdog_running_ = false;
    }
  }
  if (thread_to_join.joinable()) {
    thread_to_join.join();
  }
}

void DiffDriveChassis::update_imu_yaw(double yaw)
{
  current_yaw_.store(yaw, std::memory_order_relaxed);
}

void DiffDriveChassis::cancel_motion()
{
  abort_requested_.store(true, std::memory_order_release);
  if (!motors_ready_) {
    return;
  }
  (void)apply_wheel(left_motor_, 0.0, kLeftForwardRequiresReverse);
  (void)apply_wheel(right_motor_, 0.0, kRightForwardRequiresReverse);
}

bool DiffDriveChassis::motion_aborted() const
{
  return motion_aborted_.exchange(false, std::memory_order_acq_rel);
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

/** 0x2A 有车体坐标系下有符号位移（米）；符号与 kXForwardRequiresReverse 一致。 */
double signed_travel_m(
  std::int32_t left_start, std::int32_t right_start,
  std::int32_t left_now, std::int32_t right_now, double ppm)
{
  const auto dl_raw = static_cast<std::int64_t>(left_now) - static_cast<std::int64_t>(left_start);
  const auto dr_raw = static_cast<std::int64_t>(right_now) - static_cast<std::int64_t>(right_start);
  const auto dl = kLeftForwardRequiresReverse ? -dl_raw : dl_raw;
  const auto dr = kRightForwardRequiresReverse ? -dr_raw : dr_raw;
  return static_cast<double>(dl + dr) * 0.5 / ppm;
}
} // anonymous

bool DiffDriveChassis::move(double distance_m, double speed_mps, MoveProgressCallback on_progress)
{
  if (mode_ != DriveMode::kMove || !motors_ready_ || std::abs(distance_m) < 1e-6 || speed_mps <= 0.0) {
    return false;
  }

  const double ppm = pulses_per_meter(params_);
  if (ppm <= 0.0) {
    return false;   // 缺少编码器参数，无法换算
  }

  const double dir = (distance_m > 0.0) ? 1.0 : -1.0;
  const double target_m = std::abs(distance_m);

  abort_requested_.store(false, std::memory_order_release);
  motion_aborted_.store(false, std::memory_order_release);

  // 1. 初始化：记录有符号基准位置（0x2A，后退时读数减小、可为负）
  const auto left_start_opt = left_motor_.pos();
  const auto right_start_opt = right_motor_.pos();
  if (!left_start_opt || !right_start_opt) {
    return false;
  }
  const std::int32_t left_start = *left_start_opt;
  const std::int32_t right_start = *right_start_opt;

  // 2. 锁死初始航向（使用此刻最新注入值）
  const double target_yaw = current_yaw_.load(std::memory_order_relaxed);

  while (true) {
    if (abort_requested_.load(std::memory_order_acquire)) {
      motion_aborted_.store(true, std::memory_order_release);
      (void)apply_wheel(left_motor_, 0.0, kLeftForwardRequiresReverse);
      (void)apply_wheel(right_motor_, 0.0, kRightForwardRequiresReverse);
      return false;
    }

    // --- 数据采集 ---
    const auto left_now_opt = left_motor_.pos();
    const auto right_now_opt = right_motor_.pos();
    if (!left_now_opt || !right_now_opt) {
      (void)apply_wheel(left_motor_, 0.0, kLeftForwardRequiresReverse);
      (void)apply_wheel(right_motor_, 0.0, kRightForwardRequiresReverse);
      if (abort_requested_.load(std::memory_order_acquire)) {
        motion_aborted_.store(true, std::memory_order_release);
        return false;
      }
      std::this_thread::sleep_for(kMoveLoopPeriod);
      continue;
    }

    // traveled_raw：车体前进为正、后退为负（左右轮 0x2A 已在 signed_travel_m 中对齐）
    const double traveled_raw_m = signed_travel_m(
      left_start, right_start, *left_now_opt, *right_now_opt, ppm)
      * params_.odometry_correction_factor;
    // progress：沿命令方向的已走距离，恒为非负直至到达 target
    const double progress_m = dir * traveled_raw_m;

    if (progress_m >= target_m) {
      break;
    }

    // --- 基础速度（带减速）；speed_mps 为速率幅值，dir 决定前进/后退 ---
    const double remaining = target_m - progress_m;

    if (on_progress) {
      on_progress(MoveProgress{
        remaining,
        *left_now_opt,
        *right_now_opt,
      });
    }
    double base_speed = dir * speed_mps;

    if (remaining <= kStraightDecelDistance) {
      base_speed = dir * std::max(kStraightMinSpeed, kStraightDecelGain * remaining);
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

  abort_requested_.store(false, std::memory_order_release);
  motion_aborted_.store(false, std::memory_order_release);

  // 1. 读取起始注入角度，计算最短旋转方向和目标
  const double start_yaw = current_yaw_.load(std::memory_order_relaxed);
  // 简单处理：用户传入的就是期望的相对旋转量（正=逆时针）
  const double target_yaw = start_yaw + angle_rad;

  while (true) {
    if (abort_requested_.load(std::memory_order_acquire)) {
      motion_aborted_.store(true, std::memory_order_release);
      (void)apply_wheel(left_motor_, 0.0, kLeftForwardRequiresReverse);
      (void)apply_wheel(right_motor_, 0.0, kRightForwardRequiresReverse);
      return false;
    }

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

// ===================== 电机错误诊断（供上层 open() 失败时打印详细原因） =====================

std::uint8_t DiffDriveChassis::left_motor_error_code() const
{
  return left_motor_.error_code();
}

std::string DiffDriveChassis::left_motor_error_hint() const
{
  return left_motor_.error_hint();
}

std::uint8_t DiffDriveChassis::right_motor_error_code() const
{
  return right_motor_.error_code();
}

std::string DiffDriveChassis::right_motor_error_hint() const
{
  return right_motor_.error_hint();
}

}  // namespace chassis
