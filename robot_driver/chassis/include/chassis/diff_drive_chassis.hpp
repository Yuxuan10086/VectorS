#ifndef CHASSIS__DIFF_DRIVE_CHASSIS_HPP_
#define CHASSIS__DIFF_DRIVE_CHASSIS_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>

#include "robot_driver/can_interface.hpp"
#include "robot_driver/pd42_motor.hpp"

namespace chassis
{

inline constexpr std::chrono::milliseconds kVelocityCommandTimeout{300}; //超过该时间未调用 set_twist，看门狗自动置零 

// 左右轮正方向对应的 reverse 标志（硬件接线决定，true 表示正速需传 reverse=true）
inline constexpr bool kLeftForwardRequiresReverse = true;   
inline constexpr bool kRightForwardRequiresReverse = false;

struct DiffDriveParams
{
  std::uint8_t left_id{};
  std::uint8_t right_id{};
  double wheel_diameter_m{0.0};
  double track_m{0.0};
  std::uint8_t accel{0};

  // 用于 move 的距离-脉冲换算（kMove 模式必需，电机直连车轮，无减速器）
  double encoder_pulses_per_rev{0.0};   // 编码器每转脉冲数（512000）
};

enum class DriveMode {
  kTwist,  // 速度控制（set_twist 有效，带 300ms 看门狗）
  kMove    // 阻塞移动（move/span 有效，无看门狗）
};

class DiffDriveChassis
{
public:
  DiffDriveChassis(robot_driver::CanInterface & can, DiffDriveParams params);
  ~DiffDriveChassis();

  DiffDriveChassis(const DiffDriveChassis &) = delete;
  DiffDriveChassis & operator=(const DiffDriveChassis &) = delete;

  bool set_mode(DriveMode mode);                    // kTwist 带看门狗，kMove 无
  bool set_twist(double linear_x, double omega);    // 仅 kTwist 模式有效

  /** 阻塞式直线运动（仅 kMove 模式有效）。内部 20ms 控制循环依赖外部注入的 Yaw。 */
  bool move(double distance_m, double speed_mps);

  /** 阻塞式原地旋转（仅 kMove 模式有效）。完全由注入的 Yaw 驱动。 */
  bool span(double angle_rad, double omega_radps);

  /** 由外部（ROS 线程）高频调用，注入最新的 IMU 航向角（弧度，连续累加或 -π~π 均可）。
   *  move / span 的内部循环会直接读取该值用于航向闭环。
   */
  void update_imu_yaw(double yaw);

  robot_driver::CanInterface & can() { return can_; }
  const DiffDriveParams & params() const { return params_; }
  DriveMode mode() const { return mode_; }

private:
  bool apply_wheel(robot_driver::Pd42Motor & motor, double v_wheel_ms, bool positive_requires_reverse);
  bool send_twist_locked(double linear_x, double omega);
  void watchdog_loop();
  void shutdown_watchdog();  // 私有辅助，供析构和 set_mode 调用

  robot_driver::CanInterface & can_;
  DiffDriveParams params_;
  robot_driver::Pd42Motor left_motor_;
  robot_driver::Pd42Motor right_motor_;
  bool motors_ready_{false};

  DriveMode mode_{DriveMode::kMove};   // 构造函数默认 move 模式

  double linear_x_{0.0};
  double omega_{0.0};
  std::chrono::steady_clock::time_point last_cmd_time_{};

  // kMove 模式下由外部异步注入的最新 IMU 航向（弧度）
  std::atomic<double> current_yaw_{0.0};

  std::mutex mutex_;
  std::thread watchdog_thread_;
  std::atomic<bool> shutdown_{false};
  bool watchdog_running_{false};
};

}  // namespace chassis

#endif  // CHASSIS__DIFF_DRIVE_CHASSIS_HPP_
