// PD42 电机（协议组帧见 pd42_protocol.hpp，CAN 见 CanInterface）
#ifndef ROBOT_DRIVER__PD42_MOTOR_HPP_
#define ROBOT_DRIVER__PD42_MOTOR_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "robot_driver/pd42_protocol.hpp"

namespace robot_driver
{

class CanInterface;

enum class Pd42CommMode : std::uint8_t
{
  kPosition = 0x00,
  kSpeed = 0x01,
  kTorque = 0x02,
};

class Pd42Motor
{
public:
  // 构造：仅保存 CAN 引用与电机 ID；所有 CAN 指令在 initialize() 中下发。
  Pd42Motor(CanInterface & can, std::uint8_t motor_id, std::uint16_t stall_current_ma = kDisableStallProtection);

  static constexpr std::uint16_t kDisableStallProtection = 10000U;

  bool initialize();
  bool clear_status();
  bool enable(bool on);
  bool set_mode(Pd42CommMode mode);
  bool set_absolute_position(
    std::uint32_t position_units, std::uint16_t speed_rpm, std::uint8_t accel_r_ss,
    bool reverse = false);
  bool stop();
  bool set_speed(float rpm, bool reverse = false, std::uint8_t accel_r_ss = 50);
  bool set_torque(bool reverse, std::uint16_t current_ma);
  bool set_limit_origins(std::int32_t left, std::int32_t right);
  bool set_limit_sw(bool enable);
  bool set_zero_position();
  bool save_parameters();
  bool enable_stall_protection(std::uint16_t stall_current_ma);
  bool set_speed_loop_pid(std::uint32_t p, std::uint32_t i, std::uint32_t d);
  /** 手册 4.3.18：读取系统参数（0x31），每次调用即发 CAN，无内部缓存 */
  std::optional<Pd42SystemParameters> read_system_parameters();
  /** 手册 4.3.7：读取速度环 PID（0x26） */
  std::optional<Pd42SpeedLoopPid> read_speed_loop_pid();

  std::optional<int16_t> rpm();
  std::optional<int32_t> pos();
  std::optional<bool> stall_flag();
  std::optional<std::int16_t> stall_current_setting_ma();
  std::optional<std::int16_t> phase_current_ma();

  std::uint8_t error_code() const noexcept { return error_code_; }
  std::uint8_t motor_id() const noexcept { return motor_id_; }
  std::string error_hint() const;
  std::string status_string() const;

  std::optional<Pd42CommMode> comm_mode() const noexcept { return comm_mode_; }

private:
  std::optional<std::vector<std::uint8_t>> send_cmd(const std::vector<std::uint8_t> & bytes);

  CanInterface & can_;
  std::uint8_t motor_id_;
  std::uint16_t stall_current_ma_{kDisableStallProtection};
  std::uint8_t error_code_{0};
  std::optional<Pd42CommMode> comm_mode_{};
};

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__PD42_MOTOR_HPP_
