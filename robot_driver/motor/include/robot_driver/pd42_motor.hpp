// PD42 电机（协议组帧见 pd42_protocol.hpp，CAN 见 CanInterface）
#ifndef ROBOT_DRIVER__PD42_MOTOR_HPP_
#define ROBOT_DRIVER__PD42_MOTOR_HPP_

#include <cstdint>
#include <optional>
#include <vector>

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
  /** 构造完成后按 stall_current_ma 下发堵转电流并开启堵转保护（0~3000 mA） */
  Pd42Motor(CanInterface & can, std::uint8_t motor_id, std::uint16_t stall_current_ma = 2000U);

  bool initialize();
  bool clear_status();
  bool enable(bool on);
  bool set_mode(Pd42CommMode mode);
  bool set_absolute_position(
    std::uint32_t position_units, std::uint16_t speed_rpm, std::uint8_t accel_r_ss,
    bool reverse = false);
  bool stop();
  bool set_speed(float rpm, bool reverse = false, std::uint8_t accel_r_ss = 50);
  bool set_torque(bool reverse, std::uint16_t current_ma);  // reverse：反转；current_ma：0~3000
  bool set_limit_origins(std::int32_t left, std::int32_t right);  // 左右限位原点位置（脉冲/int32）
  bool set_limit_sw(bool enable);  // true：开启左右限位行程限制
  bool set_zero_position();  // 当前位置/角度清零（0xF8）
  /** 保存参数到驱动掉电保持（手册 0x04） */
  bool save_parameters();
  /** 先设堵转电流（0x6B）再开启堵转保护（0x6A）；current_ma 超过 3000 按 3000 处理 */
  bool enable_stall_protection(std::uint16_t current_ma);

  std::optional<int16_t> rpm();
  std::optional<int32_t> pos();
  /** 堵转标志（0x2D）；`true` 为堵转 */
  std::optional<bool> stall_flag();
  /** 堵转电流设定（手册 4.3.15，0x2E）；单位 mA */
  std::optional<std::int16_t> stall_current_setting_ma();
  /** 相电流瞬时值（手册 4.3.4，0x23）；单位 mA */
  std::optional<std::int16_t> phase_current_ma();

  std::uint8_t error_code() const noexcept { return error_code_; }
  std::uint8_t motor_id() const noexcept { return motor_id_; }

  std::optional<Pd42CommMode> comm_mode() const noexcept { return comm_mode_; }

private:
  std::optional<std::vector<std::uint8_t>> exchange_(const std::vector<std::uint8_t> & bytes);
  bool send_cmd(const std::vector<std::uint8_t> & bytes);

  CanInterface & can_;
  std::uint8_t motor_id_;
  std::uint8_t error_code_{0};
  std::optional<Pd42CommMode> comm_mode_{};
};

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__PD42_MOTOR_HPP_
