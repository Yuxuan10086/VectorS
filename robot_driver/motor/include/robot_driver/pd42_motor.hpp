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
  Pd42Motor(CanInterface & can, std::uint8_t motor_id);

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
  bool zero_position();  // 当前位置/角度清零（0xF8）

  std::optional<int16_t> rpm();
  std::optional<int32_t> pos();

  std::uint8_t error_code() const noexcept { return error_code_; }
  std::uint8_t motor_id() const noexcept { return motor_id_; }

private:
  std::optional<std::vector<std::uint8_t>> exchange_(const std::vector<std::uint8_t> & bytes);
  bool send_cmd(const std::vector<std::uint8_t> & bytes);

  CanInterface & can_;
  std::uint8_t motor_id_;
  std::uint8_t error_code_{0};
};

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__PD42_MOTOR_HPP_
