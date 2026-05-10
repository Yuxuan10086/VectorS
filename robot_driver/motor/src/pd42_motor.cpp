#include "robot_driver/pd42_motor.hpp"

#include "robot_driver/can_interface.hpp"
#include "robot_driver/pd42_protocol.hpp"

#include <chrono>
#include <thread>

namespace robot_driver
{

namespace
{
constexpr auto kPd42ReplyWait = std::chrono::milliseconds(100);

constexpr std::uint8_t kErrSendEnqueue = 0xFE;
constexpr std::uint8_t kErrReplyTimeout = 0xFF;
constexpr std::uint8_t kErrReplyMismatch = 0xFC;
constexpr std::uint8_t kErrBadCommandFrame = 0xFB;
}

Pd42Motor::Pd42Motor(CanInterface & can, std::uint8_t motor_id)
: can_(can), motor_id_(motor_id)
{
  if (motor_id_ == 0) {
    return;
  }
  const auto cfg = make_set_send_can_id_frame(motor_id_, default_can_eff_id(motor_id_));
  if (!send_cmd(cfg)) {
    return;
  }
  (void)send_cmd(make_save_parameters_frame(motor_id_));
}

std::optional<std::vector<std::uint8_t>> Pd42Motor::exchange_(
  const std::vector<std::uint8_t> & bytes)
{
  error_code_ = 0;

  if (bytes.size() < 3) {
    error_code_ = kErrBadCommandFrame;
    return std::nullopt;
  }

  if (!can_.send(bytes)) {
    error_code_ = kErrSendEnqueue;
    return std::nullopt;
  }

  const std::uint8_t expect_func = bytes[2];
  const std::uint32_t rx_route = default_can_eff_id(motor_id_);
  const auto deadline =
    std::chrono::steady_clock::now() + kPd42ReplyWait;
  std::vector<std::uint8_t> reply;

  for (;;) {
    if (std::chrono::steady_clock::now() >= deadline) {
      error_code_ = kErrReplyTimeout;
      return std::nullopt;
    }
    if (can_.try_receive(rx_route, reply)) {
      if (reply.size() >= 6U && reply[1] == motor_id_ && reply[2] == expect_func) {
        return reply;
      }
      continue;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
}

bool Pd42Motor::send_cmd(const std::vector<std::uint8_t> & bytes)
{
  auto reply = exchange_(bytes);
  if (!reply) {
    return false;
  }
  if (reply->size() < 6U) {
    error_code_ = kErrReplyMismatch;
    return false;
  }
  if ((*reply)[3] == kAckOk) {
    error_code_ = 0;
    return true;
  }

  error_code_ = (*reply)[3];
  return false;
}

std::optional<int16_t> Pd42Motor::rpm()
{
  auto reply = exchange_(make_read_speed_frame(motor_id_));
  if (!reply) {
    return std::nullopt;
  }
  if (reply->size() < 6U) {
    error_code_ = kErrReplyMismatch;
    return std::nullopt;
  }
  if ((*reply)[3] != kAckOk) {
    error_code_ = (*reply)[3];
    return std::nullopt;
  }
  if (auto v = parse_read_speed_rpm(*reply)) {
    error_code_ = 0;
    return v;
  }
  error_code_ = kErrReplyMismatch;
  return std::nullopt;
}

std::optional<int32_t> Pd42Motor::pos()
{
  auto reply = exchange_(make_read_position_frame(motor_id_));
  if (!reply) {
    return std::nullopt;
  }
  if (reply->size() < 6U) {
    error_code_ = kErrReplyMismatch;
    return std::nullopt;
  }
  if ((*reply)[3] != kAckOk) {
    error_code_ = (*reply)[3];
    return std::nullopt;
  }
  if (auto v = parse_read_position_units(*reply)) {
    error_code_ = 0;
    return v;
  }
  error_code_ = kErrReplyMismatch;
  return std::nullopt;
}

bool Pd42Motor::clear_status()
{
  return send_cmd(make_clear_status_frame(motor_id_));
}

bool Pd42Motor::enable(bool on)
{
  return send_cmd(make_motor_enable_frame(motor_id_, on));
}

bool Pd42Motor::initialize()
{
  if (!clear_status()) {
    return false;
  }
  return enable(true);
}

bool Pd42Motor::set_mode(Pd42CommMode mode)
{
  if (!enable(false)) {
    return false;
  }
  if (!enable(true)) {
    return false;
  }

  return send_cmd(make_set_work_mode_frame(motor_id_, static_cast<std::uint8_t>(mode)));
}

bool Pd42Motor::set_absolute_position(
  std::uint32_t position_units, std::uint16_t speed_rpm, std::uint8_t accel_r_ss,
  bool reverse)
{
  return send_cmd(make_absolute_position_frame(
    motor_id_, reverse, accel_r_ss, speed_rpm, position_units));
}

bool Pd42Motor::stop()
{
  if (!send_cmd(make_emergency_brake_frame(motor_id_))) {
    return false;
  }
  return clear_status();
}

bool Pd42Motor::set_speed(float rpm, bool reverse, std::uint8_t accel_r_ss)
{
  return send_cmd(make_speed_mode_frame(motor_id_, rpm, reverse, accel_r_ss));
}

bool Pd42Motor::set_torque(bool reverse, std::uint16_t current_ma)
{
  return send_cmd(make_torque_mode_frame(motor_id_, reverse, current_ma));
}

bool Pd42Motor::set_limit_origins(std::int32_t left, std::int32_t right)
{
  if (!send_cmd(make_set_left_limit_origin_frame(motor_id_, left))) {
    return false;
  }
  return send_cmd(make_set_right_limit_origin_frame(motor_id_, right));
}

bool Pd42Motor::set_limit_sw(bool enable)
{
  return send_cmd(make_set_limit_switch_frame(motor_id_, enable));
}

bool Pd42Motor::zero_position()
{
  return send_cmd(make_zero_position_frame(motor_id_));
}

}  // namespace robot_driver
