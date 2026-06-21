#include "robot_driver/pd42_motor.hpp"

#include "robot_driver/can_interface.hpp"
#include "robot_driver/pd42_protocol.hpp"

#include <chrono>
#include <sstream>
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
// 非通信位置模式下调用 set_absolute_position（须先 set_mode(kPosition)）
constexpr std::uint8_t kErrWrongCommMode = 0xFA;
}  // namespace

Pd42Motor::Pd42Motor(CanInterface & can, std::uint8_t motor_id, std::uint16_t stall_current_ma)
: can_(can), motor_id_(motor_id), stall_current_ma_(stall_current_ma)
{}

std::optional<std::vector<std::uint8_t>> Pd42Motor::send_cmd(const std::vector<std::uint8_t> & bytes)
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
  const auto deadline = std::chrono::steady_clock::now() + kPd42ReplyWait;
  std::vector<std::uint8_t> reply;

  for (;;) {
    if (std::chrono::steady_clock::now() >= deadline) {
      error_code_ = kErrReplyTimeout;
      return std::nullopt;
    }
    if (can_.try_receive(rx_route, reply)) {
      if (reply.size() < 6U || reply[1] != motor_id_ || reply[2] != expect_func) {
        continue;
      }
      // 头尾与校验和不对则丢弃，继续等同功能码的下一帧（避免误读 byte3）
      if (!verify_frame_checksum(reply)) {
        continue;
      }
      if (reply[3] != kAckOk) {
        error_code_ = reply[3];
        return std::nullopt;
      }
      error_code_ = 0;
      return reply;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
}

std::optional<Pd42SystemParameters> Pd42Motor::read_system_parameters()
{
  auto reply = send_cmd(make_read_system_parameters_frame(motor_id_));
  if (!reply) {
    return std::nullopt;
  }
  if (auto params = parse_read_system_parameters(*reply)) {
    return params;
  }
  error_code_ = kErrReplyMismatch;
  return std::nullopt;
}

bool Pd42Motor::set_speed_loop_pid(std::uint32_t p, std::uint32_t i, std::uint32_t d)
{
  return send_cmd(make_set_speed_loop_pid_frame(motor_id_, p, i, d)).has_value();
}

bool Pd42Motor::set_position_loop_pid(std::uint32_t p, std::uint32_t i, std::uint32_t d)
{
  return send_cmd(make_set_position_loop_pid_frame(motor_id_, p, i, d)).has_value();
}

std::optional<Pd42SpeedLoopPid> Pd42Motor::read_speed_loop_pid()
{
  auto reply = send_cmd(make_read_speed_loop_pid_frame(motor_id_));
  if (!reply) {
    return std::nullopt;
  }
  if (auto v = parse_read_speed_loop_pid(*reply)) {
    return v;
  }
  error_code_ = kErrReplyMismatch;
  return std::nullopt;
}

std::optional<Pd42PositionLoopPid> Pd42Motor::read_position_loop_pid()
{
  auto reply = send_cmd(make_read_position_loop_pid_frame(motor_id_));
  if (!reply) {
    return std::nullopt;
  }
  if (auto v = parse_read_position_loop_pid(*reply)) {
    return v;
  }
  error_code_ = kErrReplyMismatch;
  return std::nullopt;
}

std::optional<int16_t> Pd42Motor::rpm()
{
  auto reply = send_cmd(make_read_speed_frame(motor_id_));
  if (!reply) {
    return std::nullopt;
  }
  if (auto v = parse_read_speed_rpm(*reply)) {
    return v;
  }
  error_code_ = kErrReplyMismatch;
  return std::nullopt;
}

std::optional<int32_t> Pd42Motor::pos()
{
  auto reply = send_cmd(make_read_position_frame(motor_id_));
  if (!reply) {
    return std::nullopt;
  }
  if (auto v = parse_read_position_units(*reply)) {
    return v;
  }
  error_code_ = kErrReplyMismatch;
  return std::nullopt;
}

std::optional<bool> Pd42Motor::stall_flag()
{
  auto reply = send_cmd(make_read_stall_flag_frame(motor_id_));
  if (!reply) {
    return std::nullopt;
  }
  if (auto v = parse_read_stall_flag(*reply)) {
    return v;
  }
  error_code_ = kErrReplyMismatch;
  return std::nullopt;
}

std::optional<bool> Pd42Motor::arrived_flag()
{
  auto reply = send_cmd(make_read_arrived_flag_frame(motor_id_));
  if (!reply) {
    return std::nullopt;
  }
  if (auto v = parse_read_arrived_flag(*reply)) {
    return v;
  }
  error_code_ = kErrReplyMismatch;
  return std::nullopt;
}

std::optional<std::int16_t> Pd42Motor::stall_current_setting_ma()
{
  auto reply = send_cmd(make_read_stall_current_frame(motor_id_));
  if (!reply) {
    return std::nullopt;
  }
  if (auto v = parse_read_stall_current_ma(*reply)) {
    return v;
  }
  error_code_ = kErrReplyMismatch;
  return std::nullopt;
}

std::optional<std::int16_t> Pd42Motor::phase_current_ma()
{
  auto reply = send_cmd(make_read_phase_current_frame(motor_id_));
  if (!reply) {
    return std::nullopt;
  }
  if (auto v = parse_read_phase_current_ma(*reply)) {
    return v;
  }
  error_code_ = kErrReplyMismatch;
  return std::nullopt;
}

bool Pd42Motor::clear_status()
{
  return send_cmd(make_clear_status_frame(motor_id_)).has_value();
}

bool Pd42Motor::enable(bool on)
{
  return send_cmd(make_motor_enable_frame(motor_id_, on)).has_value();
}

bool Pd42Motor::initialize()
{
  if (motor_id_ == 0U) {
    error_code_ = kErrBadCommandFrame;
    return false;
  }

  const auto cfg = make_set_send_can_id_frame(motor_id_, default_can_eff_id(motor_id_));
  if (!send_cmd(cfg)) {
    return false;
  }

  if (stall_current_ma_ != kDisableStallProtection) {
    if (!enable_stall_protection(stall_current_ma_)) {
      return false;
    }
  }

  if (!save_parameters()) {
    return false;
  }

  if (!clear_status()) {
    return false;
  }
  return enable(true);
}

bool Pd42Motor::set_mode(Pd42CommMode mode)
{
  if (!send_cmd(make_set_work_mode_frame(motor_id_, static_cast<std::uint8_t>(mode)))) {
    return false;
  }
  comm_mode_ = mode;
  return true;
}

bool Pd42Motor::set_absolute_position(
  std::uint32_t position_units, std::uint16_t speed_rpm, std::uint8_t accel_r_ss,
  bool reverse)
{
  if (!comm_mode_.has_value() || comm_mode_.value() != Pd42CommMode::kPosition) {
    error_code_ = kErrWrongCommMode;
    return false;
  }
  return send_cmd(make_absolute_position_frame(
           motor_id_, reverse, accel_r_ss, speed_rpm, position_units))
    .has_value();
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
  if (!comm_mode_.has_value() || comm_mode_.value() != Pd42CommMode::kSpeed) {
    error_code_ = kErrWrongCommMode;
    return false;
  }
  return send_cmd(make_speed_mode_frame(motor_id_, rpm, reverse, accel_r_ss)).has_value();
}

bool Pd42Motor::set_torque(bool reverse, std::uint16_t current_ma)
{
  if (!comm_mode_.has_value() || comm_mode_.value() != Pd42CommMode::kTorque) {
    error_code_ = kErrWrongCommMode;
    return false;
  }
  return send_cmd(make_torque_mode_frame(motor_id_, reverse, current_ma)).has_value();
}

bool Pd42Motor::set_limit_origins(std::int32_t left, std::int32_t right)
{
  if (!send_cmd(make_set_left_limit_origin_frame(motor_id_, left))) {
    return false;
  }
  return send_cmd(make_set_right_limit_origin_frame(motor_id_, right)).has_value();
}

bool Pd42Motor::set_limit_sw(bool enable)
{
  return send_cmd(make_set_limit_switch_frame(motor_id_, enable)).has_value();
}

bool Pd42Motor::set_zero_position()
{
  return send_cmd(make_zero_position_frame(motor_id_)).has_value();
}

bool Pd42Motor::save_parameters()
{
  if (motor_id_ == 0U) {
    return false;
  }
  return send_cmd(make_save_parameters_frame(motor_id_)).has_value();
}

bool Pd42Motor::enable_stall_protection(std::uint16_t current_ma)
{
  if (!send_cmd(make_set_stall_current_frame(motor_id_, current_ma))) {
    return false;
  }
  return send_cmd(make_stall_protection_frame(motor_id_, true)).has_value();
}

std::string Pd42Motor::error_hint() const
{
  const char * text = nullptr;
  switch (error_code_) {
    case 0:
      text = "正常";
      break;
    case 0x01:
      text = "应答成功";
      break;
    case 0xE1:
      text = "帧长度不足";
      break;
    case 0xE2:
      text = "帧头错误（非0xC5）";
      break;
    case 0xE3:
      text = "帧尾错误（非0x5C）";
      break;
    case 0xE4:
      text = "校验和错误";
      break;
    case 0xE5:
      text = "不支持的功能码";
      break;
    case 0xE6:
      text = "数据不合法";
      break;
    case kErrWrongCommMode:
      text = "comm_mode 与指令不符（须先 set_mode）";
      break;
    case kErrReplyTimeout:
      text = "等待应答超时";
      break;
    case kErrSendEnqueue:
      text = "发送入队失败";
      break;
    case kErrReplyMismatch:
      text = "应答与指令不匹配、校验失败或解析失败";
      break;
    case kErrBadCommandFrame:
      text = "下发帧过短（本机检查）";
      break;
    default:
      text = "驱动应答错误字节（见手册应答码）";
      break;
  }

  std::ostringstream oss;
  oss << "motor_id=" << static_cast<int>(motor_id_) << " error_code=0x" << std::hex << std::uppercase
      << static_cast<unsigned>(error_code_) << std::dec << ' ' << text;
  return oss.str();
}

std::string Pd42Motor::status_string() const
{
  if (error_code_ == 0U) {
    return "正常";
  }
  return error_hint();
}

}  // namespace robot_driver
