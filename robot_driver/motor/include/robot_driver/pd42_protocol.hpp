// PD42 步进闭环驱动器串口帧协议（以 `doc/电机手册提取.md` 与手册 xlsx 为准）
#ifndef ROBOT_DRIVER__PD42_PROTOCOL_HPP_
#define ROBOT_DRIVER__PD42_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

namespace robot_driver
{

inline constexpr uint8_t kFrameHead = 0xC5;
inline constexpr uint8_t kFrameTail = 0x5C;
inline constexpr std::size_t kCanDataMax = 8;

inline constexpr uint8_t kAckOk = 0x01;

/** 工作模式：通信位置 / 速度 / 力矩（0x62） */
namespace work_mode
{
inline constexpr uint8_t kCommPosition = 0x00;
inline constexpr uint8_t kCommSpeed = 0x01;
inline constexpr uint8_t kCommTorque = 0x02;
}

inline uint8_t checksum_from_parts(
  uint8_t addr, uint8_t func,
  const uint8_t * data, std::size_t data_len)
{
  unsigned sum = kFrameHead + addr + func;
  for (std::size_t i = 0; i < data_len; ++i) {
    sum += data[i];
  }
  return static_cast<uint8_t>(sum & 0xFFU);
}

inline bool verify_frame_checksum(const std::vector<uint8_t> & frame)
{
  if (frame.size() < 5) {
    return false;
  }
  if (frame.front() != kFrameHead || frame.back() != kFrameTail) {
    return false;
  }
  const uint8_t addr = frame[1];
  const uint8_t func = frame[2];
  const std::size_t n = frame.size() - 5U;
  const uint8_t expected = checksum_from_parts(addr, func, n ? &frame[3] : nullptr, n);
  return frame[frame.size() - 2U] == expected;
}

inline bool reply_success(const std::vector<uint8_t> & frame, uint8_t expected_func)
{
  if (frame.size() < 6) {
    return false;
  }
  if (!verify_frame_checksum(frame)) {
    return false;
  }
  if (frame[2] != expected_func) {
    return false;
  }
  return frame[3] == kAckOk;
}

inline void uint32_to_big_endian(uint32_t u, std::array<uint8_t, 4> & out)
{
  out[0] = static_cast<uint8_t>((u >> 24) & 0xFFU);
  out[1] = static_cast<uint8_t>((u >> 16) & 0xFFU);
  out[2] = static_cast<uint8_t>((u >> 8) & 0xFFU);
  out[3] = static_cast<uint8_t>(u & 0xFFU);
}

inline void float_ieee754_to_big_endian_bytes(float v, std::array<uint8_t, 4> & out)
{
  uint32_t u = 0;
  static_assert(sizeof(float) == 4U, "float must be 32-bit");
  std::memcpy(&u, &v, sizeof(float));
  uint32_to_big_endian(u, out);
}

inline void uint16_to_big_endian(uint16_t v, uint8_t out[2])
{
  out[0] = static_cast<uint8_t>((v >> 8) & 0xFFU);
  out[1] = static_cast<uint8_t>(v & 0xFFU);
}

inline int16_t int16_from_big_endian(const uint8_t b[2])
{
  return static_cast<int16_t>(
    (static_cast<uint16_t>(b[0]) << 8) | static_cast<uint16_t>(b[1]));
}

inline int32_t int32_from_big_endian(const uint8_t b[4])
{
  return static_cast<int32_t>(
    (static_cast<uint32_t>(b[0]) << 24) |
    (static_cast<uint32_t>(b[1]) << 16) |
    (static_cast<uint32_t>(b[2]) << 8) |
    static_cast<uint32_t>(b[3]));
}

inline std::vector<uint8_t> build_downlink(
  uint8_t addr, uint8_t func,
  const uint8_t * payload, std::size_t payload_len)
{
  std::vector<uint8_t> out;
  out.reserve(4 + payload_len);
  out.push_back(kFrameHead);
  out.push_back(addr);
  out.push_back(func);
  for (std::size_t i = 0; i < payload_len; ++i) {
    out.push_back(payload[i]);
  }
  out.push_back(checksum_from_parts(addr, func, payload, payload_len));
  out.push_back(kFrameTail);
  return out;
}

inline std::vector<uint8_t> build_downlink(uint8_t addr, uint8_t func)
{
  return build_downlink(addr, func, nullptr, 0);
}

/** 手册 4.2.4：0x04 保存参数（掉电不丢失） */
inline std::vector<uint8_t> make_save_parameters_frame(uint8_t addr)
{
  return build_downlink(addr, 0x04);
}

/** 0x62：mode 见 work_mode（含 kCommTorque） */
inline std::vector<uint8_t> make_set_work_mode_frame(uint8_t addr, uint8_t mode_comm_position_or_speed)
{
  return build_downlink(addr, 0x62, &mode_comm_position_or_speed, 1U);
}

/**
 * 0x6C 设置发送 CAN_ID（《电机手册提取.md》功能码表）。
 * 载荷为扩展帧 ID 的 4 字节大端；本仓库用 `default_can_eff_id(addr)`（上行 **`0000000x`**，x=地址低 4 位）。
 */
inline std::vector<uint8_t> make_set_send_can_id_frame(uint8_t addr, std::uint32_t extended_send_id)
{
  std::array<uint8_t, 4> be{};
  uint32_to_big_endian(extended_send_id, be);
  return build_downlink(addr, 0x6C, be.data(), be.size());
}

/**
 * 0xF2：绝对位置（手册 4.4.3）
 * @param position_units 51200 为一圈
 */
inline std::vector<uint8_t> make_absolute_position_frame(
  uint8_t addr, bool reverse, uint8_t accel_r_ss_0_200,
  std::uint16_t speed_rpm_0_6000, std::uint32_t position_units)
{
  uint8_t a = accel_r_ss_0_200 > 200U ? 200U : accel_r_ss_0_200;
  std::uint16_t sp = speed_rpm_0_6000 > 6000U ? 6000U : speed_rpm_0_6000;
  std::array<uint8_t, 8> pl{};
  pl[0] = reverse ? 1U : 0U;
  pl[1] = a;
  uint16_to_big_endian(sp, &pl[2]);
  pl[4] = static_cast<uint8_t>((position_units >> 24) & 0xFFU);
  pl[5] = static_cast<uint8_t>((position_units >> 16) & 0xFFU);
  pl[6] = static_cast<uint8_t>((position_units >> 8) & 0xFFU);
  pl[7] = static_cast<uint8_t>(position_units & 0xFFU);
  return build_downlink(addr, 0xF2, pl.data(), pl.size());
}

/**
 * 0xF3：相对位置（手册 4.4.4）
 * @param delta_units 51200 为一圈
 */
inline std::vector<uint8_t> make_relative_position_frame(
  uint8_t addr, bool reverse, uint8_t accel_r_ss_0_200,
  std::uint16_t speed_rpm_0_6000, std::uint32_t delta_units)
{
  uint8_t a = accel_r_ss_0_200 > 200U ? 200U : accel_r_ss_0_200;
  std::uint16_t sp = speed_rpm_0_6000 > 6000U ? 6000U : speed_rpm_0_6000;
  std::array<uint8_t, 8> pl{};
  pl[0] = reverse ? 1U : 0U;
  pl[1] = a;
  uint16_to_big_endian(sp, &pl[2]);
  pl[4] = static_cast<uint8_t>((delta_units >> 24) & 0xFFU);
  pl[5] = static_cast<uint8_t>((delta_units >> 16) & 0xFFU);
  pl[6] = static_cast<uint8_t>((delta_units >> 8) & 0xFFU);
  pl[7] = static_cast<uint8_t>(delta_units & 0xFFU);
  return build_downlink(addr, 0xF3, pl.data(), pl.size());
}

/** 0xF1：闭环速度（手册 4.4.2）。rpm==0 时发 IEEE754 0，用作速度清零停机（非 0xFC 急停） */
inline std::vector<uint8_t> make_speed_mode_frame(
  uint8_t addr, float rpm_0p1_to_6000, bool reverse, uint8_t accel_r_ss_0_200)
{
  float s = rpm_0p1_to_6000;
  if (s <= 0.F) {
    s = 0.F;
  } else if (s < 0.1F) {
    s = 0.1F;
  } else if (s > 6000.F) {
    s = 6000.F;
  }
  uint8_t a = accel_r_ss_0_200 > 200U ? 200U : accel_r_ss_0_200;
  std::array<uint8_t, 4> be{};
  float_ieee754_to_big_endian_bytes(s, be);
  std::array<uint8_t, 6> pl{};
  pl[0] = reverse ? 1U : 0U;
  pl[1] = a;
  pl[2] = be[0];
  pl[3] = be[1];
  pl[4] = be[2];
  pl[5] = be[3];
  return build_downlink(addr, 0xF1, pl.data(), pl.size());
}

/** 0xF0：力矩模式控制（手册 4.4.1）；电流 0~3000 mA，大端 uint16 */
inline std::vector<uint8_t> make_torque_mode_frame(
  uint8_t addr, bool reverse, std::uint16_t current_ma_0_3000)
{
  std::uint16_t ma = current_ma_0_3000 > 3000U ? 3000U : current_ma_0_3000;
  std::array<uint8_t, 3> pl{};
  pl[0] = reverse ? 1U : 0U;
  uint16_to_big_endian(ma, &pl[1]);
  return build_downlink(addr, 0xF0, pl.data(), pl.size());
}

/** 0x90：左限位回零零点，int32 位置大端（51200 一圈） */
inline std::vector<uint8_t> make_set_left_limit_origin_frame(uint8_t addr, std::int32_t position_units)
{
  std::array<uint8_t, 4> be{};
  uint32_to_big_endian(static_cast<std::uint32_t>(position_units), be);
  return build_downlink(addr, 0x90, be.data(), be.size());
}

/** 0x98：右限位回零零点 */
inline std::vector<uint8_t> make_set_right_limit_origin_frame(uint8_t addr, std::int32_t position_units)
{
  std::array<uint8_t, 4> be{};
  uint32_to_big_endian(static_cast<std::uint32_t>(position_units), be);
  return build_downlink(addr, 0x98, be.data(), be.size());
}

/** 0x99：左右限位开关；enable=true 开启行程限制 */
inline std::vector<uint8_t> make_set_limit_switch_frame(uint8_t addr, bool enable)
{
  const uint8_t b = enable ? 1U : 0U;
  return build_downlink(addr, 0x99, &b, 1U);
}

/** 0xFC：立即停止（刹车） */
inline std::vector<uint8_t> make_emergency_brake_frame(uint8_t addr)
{
  return build_downlink(addr, 0xFC);
}

/** 0xFA：电机使能控制（手册 4.4.11；Byte1：**0 使能 / 1 失能**） */
inline std::vector<uint8_t> make_motor_enable_frame(uint8_t addr, bool enable)
{
  const uint8_t b = enable ? 0U : 1U;
  return build_downlink(addr, 0xFA, &b, 1U);
}

/** 0xFB：清除状态（堵转、刹车、失能） */
inline std::vector<uint8_t> make_clear_status_frame(uint8_t addr)
{
  return build_downlink(addr, 0xFB);
}

/** 0xF8：当前位置角度清零 */
inline std::vector<uint8_t> make_zero_position_frame(uint8_t addr)
{
  return build_downlink(addr, 0xF8);
}

/** 0x6A：堵转保护；enable=true 开启（手册 4.3.30） */
inline std::vector<uint8_t> make_stall_protection_frame(uint8_t addr, bool enable)
{
  const uint8_t b = enable ? 1U : 0U;
  return build_downlink(addr, 0x6A, &b, 1U);
}

/** 0x6B：堵转电流 mA，范围 0~3000，大端（手册 4.3.31） */
inline std::vector<uint8_t> make_set_stall_current_frame(uint8_t addr, uint16_t current_ma)
{
  uint16_t v = current_ma;
  if (v > 3000U) {
    v = 3000U;
  }
  uint8_t payload[2];
  uint16_to_big_endian(v, payload);
  return build_downlink(addr, 0x6B, payload, 2U);
}

inline std::vector<uint8_t> make_read_position_frame(uint8_t addr)
{
  return build_downlink(addr, 0x2A);
}

inline std::vector<uint8_t> make_read_speed_frame(uint8_t addr)
{
  return build_downlink(addr, 0x29);
}

inline std::vector<uint8_t> make_read_run_state_frame(uint8_t addr)
{
  return build_downlink(addr, 0x2C);
}

inline std::vector<uint8_t> make_read_stall_flag_frame(uint8_t addr)
{
  return build_downlink(addr, 0x2D);
}

/** 手册 4.3.4：读取相电流（mA） */
inline std::vector<uint8_t> make_read_phase_current_frame(uint8_t addr)
{
  return build_downlink(addr, 0x23);
}

/** 手册 4.3.15：读取堵转电流设定（mA） */
inline std::vector<uint8_t> make_read_stall_current_frame(uint8_t addr)
{
  return build_downlink(addr, 0x2E);
}

/** 0x29 应答：Byte2~3 实时转速 int16（RPM），大端 */
inline std::optional<int16_t> parse_read_speed_rpm(const std::vector<uint8_t> & frame)
{
  if (!reply_success(frame, 0x29) || frame.size() < 8) {
    return std::nullopt;
  }
  const uint8_t b[2] = {frame[4], frame[5]};
  return int16_from_big_endian(b);
}

/** 0x2A 应答：Byte2~5 实时位置 int32（51200 一圈），大端 */
inline std::optional<int32_t> parse_read_position_units(const std::vector<uint8_t> & frame)
{
  if (!reply_success(frame, 0x2A) || frame.size() < 10) {
    return std::nullopt;
  }
  const uint8_t b[4] = {frame[4], frame[5], frame[6], frame[7]};
  return int32_from_big_endian(b);
}

/** 0x2C 应答：Byte2 运行状态（0 停止…5 欠压，见手册） */
inline std::optional<uint8_t> parse_read_run_state(const std::vector<uint8_t> & frame)
{
  if (!reply_success(frame, 0x2C) || frame.size() < 7) {
    return std::nullopt;
  }
  return frame[4];
}

/** 0x2D 应答：Byte2 堵转标志 0/1 */
inline std::optional<bool> parse_read_stall_flag(const std::vector<uint8_t> & frame)
{
  if (!reply_success(frame, 0x2D) || frame.size() < 7) {
    return std::nullopt;
  }
  return frame[4] != 0;
}

/** 手册 4.3.15：应答 Byte2~Byte3 为堵转电流 int16（大端），单位 mA */
inline std::optional<int16_t> parse_read_stall_current_ma(const std::vector<uint8_t> & frame)
{
  if (!reply_success(frame, 0x2E) || frame.size() < 8) {
    return std::nullopt;
  }
  const uint8_t b[2] = {frame[4], frame[5]};
  return int16_from_big_endian(b);
}

/** 手册 4.3.4：应答 Byte2~Byte3 为相电流 int16（大端），单位 mA */
inline std::optional<int16_t> parse_read_phase_current_ma(const std::vector<uint8_t> & frame)
{
  if (!reply_success(frame, 0x23) || frame.size() < 8) {
    return std::nullopt;
  }
  const uint8_t b[2] = {frame[4], frame[5]};
  return int16_from_big_endian(b);
}

/**
 * 上行扩展帧 29 位 ID：手册/candump 常写作 **`0000000x`**（末位为十六进制），即 **仅地址低 4 位**，
 * 例如地址 5 → **`0x00000005`**，而非 `0x00001005`（旧写作 **0x1005**）。
 */
inline uint32_t default_can_eff_id(uint8_t slave_addr)
{
  return static_cast<uint32_t>(slave_addr) & 0x0FU;
}

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__PD42_PROTOCOL_HPP_
