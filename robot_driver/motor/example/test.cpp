// 交互式调用 Pd42Motor；指令名为缩短形式，例如 init() / spd(200) / rpm()
// 用法: pd42_cycle_example [can0] [电机ID]
#include "robot_driver/can_interface.hpp"
#include "robot_driver/pd42_motor.hpp"
#include "robot_driver/pd42_protocol.hpp"

#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

void trim_in_place(std::string & s)
{
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) {
    s.erase(s.begin());
  }
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) {
    s.pop_back();
  }
}

std::vector<std::string> split_args(const std::string & inside)
{
  std::string t = inside;
  trim_in_place(t);
  if (t.empty()) {
    return {};
  }
  std::vector<std::string> out;
  std::string cur;
  for (char c : t) {
    if (c == ',') {
      trim_in_place(cur);
      out.push_back(cur);
      cur.clear();
    } else {
      cur += c;
    }
  }
  trim_in_place(cur);
  out.push_back(cur);
  return out;
}

/** 解析一行 `name(arg,...)`；失败返回 false */
bool parse_call(const std::string & line, std::string & name, std::vector<std::string> & args)
{
  std::string s = line;
  trim_in_place(s);
  if (s.empty()) {
    return false;
  }
  const std::size_t lp = s.find('(');
  if (lp == std::string::npos) {
    return false;
  }
  name = s.substr(0, lp);
  trim_in_place(name);
  if (s.back() != ')') {
    return false;
  }
  const std::size_t rp = s.rfind(')');
  if (rp <= lp + 1) {
    args = {};
    return !name.empty();
  }
  args = split_args(s.substr(lp + 1, rp - lp - 1));
  return !name.empty();
}

unsigned long parse_uint(const std::string & tok)
{
  char * end = nullptr;
  const unsigned long v = std::strtoul(tok.c_str(), &end, 0);
  if (!end || *end != '\0') {
    throw std::runtime_error("非法整数: " + tok);
  }
  return v;
}

float parse_float(const std::string & tok)
{
  char * end = nullptr;
  const float v = std::strtof(tok.c_str(), &end);
  if (!end || *end != '\0') {
    throw std::runtime_error("非法浮点: " + tok);
  }
  return v;
}

bool parse_bool(const std::string & tok)
{
  std::string t = tok;
  trim_in_place(t);
  for (char & c : t) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  if (t == "true" || t == "1") {
    return true;
  }
  if (t == "false" || t == "0") {
    return false;
  }
  throw std::runtime_error("非法布尔，使用 true/false 或 1/0: " + tok);
}

const char * error_code_hint(std::uint8_t code)
{
  switch (code) {
    case 0xFF:
      return " — SDK：100ms 内未收到应答（检查接线/从站地址/回包 CAN ID；非手册 E1～E6）";
    case 0xFE:
      return " — SDK：发送入队失败";
    case 0xFC:
      return " — SDK：应答与本次指令不匹配或帧异常";
    case 0xFB:
      return " — SDK：下发帧过短";
    default:
      break;
  }
  if (code >= 0xE1 && code <= 0xE6) {
    return " — 手册协议错误（应答首字节）";
  }
  return "";
}

void print_result(bool ok, const robot_driver::Pd42Motor & motor)
{
  if (ok) {
    std::cout << "成功\n";
  } else {
    const auto code = motor.error_code();
    std::cout << "失败，error_code=0x" << std::hex << std::uppercase
              << static_cast<unsigned>(code) << std::dec << std::nouppercase
              << error_code_hint(code) << "\n";
  }
}

void print_help()
{
  std::cout
    << "指令须带括号（短名）：\n"
    << "  init()              initialize\n"
    << "  clr()               clear_status\n"
    << "  en(true|false)      enable\n"
    << "  mode(0|1)           set_mode 位置|速度\n"
    << "  apos(pos,rpm,accel[,rev])  set_absolute_position\n"
    << "  stop()              急停\n"
    << "  spd(rpm[,rev[,accel]])     set_speed\n"
    << "  rpm()               读转速 RPM（0x29）\n"
    << "  pos()               读位置（51200/圈）（0x2A）\n"
    << "其它: help / quit / exit\n"
    << "错误码: 0xE1～0xE6 手册；0xFF/FE/FC/FB SDK 内部\n";
}

bool dispatch(
  robot_driver::Pd42Motor & motor,
  const std::string & name,
  const std::vector<std::string> & args)
{
  using robot_driver::Pd42CommMode;

  if (name == "init") {
    if (!args.empty()) {
      throw std::runtime_error("init() 无参数");
    }
    return motor.initialize();
  }
  if (name == "clr") {
    if (!args.empty()) {
      throw std::runtime_error("clr() 无参数");
    }
    return motor.clear_status();
  }
  if (name == "en") {
    if (args.size() != 1U) {
      throw std::runtime_error("en(true|false)");
    }
    return motor.enable(parse_bool(args[0]));
  }
  if (name == "mode") {
    if (args.size() != 1U) {
      throw std::runtime_error("mode(0|1)");
    }
    const auto v = static_cast<std::uint8_t>(parse_uint(args[0]) & 0xFFU);
    return motor.set_mode(static_cast<Pd42CommMode>(v));
  }
  if (name == "apos") {
    if (args.size() < 3 || args.size() > 4) {
      throw std::runtime_error("apos(pos,rpm,accel[,rev])");
    }
    const auto pos = static_cast<std::uint32_t>(parse_uint(args[0]));
    const auto sp = static_cast<std::uint16_t>(parse_uint(args[1]) & 0xFFFFU);
    const auto ac = static_cast<std::uint8_t>(parse_uint(args[2]) & 0xFFU);
    if (args.size() == 3U) {
      return motor.set_absolute_position(pos, sp, ac);
    }
    return motor.set_absolute_position(pos, sp, ac, parse_bool(args[3]));
  }
  if (name == "stop") {
    if (!args.empty()) {
      throw std::runtime_error("stop() 无参数");
    }
    return motor.stop();
  }
  if (name == "spd") {
    if (args.empty() || args.size() > 3U) {
      throw std::runtime_error("spd(rpm[, rev[, accel]])");
    }
    const float rpm = parse_float(args[0]);
    if (args.size() == 1U) {
      return motor.set_speed(rpm);
    }
    if (args.size() == 2U) {
      return motor.set_speed(rpm, parse_bool(args[1]));
    }
    const bool rev = parse_bool(args[1]);
    const auto accel = static_cast<std::uint8_t>(parse_uint(args[2]) & 0xFFU);
    return motor.set_speed(rpm, rev, accel);
  }

  throw std::runtime_error("未知指令: " + name);
}

}  // namespace

int main(int argc, char * argv[])
{
  const std::string iface = (argc >= 2) ? argv[1] : "can0";
  const uint8_t motor_id = (argc >= 3) ? static_cast<uint8_t>(std::atoi(argv[2])) : 1U;

  robot_driver::CanInterface can(iface);
  if (!can.open()) {
    std::cerr << "无法打开 CAN 接口: " << iface << "\n";
    return 1;
  }

  robot_driver::Pd42Motor motor(can, motor_id);

  std::cout << "PD42  iface=" << iface << "  motor_id=" << static_cast<int>(motor.motor_id())
            << "  uplink_eff=0x" << std::hex
            << robot_driver::default_can_eff_id(motor.motor_id()) << std::dec << "\n";
  print_help();

  std::string line;
  while (std::cout << "> " && std::getline(std::cin, line)) {
    trim_in_place(line);
    if (line.empty()) {
      continue;
    }
    if (line == "quit" || line == "exit" || line == "q") {
      break;
    }
    if (line == "help" || line == "?") {
      print_help();
      continue;
    }

    std::string name;
    std::vector<std::string> args;
    if (!parse_call(line, name, args)) {
      std::cerr << "格式: 指令(参数)，例如 init()\n";
      continue;
    }

    try {
      if (name == "rpm") {
        if (!args.empty()) {
          throw std::runtime_error("rpm() 无参数");
        }
        const auto v = motor.rpm();
        if (v) {
          std::cout << *v << " RPM\n";
        } else {
          print_result(false, motor);
        }
        continue;
      }
      if (name == "pos") {
        if (!args.empty()) {
          throw std::runtime_error("pos() 无参数");
        }
        const auto v = motor.pos();
        if (v) {
          std::cout << *v << " (51200/圈)\n";
        } else {
          print_result(false, motor);
        }
        continue;
      }

      const bool ok = dispatch(motor, name, args);
      print_result(ok, motor);
    } catch (const std::exception & ex) {
      std::cerr << "解析或调用错误: " << ex.what() << "\n";
    }
  }

  return 0;
}
