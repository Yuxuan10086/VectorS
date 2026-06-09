// 交互式调用 Pd42Motor；命令为单词形式，例如 init() / speed(200) / rpm()
// 用法: pd42_motor_repl [电机ID]（CAN 网卡名见 can_interface.cpp 内常量）
#include "robot_driver/can_interface.hpp"
#include "robot_driver/pd42_motor.hpp"
#include "robot_driver/pd42_protocol.hpp"

#include <chrono>
#include <cctype>
#include <cerrno>
#include <climits>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <thread>
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

std::int32_t parse_int32(const std::string & tok)
{
  errno = 0;
  char * end = nullptr;
  const long v = std::strtol(tok.c_str(), &end, 0);
  if (!end || *end != '\0' || errno == ERANGE) {
    throw std::runtime_error("非法有符号整数: " + tok);
  }
  if (v < static_cast<long>(INT32_MIN) || v > static_cast<long>(INT32_MAX)) {
    throw std::runtime_error("数值超出 int32 范围: " + tok);
  }
  return static_cast<std::int32_t>(v);
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
    case 0xFA:
      return " — SDK：comm_mode 与指令不符（须先 mode(0|1|2) 对应 move/speed/力矩）";
    case 0xFF:
      return " — SDK：100ms 内未收到应答（检查接线/从站地址/回包 CAN ID；非手册 E1～E6）";
    case 0xFE:
      return " — SDK：发送入队失败";
    case 0xFC:
      return " — SDK：应答与本次指令不匹配或帧异常";
    case 0xFB:
      return " — SDK：下发帧过短";
    case 0xE1:
      return " — 帧长度不足";
    case 0xE2:
      return " — 帧头错误（非0xC5）";
    case 0xE3:
      return " — 帧尾错误（非0x5C）";
    case 0xE4:
      return " — 校验和错误";
    case 0xE5:
      return " — 不支持的功能码";
    case 0xE6:
      return " — 数据不合法";
    default:
      return "";
  }
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
    << "命令须带括号（单词名）：\n"
    << "  init()                         initialize\n"
    << "  clear()                        clear_status\n"
    << "  enable(true|false)             enable\n"
    << "  mode(0|1|2)                    set_mode 位置|速度|力矩（move/speed/torque 须先切对模式）\n"
    << "  move(pos,rpm,accel[,rev])      set_absolute_position\n"
    << "  stop()                         急停\n"
    << "  speed(rpm[,rev[,accel]])       set_speed（须 mode(1)）\n"
    << "  torque(reverse,ma)             set_torque（须 mode(2)；reverse true|false；ma 0~3000）\n"
    << "  origin(left,right)             set_limit_origins（左右限位原点，有符号脉冲）\n"
    << "  limit(true|false)              set_limit_sw 限位行程开关\n"
    << "  zero()                         set_zero_position 当前位置清零\n"
    << "  save()                         save_parameters 掉电保存\n"
    << "  protect(ma)                    enable_stall_protection 堵转电流与保护（0~3000 mA）\n"
    << "组合:\n"
    << "  findlimit(rpm,dir,ma)          找限位：切速度模式后 speed(rpm,dir,accel=20)，"
    << "等 100ms 后轮询 phase()，|相电流|>ma 时 speed(0,dir,20)；dir 0|1（0 正转 1 反转）\n"
    << "读回:\n"
    << "  rpm()                          转速 RPM（0x29）\n"
    << "  pos()                          位置（51200/圈）（0x2A）\n"
    << "  stalled()                      stall_flag 堵转标志\n"
    << "  preset()                       stall_current_setting_ma 堵转电流设定 mA（0x2E）\n"
    << "  phase()                        phase_current_ma 相电流 mA（0x23）\n"
    << "  comm()                         comm_mode 本机记录的通信模式（0/1/2 或未知）\n"
    << "其它: help / quit / exit\n"
    << "应答错误字节: 0x01 成功；0xE1 帧长度不足；0xE2 帧头非0xC5；0xE3 帧尾非0x5C；"
    << "0xE4 校验和错误；0xE5 不支持的功能码；0xE6 数据不合法；0xFA 模式不符；0xFF/FE/FC/FB SDK\n";
}

/** 处理只读命令；已处理返回 true（含失败已打印） */
bool try_read_commands(robot_driver::Pd42Motor & motor, const std::string & name,
  const std::vector<std::string> & args)
{
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
    return true;
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
    return true;
  }
  if (name == "stalled") {
    if (!args.empty()) {
      throw std::runtime_error("stalled() 无参数");
    }
    const auto v = motor.stall_flag();
    if (v) {
      std::cout << (*v ? "true" : "false") << "\n";
    } else {
      print_result(false, motor);
    }
    return true;
  }
  if (name == "preset") {
    if (!args.empty()) {
      throw std::runtime_error("preset() 无参数");
    }
    const auto v = motor.stall_current_setting_ma();
    if (v) {
      std::cout << *v << " mA\n";
    } else {
      print_result(false, motor);
    }
    return true;
  }
  if (name == "phase") {
    if (!args.empty()) {
      throw std::runtime_error("phase() 无参数");
    }
    const auto v = motor.phase_current_ma();
    if (v) {
      std::cout << *v << " mA\n";
    } else {
      print_result(false, motor);
    }
    return true;
  }
  if (name == "comm") {
    if (!args.empty()) {
      throw std::runtime_error("comm() 无参数");
    }
    const auto m = motor.comm_mode();
    if (m) {
      const int code = static_cast<int>(*m);
      std::cout << code;
      if (code == 0) {
        std::cout << " (position)\n";
      } else if (code == 1) {
        std::cout << " (speed)\n";
      } else if (code == 2) {
        std::cout << " (torque)\n";
      } else {
        std::cout << "\n";
      }
    } else {
      std::cout << "unknown（尚未 set_mode 成功或未记录）\n";
    }
    return true;
  }
  return false;
}

/** 找限位：定速 → 等 100ms → 轮询相电流超阈 → 零速（加速度固定 20） */
bool run_find_limit(
  robot_driver::Pd42Motor & motor, float rpm, bool reverse, std::uint16_t threshold_ma)
{
  using robot_driver::Pd42CommMode;
  constexpr std::uint8_t kAccel = 20;
  if (!motor.set_mode(Pd42CommMode::kSpeed)) {
    std::cout << "找限位：set_mode(speed) 失败\n";
    return false;
  }
  if (!motor.set_speed(rpm, reverse, kAccel)) {
    std::cout << "找限位：初始 speed 下发失败\n";
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  constexpr auto kPollInterval = std::chrono::milliseconds(5);
  constexpr auto kMaxMove = std::chrono::seconds(60);
  const auto deadline = std::chrono::steady_clock::now() + kMaxMove;
  int bad_reads = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto ph = motor.phase_current_ma();
    if (ph) {
      bad_reads = 0;
      const int mag = (*ph >= 0) ? static_cast<int>(*ph) : -static_cast<int>(*ph);
      if (mag > static_cast<int>(threshold_ma)) {
        if (motor.set_speed(0.0f, reverse, kAccel)) {
          std::cout << "找限位：|相电流|=" << mag << " mA 超过阈值 " << threshold_ma
                    << "，已下发零速\n";
          return true;
        }
        std::cout << "找限位：电流已超阈但零速指令失败\n";
        return false;
      }
    } else {
      ++bad_reads;
      if (bad_reads >= 40) {
        (void)motor.set_speed(0.0f, reverse, kAccel);
        std::cout << "找限位：相电流多次读失败，已下发零速\n";
        return false;
      }
    }
    std::this_thread::sleep_for(kPollInterval);
  }
  (void)motor.set_speed(0.0f, reverse, kAccel);
  std::cout << "找限位：等待超 " << kMaxMove.count() << "s，已下发零速\n";
  return false;
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
  if (name == "clear") {
    if (!args.empty()) {
      throw std::runtime_error("clear() 无参数");
    }
    return motor.clear_status();
  }
  if (name == "enable") {
    if (args.size() != 1U) {
      throw std::runtime_error("enable(true|false)");
    }
    return motor.enable(parse_bool(args[0]));
  }
  if (name == "mode") {
    if (args.size() != 1U) {
      throw std::runtime_error("mode(0|1|2)");
    }
    const auto v = static_cast<std::uint8_t>(parse_uint(args[0]) & 0xFFU);
    return motor.set_mode(static_cast<Pd42CommMode>(v));
  }
  if (name == "move") {
    if (args.size() < 3 || args.size() > 4) {
      throw std::runtime_error("move(pos,rpm,accel[,rev])");
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
  if (name == "speed") {
    if (args.empty() || args.size() > 3U) {
      throw std::runtime_error("speed(rpm[,rev[,accel]])");
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
  if (name == "torque") {
    if (args.size() != 2U) {
      throw std::runtime_error("torque(reverse,ma) 例如 torque(false,500)");
    }
    const bool rev = parse_bool(args[0]);
    const auto ma = static_cast<std::uint16_t>(parse_uint(args[1]) & 0xFFFFU);
    return motor.set_torque(rev, ma);
  }
  if (name == "origin") {
    if (args.size() != 2U) {
      throw std::runtime_error("origin(left,right) 有符号 int32");
    }
    return motor.set_limit_origins(parse_int32(args[0]), parse_int32(args[1]));
  }
  if (name == "limit") {
    if (args.size() != 1U) {
      throw std::runtime_error("limit(true|false)");
    }
    return motor.set_limit_sw(parse_bool(args[0]));
  }
  if (name == "zero") {
    if (!args.empty()) {
      throw std::runtime_error("zero() 无参数");
    }
    return motor.set_zero_position();
  }
  if (name == "save") {
    if (!args.empty()) {
      throw std::runtime_error("save() 无参数");
    }
    return motor.save_parameters();
  }
  if (name == "protect") {
    if (args.size() != 1U) {
      throw std::runtime_error("protect(ma) 堵转电流 mA，0~3000");
    }
    const auto ma = static_cast<std::uint16_t>(parse_uint(args[0]) & 0xFFFFU);
    return motor.enable_stall_protection(ma);
  }
  if (name == "findlimit") {
    if (args.size() != 3U) {
      throw std::runtime_error("findlimit(rpm,dir,threshold_ma) dir 为 0|1，加速度固定 20");
    }
    const float rpm = parse_float(args[0]);
    const unsigned dir = parse_uint(args[1]);
    if (dir > 1U) {
      throw std::runtime_error("dir 须为 0 或 1（0 正转 1 反转）");
    }
    const bool reverse = (dir == 1U);
    const auto thr = static_cast<std::uint16_t>(parse_uint(args[2]) & 0xFFFFU);
    return run_find_limit(motor, rpm, reverse, thr);
  }

  throw std::runtime_error("未知命令: " + name);
}

}  // namespace

int main(int argc, char * argv[])
{
  const uint8_t motor_id = (argc >= 2) ? static_cast<uint8_t>(std::atoi(argv[1])) : 1U;

  robot_driver::CanInterface can;
  if (!can.open()) {
    std::cerr << "无法打开 CAN（接口名在 can_interface.cpp 内固定）\n";
    return 1;
  }

  robot_driver::Pd42Motor motor(can, motor_id);

  std::cout << "PD42  motor_id=" << static_cast<int>(motor.motor_id())
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
      std::cerr << "格式: 命令(参数)，例如 init()\n";
      continue;
    }

    try {
      if (try_read_commands(motor, name, args)) {
        continue;
      }
      const bool ok = dispatch(motor, name, args);
      if (!(name == "findlimit" && ok)) {
        print_result(ok, motor);
      }
    } catch (const std::exception & ex) {
      std::cerr << "解析或调用错误: " << ex.what() << "\n";
    }
  }

  return 0;
}
