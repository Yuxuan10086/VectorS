// 示例：读 ROS 参数后仅构造 J1/J2 关节与电机，终端交互（关节命令 + 直连电机），不做自动标定
// 参数须由外部提供（如 --params-file），不设代码内默认值
#include "robot_driver/can_interface.hpp"
#include "robot_driver/pd42_motor.hpp"

#include "scara_arm/arm_joint.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cctype>
#include <cerrno>
#include <climits>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace
{

/** 双关节示例所需 ROS 参数（可与完整 scara_arm.yaml 共存；未列出的键不要求） */
static const char * const kRequiredParams[] = {
  "limit_margin_units",
  "motor_j1_id",
  "motor_j2_id",
  "torque_j1_ma",
  "torque_j2_ma",
  "stall_current_j1_ma",
  "stall_current_j2_ma",
  "position_speed_rpm_j1",
  "position_speed_rpm_j2",
  "position_accel_j1",
  "position_accel_j2",
  "bump_speed_rpm_j1",
  "bump_speed_rpm_j2",
  "span_joint1",
  "span_joint2",
};
static constexpr std::size_t kRequiredParamsCount = sizeof(kRequiredParams) / sizeof(kRequiredParams[0]);

/** 要求参数均来自 override（如 YAML）；缺少任一项则失败 */
bool params_ok(const rclcpp::Node & node, std::string & first_missing)
{
  for (std::size_t i = 0; i < kRequiredParamsCount; ++i) {
    if (!node.has_parameter(kRequiredParams[i])) {
      first_missing = kRequiredParams[i];
      return false;
    }
  }
  return true;
}

/** 整串为合法浮点数时写入 out，否则 false */
bool parse_double_strict(const std::string & s, double & out)
{
  try {
    std::size_t idx = 0;
    out = std::stod(s, &idx);
    return idx == s.size();
  } catch (const std::exception &) {
    return false;
  }
}

std::vector<std::string> split_tokens(const std::string & line)
{
  std::vector<std::string> tok;
  std::istringstream iss(line);
  std::string w;
  while (iss >> w) {
    tok.push_back(std::move(w));
  }
  return tok;
}

namespace motor_cli
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
      return " — comm_mode 与指令不符（须先 mode(0|1|2)）";
    case 0xFF:
      return " — 100ms 内未收到应答";
    case 0xFE:
      return " — 发送入队失败";
    case 0xFC:
      return " — 应答与指令不匹配或帧异常";
    case 0xFB:
      return " — 下发帧过短";
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

std::string format_cmd(const std::string & name, const std::vector<std::string> & args)
{
  std::ostringstream o;
  o << name << '(';
  for (std::size_t i = 0; i < args.size(); ++i) {
    if (i > 0) {
      o << ',';
    }
    o << args[i];
  }
  o << ')';
  return o.str();
}

void print_status_line(const robot_driver::Pd42Motor & motor, bool ok)
{
  if (ok) {
    std::cout << "成功 (error_code=0)\n";
  } else {
    const auto code = motor.error_code();
    std::cout << "失败 error_code=0x" << std::hex << std::uppercase
              << static_cast<unsigned>(code) << std::dec << std::nouppercase
              << error_code_hint(code) << "\n";
  }
}

robot_driver::Pd42Motor & motor_by_id(
  robot_driver::Pd42Motor & motor_j1, robot_driver::Pd42Motor & motor_j2, std::uint8_t id,
  std::uint8_t id_j1, std::uint8_t id_j2)
{
  if (id == id_j1) {
    return motor_j1;
  }
  if (id == id_j2) {
    return motor_j2;
  }
  throw std::runtime_error("motor id 须为 YAML motor_j1_id 或 motor_j2_id");
}

bool run_find_limit(
  robot_driver::Pd42Motor & motor, float rpm, bool reverse, std::uint16_t threshold_ma)
{
  using robot_driver::Pd42CommMode;
  constexpr std::uint8_t kAccel = 20;
  if (!motor.set_mode(Pd42CommMode::kSpeed)) {
    std::cout << "（findlimit 内部）set_mode(speed) 失败\n";
    return false;
  }
  if (!motor.set_speed(rpm, reverse, kAccel)) {
    std::cout << "（findlimit 内部）初始 speed 下发失败\n";
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
          std::cout << "（findlimit）|相电流|=" << mag << " mA 超过阈值 " << threshold_ma
                    << "，已下发零速\n";
          return true;
        }
        std::cout << "（findlimit）电流已超阈但零速指令失败\n";
        return false;
      }
    } else {
      ++bad_reads;
      if (bad_reads >= 40) {
        (void)motor.set_speed(0.0f, reverse, kAccel);
        std::cout << "（findlimit）相电流多次读失败，已下发零速\n";
        return false;
      }
    }
    std::this_thread::sleep_for(kPollInterval);
  }
  (void)motor.set_speed(0.0f, reverse, kAccel);
  std::cout << "（findlimit）超时 " << kMaxMove.count() << "s，已下发零速\n";
  return false;
}

bool try_read_commands(
  robot_driver::Pd42Motor & motor, const std::string & name, const std::vector<std::string> & args)
{
  if (name == "rpm") {
    if (!args.empty()) {
      throw std::runtime_error("rpm() 无参数");
    }
    const auto v = motor.rpm();
    if (v) {
      std::cout << "读值 " << *v << " RPM；";
    }
    print_status_line(motor, static_cast<bool>(v));
    return true;
  }
  if (name == "pos") {
    if (!args.empty()) {
      throw std::runtime_error("pos() 无参数");
    }
    const auto v = motor.pos();
    if (v) {
      std::cout << "读值 脉冲=" << *v << "；";
    }
    print_status_line(motor, static_cast<bool>(v));
    return true;
  }
  if (name == "stalled") {
    if (!args.empty()) {
      throw std::runtime_error("stalled() 无参数");
    }
    const auto v = motor.stall_flag();
    if (v) {
      std::cout << "读值 " << (*v ? "true" : "false") << "；";
    }
    print_status_line(motor, static_cast<bool>(v));
    return true;
  }
  if (name == "preset") {
    if (!args.empty()) {
      throw std::runtime_error("preset() 无参数");
    }
    const auto v = motor.stall_current_setting_ma();
    if (v) {
      std::cout << "读值 堵转电流设定=" << *v << " mA；";
    }
    print_status_line(motor, static_cast<bool>(v));
    return true;
  }
  if (name == "phase") {
    if (!args.empty()) {
      throw std::runtime_error("phase() 无参数");
    }
    const auto v = motor.phase_current_ma();
    if (v) {
      std::cout << "读值 相电流=" << *v << " mA；";
    }
    print_status_line(motor, static_cast<bool>(v));
    return true;
  }
  if (name == "comm") {
    if (!args.empty()) {
      throw std::runtime_error("comm() 无参数");
    }
    const auto m = motor.comm_mode();
    if (m) {
      const int code = static_cast<int>(*m);
      std::cout << "读值 comm_mode=" << code;
      if (code == 0) {
        std::cout << " (position)";
      } else if (code == 1) {
        std::cout << " (speed)";
      } else if (code == 2) {
        std::cout << " (torque)";
      }
      std::cout << "；";
    } else {
      std::cout << "读值 unknown（本机未记录 set_mode）；";
    }
    std::cout << "成功 (error_code=0)\n";
    return true;
  }
  return false;
}

bool dispatch_write(
  robot_driver::Pd42Motor & motor, const std::string & name, const std::vector<std::string> & args)
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
    if (args.size() < 3 || args.size() > 4U) {
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
      throw std::runtime_error("torque(reverse,ma)");
    }
    const bool rev = parse_bool(args[0]);
    const auto ma = static_cast<std::uint16_t>(parse_uint(args[1]) & 0xFFFFU);
    return motor.set_torque(rev, ma);
  }
  if (name == "origin") {
    if (args.size() != 2U) {
      throw std::runtime_error("origin(left,right)");
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
      throw std::runtime_error("protect(ma)");
    }
    const auto ma = static_cast<std::uint16_t>(parse_uint(args[0]) & 0xFFFFU);
    return motor.enable_stall_protection(ma);
  }
  if (name == "findlimit") {
    if (args.size() != 3U) {
      throw std::runtime_error("findlimit(rpm,dir,threshold_ma) dir 0|1（0 正转 1 反转）");
    }
    const float rpm = parse_float(args[0]);
    const unsigned dir = parse_uint(args[1]);
    if (dir > 1U) {
      throw std::runtime_error("dir 须为 0 或 1");
    }
    const bool reverse = (dir == 1U);
    const auto thr = static_cast<std::uint16_t>(parse_uint(args[2]) & 0xFFFFU);
    return run_find_limit(motor, rpm, reverse, thr);
  }

  throw std::runtime_error("未知命令: " + name);
}

void print_motor_help(std::uint8_t id_j1, std::uint8_t id_j2)
{
  std::cout << "\n直连电机（首字段为电机 ID，须为 motor_j1_id 或 motor_j2_id）：\n"
            << "格式: <id> <命令>(参数...)\n"
            << "例: " << static_cast<int>(id_j2) << " pos()\n"
            << "电机 ID（YAML）：" << static_cast<int>(id_j1) << " / " << static_cast<int>(id_j2) << "\n"
            << "init()\n"
            << "clear()\n"
            << "enable(true|false)\n"
            << "mode(0|1|2)\n"
            << "move(pos,rpm,accel[,rev])\n"
            << "stop()\n"
            << "speed(rpm[,rev[,accel]])\n"
            << "torque(reverse,ma)\n"
            << "origin(left,right)\n"
            << "limit(true|false)\n"
            << "zero()\n"
            << "save()\n"
            << "protect(ma)\n"
            << "findlimit(rpm,dir,threshold_ma)\n"
            << "rpm()\n"
            << "pos()\n"
            << "stalled()\n"
            << "preset()\n"
            << "phase()\n"
            << "comm()\n"
            << "每行末尾打印 [motor id=N] 指令(...) -> 读值/执行结果 与 error_code\n\n";
}

/** 若本行为 <motor_id> <name(args)> 且 id 为 J1/J2 则执行并返回 true */
bool try_dispatch_motor_line(
  const std::string & line, robot_driver::Pd42Motor & motor_j1, robot_driver::Pd42Motor & motor_j2,
  std::uint8_t id_j1, std::uint8_t id_j2)
{
  std::string lt = line;
  trim_in_place(lt);
  const std::size_t sp = lt.find_first_of(" \t");
  if (sp == std::string::npos) {
    return false;
  }
  std::string idw = lt.substr(0, sp);
  std::string cmdpart = lt.substr(lt.find_first_not_of(" \t", sp));
  trim_in_place(cmdpart);
  if (cmdpart.empty() || cmdpart.find('(') == std::string::npos) {
    return false;
  }

  std::uint8_t mid = 0;
  try {
    const unsigned long v = parse_uint(idw);
    if (v > 255U) {
      return false;
    }
    mid = static_cast<std::uint8_t>(v);
  } catch (const std::exception &) {
    return false;
  }
  if (mid != id_j1 && mid != id_j2) {
    return false;
  }

  std::string name;
  std::vector<std::string> args;
  if (!parse_call(cmdpart, name, args)) {
    std::cerr << "电机行格式错误，应为: <id> 命令(参数)\n";
    return true;
  }

  robot_driver::Pd42Motor & m = motor_by_id(motor_j1, motor_j2, mid, id_j1, id_j2);
  std::cout << "[motor id=" << static_cast<int>(mid) << "] " << format_cmd(name, args) << " -> ";
  std::cout.flush();

  try {
    if (try_read_commands(m, name, args)) {
      return true;
    }
    const bool ok = dispatch_write(m, name, args);
    print_status_line(m, ok);
  } catch (const std::exception & ex) {
    std::cout << "解析/调用异常: " << ex.what() << "\n";
  }
  return true;
}

}  // namespace motor_cli

void print_joint_help(std::uint16_t default_torque_j1, std::uint16_t default_torque_j2)
{
  std::cout << "关节命令（首词 j1 或 j2，其后为 命令(参数)）：\n"
            << "j1 bump(dir[,ma])\n"
            << "  碰停；dir 0=正向 1=反向；省略 ma 时 J1 默认 " << default_torque_j1
            << " mA，J2 默认 " << default_torque_j2 << " mA（YAML torque_*）\n"
            << "j1 limits()\n"
            << "  set_limits（须已完成正向 bump）\n"
            << "j1 move(span)\n"
            << "  set_position；须 limits() 后，span 在由 margin/H/span_max 算出的可达子区间内\n"
            << "j1 summary()\n"
            << "  标定摘要\n"
            << "j1 <幅度>\n"
            << "  简写，等价 j1 move(<幅度>)\n"
            << "<幅度1> <幅度2>\n"
            << "  两轴简写，先 J1 后 J2\n"
            << "help\n"
            << "  本帮助\n\n";
}

/**
 * 关节帮助 + 直连电机帮助；stdout 为 TTY 时整块绿色加粗。
 * include_ready：启动时含「已就绪」一行；输入 help 时为 false。
 */
void print_interactive_help_screen(
  bool include_ready, std::uint16_t torque_j1_ma, std::uint16_t torque_j2_ma,
  std::uint8_t motor_id_j1, std::uint8_t motor_id_j2)
{
  const bool tty = ::isatty(STDOUT_FILENO) != 0;
  if (tty) {
    std::cout << "\033[32m\033[1m";
  }
  if (include_ready) {
    std::cout << "\nJ1/J2 已就绪（未执行自动标定），等待命令。\n";
  }
  print_joint_help(torque_j1_ma, torque_j2_ma);
  motor_cli::print_motor_help(motor_id_j1, motor_id_j2);
  if (tty) {
    std::cout << "\033[0m";
  }
}

/** j1/j2 的 bump|limits|move|summary；已处理返回 true */
bool try_dispatch_joint_line(
  const std::vector<std::string> & tok, scara_arm::ArmJoint & j1, scara_arm::ArmJoint & j2,
  std::uint16_t default_torque_j1, std::uint16_t default_torque_j2)
{
  if (tok.size() < 2U) {
    return false;
  }
  const std::string & which = tok[0];
  if (which != "j1" && which != "j2") {
    return false;
  }
  std::string cmdline;
  for (std::size_t i = 1; i < tok.size(); ++i) {
    if (i > 1) {
      cmdline += ' ';
    }
    cmdline += tok[i];
  }
  if (cmdline.find('(') == std::string::npos) {
    return false;
  }

  std::string name;
  std::vector<std::string> args;
  if (!motor_cli::parse_call(cmdline, name, args)) {
    std::cerr << "关节行格式错误，应为: j1 命令(参数)\n";
    return true;
  }

  scara_arm::ArmJoint * joint = (which == "j1") ? &j1 : &j2;
  const std::uint16_t def_ma = (which == "j1") ? default_torque_j1 : default_torque_j2;

  std::cout << "[" << which << "] " << motor_cli::format_cmd(name, args) << " -> ";
  std::cout.flush();

  bool ok = false;
  try {
    if (name == "bump") {
      if (args.empty() || args.size() > 2U) {
        throw std::runtime_error("bump(dir[,ma_mA])");
      }
      const unsigned d = motor_cli::parse_uint(args[0]);
      if (d > 1U) {
        throw std::runtime_error("dir 须为 0 或 1");
      }
      const std::uint8_t dir = static_cast<std::uint8_t>(d);
      std::uint16_t ma = def_ma;
      if (args.size() == 2U) {
        ma = static_cast<std::uint16_t>(motor_cli::parse_uint(args[1]) & 0xFFFFU);
      }
      ok = joint->bump(dir, ma);
      motor_cli::print_status_line(joint->motor(), ok);
      return true;
    }
    if (name == "limits") {
      if (!args.empty()) {
        throw std::runtime_error("limits() 无参数");
      }
      ok = joint->set_limits().has_value();
      motor_cli::print_status_line(joint->motor(), ok);
      return true;
    }
    if (name == "move") {
      if (args.size() != 1U) {
        throw std::runtime_error("move(span)");
      }
      double span = 0.0;
      if (!parse_double_strict(args[0], span)) {
        throw std::runtime_error("span 须为浮点数");
      }
      const auto Hopt = joint->forward_stop_pulse();
      const std::int32_t mu = joint->limit_margin_units();
      double rmin = 0.0;
      double rmax = 0.0;
      bool have_range = false;
      if (Hopt && *Hopt > 2 * mu && joint->span_max() > 0.0) {
        const double H = static_cast<double>(*Hopt);
        rmin = (static_cast<double>(mu) / H) * joint->span_max();
        rmax = (static_cast<double>(*Hopt - mu) / H) * joint->span_max();
        have_range = true;
      }
      if (!have_range) {
        std::cout << "失败：须先 bump(0) 再 limits() 后 move 才有效\n";
        return true;
      }
      if (span < rmin || span > rmax) {
        std::cout << "失败 span 须在可达区间 [" << rmin << ", " << rmax << "]（名义最大 " << joint->span_max()
                  << "）\n";
        return true;
      }
      ok = joint->set_position(span);
      motor_cli::print_status_line(joint->motor(), ok);
      if (ok) {
        std::cout << "  span_now=" << joint->span_now() << '\n';
      }
      return true;
    }
    if (name == "summary") {
      if (!args.empty()) {
        throw std::runtime_error("summary() 无参数");
      }
      joint->print_calibration_summary(std::cout);
      std::cout << "成功 (error_code=0)\n";
      return true;
    }
    throw std::runtime_error("未知关节命令: " + name);
  } catch (const std::exception & ex) {
    std::cout << "异常: " << ex.what() << "\n";
  }
  return true;
}

/**
 * 交互：关节命令 j1/j2、直连电机、简写幅度；不做上电标定
 */
int interactive_session(
  scara_arm::ArmJoint & joint1, scara_arm::ArmJoint & joint2, robot_driver::Pd42Motor & motor_j1,
  robot_driver::Pd42Motor & motor_j2, std::uint8_t motor_id_j1, std::uint8_t motor_id_j2,
  std::uint16_t torque_j1_ma, std::uint16_t torque_j2_ma)
{
  if (!::isatty(STDIN_FILENO)) {
    std::cerr
      << "\n错误：stdin 不是终端（常见于 ros2 launch：子进程拿不到键盘输入）。\n"
      << "请在本机终端直接运行，例如：\n"
      << "  ros2 run scara_arm scara_arm_joints_repl --ros-args --params-file <path/to/scara_arm.yaml>\n\n";
    return 0;
  }

  print_interactive_help_screen(true, torque_j1_ma, torque_j2_ma, motor_id_j1, motor_id_j2);

  std::string line;
  for (;;) {
    if (!rclcpp::ok()) {
      std::cout << "\n（ROS 已 shutdown）退出。\n";
      return 0;
    }

    std::cout << "> " << std::flush;
    line.clear();
    bool got_line = false;
    while (rclcpp::ok() && !got_line) {
      pollfd pfd{};
      pfd.fd = STDIN_FILENO;
      pfd.events = POLLIN;
      const int pr = poll(&pfd, 1, 200);
      if (!rclcpp::ok()) {
        std::cout << "\n（中断）退出。\n";
        return 0;
      }
      if (pr < 0) {
        if (errno == EINTR) {
          continue;
        }
        break;
      }
      if (pr == 0) {
        continue;
      }
      if (pfd.revents & (POLLIN | POLLRDNORM)) {
        if (!std::getline(std::cin, line)) {
          std::cout << "\n（EOF）退出。\n";
          return 0;
        }
        got_line = true;
      } else if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
        break;
      }
    }
    if (!rclcpp::ok()) {
      std::cout << "\n（中断）退出。\n";
      return 0;
    }
    if (!got_line) {
      break;
    }

    if (motor_cli::try_dispatch_motor_line(line, motor_j1, motor_j2, motor_id_j1, motor_id_j2)) {
      continue;
    }

    const auto tok = split_tokens(line);
    if (tok.empty()) {
      continue;
    }
    if (tok[0] == "q" || tok[0] == "quit" || tok[0] == "exit") {
      std::cout << "再见。\n";
      return 0;
    }
    if (tok[0] == "help" || tok[0] == "?") {
      print_interactive_help_screen(false, torque_j1_ma, torque_j2_ma, motor_id_j1, motor_id_j2);
      continue;
    }

    if (try_dispatch_joint_line(tok, joint1, joint2, torque_j1_ma, torque_j2_ma)) {
      continue;
    }

    if (tok.size() == 2U) {
      double v = 0.0;
      if (tok[0] == "j1") {
        if (!parse_double_strict(tok[1], v)) {
          std::cerr << "无效数字，请使用：j1 <幅度> 或 j1 move(<幅度>)\n";
          continue;
        }
        if (v < 0.0 || v > joint1.span_max()) {
          std::cerr << "J1 幅度须在 [0, " << joint1.span_max() << "] 内\n";
          continue;
        }
        std::cout << "[j1] move(" << v << ") -> ";
        std::cout.flush();
        const bool ok = joint1.set_position(v);
        motor_cli::print_status_line(joint1.motor(), ok);
        if (ok) {
          std::cout << "  span_now=" << joint1.span_now() << '\n';
        }
        continue;
      }
      if (tok[0] == "j2") {
        if (!parse_double_strict(tok[1], v)) {
          std::cerr << "无效数字，请使用：j2 <幅度>\n";
          continue;
        }
        if (v < 0.0 || v > joint2.span_max()) {
          std::cerr << "J2 幅度须在 [0, " << joint2.span_max() << "] 内\n";
          continue;
        }
        std::cout << "[j2] move(" << v << ") -> ";
        std::cout.flush();
        const bool ok = joint2.set_position(v);
        motor_cli::print_status_line(joint2.motor(), ok);
        if (ok) {
          std::cout << "  span_now=" << joint2.span_now() << '\n';
        }
        continue;
      }

      double s1 = 0.0;
      double s2 = 0.0;
      if (parse_double_strict(tok[0], s1) && parse_double_strict(tok[1], s2)) {
        if (s1 < 0.0 || s1 > joint1.span_max() || s2 < 0.0 || s2 > joint2.span_max()) {
          std::cerr << "两轴幅度须在 [0,span_max] 内（J1 max=" << joint1.span_max()
                    << ", J2 max=" << joint2.span_max() << "）\n";
          continue;
        }
        std::cout << "[j1+j2] move " << s1 << ", " << s2 << " ->\n";
        const bool ok1 = joint1.set_position(s1);
        std::cout << "  [j1] ";
        motor_cli::print_status_line(joint1.motor(), ok1);
        const bool ok2 = joint2.set_position(s2);
        std::cout << "  [j2] ";
        motor_cli::print_status_line(joint2.motor(), ok2);
        if (ok1 && ok2) {
          std::cout << "  span_now J1=" << joint1.span_now() << " J2=" << joint2.span_now() << '\n';
        }
        continue;
      }

      std::cerr << "无法理解：help / <motorId> cmd() / j1 bump() / j1 30 / 30 50 / q\n";
      continue;
    }

    std::cerr << "指令格式不对（多字段请用 j1 move(...) 或 help）\n";
  }

  return 0;
}

int run(const std::shared_ptr<rclcpp::Node> & node)
{
  std::string missing;
  if (!params_ok(*node, missing)) {
    std::cerr
      << "缺少 ROS 参数 \"" << missing
      << "\"；本示例仅需 J1/J2 相关键（见源码 kRequiredParams）。\n"
      << "例如: ros2 run scara_arm scara_arm_joints_repl --ros-args --params-file <path/to/scara_arm.yaml>\n";
    return 2;
  }

  std::int32_t margin = 0;
  int id_j1 = 0;
  int id_j2 = 0;
  int tj1 = 0;
  int tj2 = 0;
  int j1_stall = 0;
  int j2_stall = 0;
  int ps_j1 = 0;
  int ps_j2 = 0;
  int pa_j1 = 0;
  int pa_j2 = 0;
  int bump_j1 = 0;
  int bump_j2 = 0;
  double span_joint1 = 0.0;
  double span_joint2 = 0.0;

  (void)node->get_parameter("limit_margin_units", margin);
  (void)node->get_parameter("motor_j1_id", id_j1);
  (void)node->get_parameter("motor_j2_id", id_j2);
  (void)node->get_parameter("torque_j1_ma", tj1);
  (void)node->get_parameter("torque_j2_ma", tj2);
  (void)node->get_parameter("stall_current_j1_ma", j1_stall);
  (void)node->get_parameter("stall_current_j2_ma", j2_stall);
  (void)node->get_parameter("position_speed_rpm_j1", ps_j1);
  (void)node->get_parameter("position_speed_rpm_j2", ps_j2);
  (void)node->get_parameter("position_accel_j1", pa_j1);
  (void)node->get_parameter("position_accel_j2", pa_j2);
  (void)node->get_parameter("bump_speed_rpm_j1", bump_j1);
  (void)node->get_parameter("bump_speed_rpm_j2", bump_j2);
  (void)node->get_parameter("span_joint1", span_joint1);
  (void)node->get_parameter("span_joint2", span_joint2);

  const auto uj1 = static_cast<std::uint8_t>(id_j1);
  const auto uj2 = static_cast<std::uint8_t>(id_j2);

  if (uj1 == 0U || uj2 == 0U) {
    std::cerr << "motor_j1_id / motor_j2_id 均不可为 0\n";
    return 2;
  }

  auto clamp_u16 = [](int v) -> std::uint16_t {
    if (v <= 0) {
      return 0U;
    }
    if (v > 65535) {
      return 65535U;
    }
    return static_cast<std::uint16_t>(v);
  };
  auto clamp_u8 = [](int v) -> std::uint8_t {
    if (v <= 0) {
      return 0U;
    }
    if (v > 255) {
      return 255U;
    }
    return static_cast<std::uint8_t>(v);
  };

  auto can_if = std::make_unique<robot_driver::CanInterface>();
  if (!can_if->open()) {
    std::cerr << "无法打开 CAN（接口名在 motor 库 can_interface.cpp 内固定）\n";
    return 1;
  }

  std::unique_ptr<robot_driver::Pd42Motor> m1;
  std::unique_ptr<robot_driver::Pd42Motor> m2;
  std::unique_ptr<scara_arm::ArmJoint> j1;
  std::unique_ptr<scara_arm::ArmJoint> j2;

  try {
    m1 = std::make_unique<robot_driver::Pd42Motor>(
      *can_if, uj1, static_cast<std::uint16_t>(j1_stall));
    m2 = std::make_unique<robot_driver::Pd42Motor>(
      *can_if, uj2, static_cast<std::uint16_t>(j2_stall));
    const auto margin_units = static_cast<std::int32_t>(margin);
    const scara_arm::ArmJointParams j1_params{
      uj1, margin_units, span_joint1, clamp_u16(ps_j1), clamp_u8(pa_j1),
      clamp_u16(j1_stall), clamp_u16(bump_j1), "J1"};
    const scara_arm::ArmJointParams j2_params{
      uj2, margin_units, span_joint2, clamp_u16(ps_j2), clamp_u8(pa_j2),
      clamp_u16(j2_stall), clamp_u16(bump_j2), "J2"};
    j1 = std::make_unique<scara_arm::ArmJoint>(*m1, j1_params);
    j2 = std::make_unique<scara_arm::ArmJoint>(*m2, j2_params);
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }

  return interactive_session(
    *j1, *j2, *m1, *m2, uj1, uj2, static_cast<std::uint16_t>(tj1),
    static_cast<std::uint16_t>(tj2));
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("scara_arm_test", options);
  const int rc = run(node);
  rclcpp::shutdown();
  return rc;
}
