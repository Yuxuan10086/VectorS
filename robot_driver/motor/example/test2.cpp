// 简易双前轮差速底盘（左前 id=5，右前 id=4；后轮全向假设不参与驱动，仅几何近似差速）
// 用法: pd42_test2 [can0]
#include "robot_driver/can_interface.hpp"
#include "robot_driver/pd42_motor.hpp"

#include <poll.h>
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <chrono>
#include <iostream>
#include <sstream>
#include <string>

namespace
{

using robot_driver::CanInterface;
using robot_driver::Pd42Motor;
using robot_driver::Pd42CommMode;

/** 驱动轮直径 80 mm（米） */
constexpr double kWheelDiameterM = 0.08;
/** 左右驱动轮中心距（米），实车 45 cm */
constexpr double kTrackM = 0.45;

constexpr double kTravelSpeedMs = 0.1;
/** 差速转向 / 原地自转：单侧轮缘线速度（m/s） */
constexpr double kPivotWheelMs = 0.1;

constexpr std::uint8_t kAccel = 50;

constexpr std::uint8_t kLeftId = 5;
constexpr std::uint8_t kRightId = 4;

/** 5 号（左前）电机默认转向与运动学约定相反：发送时对 reverse 取反 */
constexpr bool kLeftMotorReverseInverted = true;

/** 运动学上的 nominal reverse → 左轮 set_speed 第二参 */
bool left_rev(bool nominal_reverse)
{
  return kLeftMotorReverseInverted ? !nominal_reverse : nominal_reverse;
}

/** 运动中输入 c（回车）置位，用于 interruptible 等待 */
std::atomic<bool> g_motion_stop{false};

static bool line_is_stop_command(const std::string & line)
{
  std::string s = line;
  const auto a = s.find_first_not_of(" \t\r\n");
  if (a == std::string::npos) {
    return false;
  }
  const auto b = s.find_last_not_of(" \t\r\n");
  s = s.substr(a, b - a + 1);
  return s.size() == 1U && (s[0] == 'c' || s[0] == 'C');
}

/**
 * 睡眠 sec 秒；期间若 stdin 收到一行且为 c/C，则置 g_motion_stop 并提前结束。
 * 返回 true 表示未被打断（时间跑满），false 表示用户请求停止。
 */
bool sleep_interruptible(double sec)
{
  using clock = std::chrono::steady_clock;
  if (sec <= 0.0) {
    return true;
  }
  const auto end = clock::now() + std::chrono::duration<double>(sec);
  while (clock::now() < end && !g_motion_stop.load(std::memory_order_relaxed)) {
    const auto remain_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                               end - clock::now())
                               .count();
    if (remain_ms <= 0) {
      break;
    }
    const int wait_ms =
      static_cast<int>(std::min<long long>(50LL, remain_ms));
    struct pollfd pfd {};
    pfd.fd = STDIN_FILENO;
    pfd.events = POLLIN;
    if (poll(&pfd, 1, wait_ms) > 0 && (pfd.revents & POLLIN) != 0) {
      std::string line;
      if (!std::getline(std::cin, line)) {
        break;
      }
      if (line_is_stop_command(line)) {
        g_motion_stop.store(true, std::memory_order_relaxed);
        break;
      }
    }
  }
  return !g_motion_stop.load(std::memory_order_relaxed);
}

/** 线速度 (m/s，轮缘) → RPM，v 取绝对值 */
float rpm_from_wheel_ms(double v_ms)
{
  const double v = std::abs(v_ms);
  return static_cast<float>(v * 60.0 / (M_PI * kWheelDiameterM));
}

bool zero_speed(Pd42Motor & left, Pd42Motor & right)
{
  return left.set_speed(0.F, left_rev(false), kAccel) &&
         right.set_speed(0.F, false, kAccel);
}

bool cmd_go(Pd42Motor & left, Pd42Motor & right, double meters)
{
  const float rpm = rpm_from_wheel_ms(kTravelSpeedMs);
  const double dur = std::abs(meters) / kTravelSpeedMs;
  const bool reverse = (meters < 0.0);
  if (!left.set_speed(rpm, left_rev(reverse), kAccel) ||
      !right.set_speed(rpm, reverse, kAccel)) {
    return false;
  }
  g_motion_stop.store(false, std::memory_order_relaxed);
  (void)sleep_interruptible(dur);
  return zero_speed(left, right);
}

/**
 * 差速转向：与直行同量级轮速 kPivotWheelMs，不按固定角速度（°/s）缩放；
 * 持续时间由 ω = 2v/L 推出。正角度为左转（从上视逆时针）。
 */
bool cmd_rotate(Pd42Motor & left, Pd42Motor & right, double deg)
{
  const double v = kPivotWheelMs;
  const double omega = 2.0 * v / kTrackM;
  const double theta = std::abs(deg) * M_PI / 180.0;
  const double dur = theta / omega;
  const float rpm = rpm_from_wheel_ms(v);
  const bool ccw = (deg > 0.0);
  if (!left.set_speed(rpm, left_rev(ccw), kAccel) ||
      !right.set_speed(rpm, !ccw, kAccel)) {
    return false;
  }
  g_motion_stop.store(false, std::memory_order_relaxed);
  (void)sleep_interruptible(dur);
  return zero_speed(left, right);
}

/** 原地自转：与 cmd_rotate 相同运动学（轮速 kPivotWheelMs）。 */
bool cmd_spin(Pd42Motor & left, Pd42Motor & right, double deg)
{
  return cmd_rotate(left, right, deg);
}

void print_help()
{
  std::cout
    << "简易底盘（左前 id=" << static_cast<int>(kLeftId) << " 右前 id="
    << static_cast<int>(kRightId) << "）\n"
    << "轮径 " << (kWheelDiameterM * 1000.0) << " mm，轴距 " << kTrackM
    << " m；左电机 id " << static_cast<int>(kLeftId)
    << " reverse 已取反（kLeftMotorReverseInverted）\n"
    << "命令:\n"
    << "  go <m>           行进 |m| 米，符号表示前后（速度 " << kTravelSpeedMs << " m/s）\n"
    << "  rotate <deg>     差速转向，轮速 " << kPivotWheelMs << " m/s（与 spin 同）；正=左转\n"
    << "  spin <deg>       原地转向，轮速 " << kPivotWheelMs << " m/s；正=左转\n"
    << "  c                急停：所有电机零速（运动中也可输入 c 后回车）\n"
    << "  help\n"
    << "  quit / exit\n";
}

}  // namespace

int main(int argc, char * argv[])
{
  const std::string iface = (argc >= 2) ? argv[1] : "can0";

  CanInterface can(iface);
  if (!can.open()) {
    std::cerr << "无法打开 CAN: " << iface << "\n";
    return 1;
  }

  Pd42Motor left(can, kLeftId);
  Pd42Motor right(can, kRightId);

  if (!left.initialize()) {
    std::cerr << "左轮 initialize 失败 error_code=0x" << std::hex
              << static_cast<unsigned>(left.error_code()) << std::dec << "\n";
    return 1;
  }
  if (!right.initialize()) {
    std::cerr << "右轮 initialize 失败 error_code=0x" << std::hex
              << static_cast<unsigned>(right.error_code()) << std::dec << "\n";
    return 1;
  }

  if (!left.set_mode(Pd42CommMode::kSpeed) || !right.set_mode(Pd42CommMode::kSpeed)) {
    std::cerr << "set_mode(速度) 失败\n";
    return 1;
  }

  print_help();

  std::string line;
  while (std::cout << "> " && std::getline(std::cin, line)) {
    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;
    if (cmd.empty()) {
      continue;
    }
    if (cmd == "quit" || cmd == "exit") {
      break;
    }
    if (cmd == "help" || cmd == "?") {
      print_help();
      continue;
    }
    if (cmd == "c" || cmd == "C") {
      g_motion_stop.store(true, std::memory_order_relaxed);
      if (!zero_speed(left, right)) {
        std::cerr << "急停发送失败 L err=0x" << std::hex << static_cast<unsigned>(left.error_code())
                  << " R err=0x" << std::hex << static_cast<unsigned>(right.error_code()) << std::dec
                  << "\n";
      }
      continue;
    }

    if (cmd == "go") {
      double m = 0.0;
      if (!(iss >> m)) {
        std::cerr << "用法: go <米>\n";
        continue;
      }
      if (!cmd_go(left, right, m)) {
        std::cerr << "执行失败 L err=0x" << std::hex << static_cast<unsigned>(left.error_code())
                  << " R err=0x" << static_cast<unsigned>(right.error_code()) << std::dec << "\n";
      }
      continue;
    }
    if (cmd == "rotate") {
      double deg = 0.0;
      if (!(iss >> deg)) {
        std::cerr << "用法: rotate <度>\n";
        continue;
      }
      if (!cmd_rotate(left, right, deg)) {
        std::cerr << "执行失败 L err=0x" << std::hex << static_cast<unsigned>(left.error_code())
                  << " R err=0x" << static_cast<unsigned>(right.error_code()) << std::dec << "\n";
      }
      continue;
    }
    if (cmd == "spin") {
      double deg = 0.0;
      if (!(iss >> deg)) {
        std::cerr << "用法: spin <度>\n";
        continue;
      }
      if (!cmd_spin(left, right, deg)) {
        std::cerr << "执行失败 L err=0x" << std::hex << static_cast<unsigned>(left.error_code())
                  << " R err=0x" << static_cast<unsigned>(right.error_code()) << std::dec << "\n";
      }
      continue;
    }

    std::cerr << "未知命令，输入 help\n";
  }

  (void)zero_speed(left, right);
  return 0;
}
