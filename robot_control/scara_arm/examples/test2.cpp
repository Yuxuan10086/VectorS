// 示例：读 ROS 参数后构造 RobotArm（Z/J1/J2），终端仅接受机械臂类指令；不做自动标定
// 参数须由外部提供（如 --params-file），不设代码内默认值
#include "scara_arm/robot_arm.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cerrno>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <poll.h>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

namespace
{

/** RobotArm 所需 ROS 参数（可与完整 scara_arm.yaml 共存；未列出的键不要求） */
static const char * const kRequiredParams[] = {
  "can_interface",
  "limit_margin_units",
  "speed_stop_threshold_rpm",
  "stall_stable_ms",
  "poll_period_ms",
  "seek_timeout_s",
  "motor_z_id",
  "motor_j1_id",
  "motor_j2_id",
  "torque_z_ma",
  "torque_j1_ma",
  "torque_j2_ma",
  "stall_current_z_ma",
  "stall_current_j1_ma",
  "stall_current_j2_ma",
  "position_speed_rpm_z",
  "position_speed_rpm_j1",
  "position_speed_rpm_j2",
  "position_accel_z",
  "position_accel_j1",
  "position_accel_j2",
  "bump_speed_rpm_z",
  "bump_speed_rpm_j1",
  "bump_speed_rpm_j2",
  "span_z",
  "span_joint1",
  "span_joint2",
};
static constexpr std::size_t kRequiredParamsCount = sizeof(kRequiredParams) / sizeof(kRequiredParams[0]);

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

scara_arm::BumpCfg load_bump_cfg(const rclcpp::Node & node)
{
  scara_arm::BumpCfg c;
  (void)node.get_parameter("speed_stop_threshold_rpm", c.rpm_zero_th);
  (void)node.get_parameter("stall_stable_ms", c.stable_ms);
  (void)node.get_parameter("poll_period_ms", c.poll_ms);
  (void)node.get_parameter("seek_timeout_s", c.timeout_s);
  return c;
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

void print_robot_arm_help(bool include_ready)
{
  const bool tty = ::isatty(STDOUT_FILENO) != 0;
  if (tty) {
    std::cout << "\033[32m\033[1m";
  }
  if (include_ready) {
    std::cout << "\nRobotArm 已就绪（无上电自动动作），等待指令。\n";
  }
  std::cout << "机械臂类指令：\n"
            << "  calibrate   调用 RobotArm::calibrate()\n"
            << "  help / ?    本帮助\n"
            << "  q / quit / exit   退出\n\n";
  if (tty) {
    std::cout << "\033[0m";
  }
}

int interactive_session(scara_arm::RobotArm & arm)
{
  if (!::isatty(STDIN_FILENO)) {
    std::cerr
      << "\n错误：stdin 不是终端（常见于 ros2 launch：子进程拿不到键盘输入）。\n"
      << "请在本机终端直接运行，例如：\n"
      << "  ros2 run scara_arm test2 --ros-args --params-file <path/to/scara_arm.yaml>\n\n";
    return 0;
  }

  print_robot_arm_help(true);

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

    const auto tok = split_tokens(line);
    if (tok.empty()) {
      continue;
    }
    if (tok[0] == "q" || tok[0] == "quit" || tok[0] == "exit") {
      std::cout << "再见。\n";
      return 0;
    }
    if (tok[0] == "help" || tok[0] == "?") {
      print_robot_arm_help(false);
      continue;
    }
    if (tok.size() == 1U && tok[0] == "calibrate") {
      std::cout << "RobotArm::calibrate() -> ";
      std::cout.flush();
      const bool ok = arm.calibrate();
      std::cout << (ok ? "成功\n" : "失败\n");
      continue;
    }

    std::cerr << "未知指令；仅支持 calibrate / help / q（见启动时帮助）\n";
  }

  return 0;
}

int run(const std::shared_ptr<rclcpp::Node> & node)
{
  std::string missing;
  if (!params_ok(*node, missing)) {
    std::cerr
      << "缺少 ROS 参数 \"" << missing
      << "\"；本示例需 RobotArm 完整键（见源码 kRequiredParams）。\n"
      << "例如: ros2 run scara_arm test2 --ros-args --params-file <path/to/scara_arm.yaml>\n";
    return 2;
  }

  std::string can;
  std::int32_t margin = 0;
  int id_z = 0;
  int id_j1 = 0;
  int id_j2 = 0;
  int tz = 0;
  int tj1 = 0;
  int tj2 = 0;
  int z_stall = 0;
  int j1_stall = 0;
  int j2_stall = 0;
  int ps_z = 0;
  int ps_j1 = 0;
  int ps_j2 = 0;
  int pa_z = 0;
  int pa_j1 = 0;
  int pa_j2 = 0;
  int bump_z = 0;
  int bump_j1 = 0;
  int bump_j2 = 0;
  double span_z = 0.0;
  double span_joint1 = 0.0;
  double span_joint2 = 0.0;

  (void)node->get_parameter("can_interface", can);
  (void)node->get_parameter("limit_margin_units", margin);
  (void)node->get_parameter("motor_z_id", id_z);
  (void)node->get_parameter("motor_j1_id", id_j1);
  (void)node->get_parameter("motor_j2_id", id_j2);
  (void)node->get_parameter("torque_z_ma", tz);
  (void)node->get_parameter("torque_j1_ma", tj1);
  (void)node->get_parameter("torque_j2_ma", tj2);
  (void)node->get_parameter("stall_current_z_ma", z_stall);
  (void)node->get_parameter("stall_current_j1_ma", j1_stall);
  (void)node->get_parameter("stall_current_j2_ma", j2_stall);
  (void)node->get_parameter("position_speed_rpm_z", ps_z);
  (void)node->get_parameter("position_speed_rpm_j1", ps_j1);
  (void)node->get_parameter("position_speed_rpm_j2", ps_j2);
  (void)node->get_parameter("position_accel_z", pa_z);
  (void)node->get_parameter("position_accel_j1", pa_j1);
  (void)node->get_parameter("position_accel_j2", pa_j2);
  (void)node->get_parameter("bump_speed_rpm_z", bump_z);
  (void)node->get_parameter("bump_speed_rpm_j1", bump_j1);
  (void)node->get_parameter("bump_speed_rpm_j2", bump_j2);
  (void)node->get_parameter("span_z", span_z);
  (void)node->get_parameter("span_joint1", span_joint1);
  (void)node->get_parameter("span_joint2", span_joint2);

  const auto uz = static_cast<std::uint8_t>(id_z);
  const auto uj1 = static_cast<std::uint8_t>(id_j1);
  const auto uj2 = static_cast<std::uint8_t>(id_j2);

  if (uz == 0U || uj1 == 0U || uj2 == 0U) {
    std::cerr << "motor_z_id / motor_j1_id / motor_j2_id 均不可为 0\n";
    return 2;
  }

  scara_arm::BumpCfg bump = load_bump_cfg(*node);

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

  std::unique_ptr<scara_arm::RobotArm> arm;

  try {
    arm = std::make_unique<scara_arm::RobotArm>(
      can, uz, uj1, uj2, bump, margin, span_z, span_joint1, span_joint2,
      static_cast<std::uint16_t>(tz), static_cast<std::uint16_t>(tj1),
      static_cast<std::uint16_t>(tj2), static_cast<std::uint16_t>(z_stall),
      static_cast<std::uint16_t>(j1_stall), static_cast<std::uint16_t>(j2_stall), clamp_u16(ps_z),
      clamp_u16(ps_j1), clamp_u16(ps_j2), clamp_u8(pa_z), clamp_u8(pa_j1), clamp_u8(pa_j2),
      clamp_u16(bump_z), clamp_u16(bump_j1), clamp_u16(bump_j2));
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }

  return interactive_session(*arm);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("scara_arm_test2", options);
  const int rc = run(node);
  rclcpp::shutdown();
  return rc;
}
