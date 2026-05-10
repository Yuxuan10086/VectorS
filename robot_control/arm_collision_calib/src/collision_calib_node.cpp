// 节点：仅负责读参数与调用 RobotArm（库内无 ROS）
// 参数必须由外部提供（如 --params-file），不设代码内默认值
#include "arm_collision_calib/robot_arm.hpp"

#include <rclcpp/rclcpp.hpp>

#include <array>
#include <exception>
#include <iostream>
#include <memory>
#include <string>

namespace
{

constexpr std::array<const char *, 12> kRequiredParams{
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
};

/** 要求参数均来自 override（如 YAML）；缺少任一项则失败 */
bool params_ok(const rclcpp::Node & node, std::string & first_missing)
{
  for (const char * name : kRequiredParams) {
    if (!node.has_parameter(name)) {
      first_missing = name;
      return false;
    }
  }
  return true;
}

arm_collision_calib::BumpCfg load_bump_cfg(const rclcpp::Node & node)
{
  arm_collision_calib::BumpCfg c;
  (void)node.get_parameter("speed_stop_threshold_rpm", c.rpm_zero_th);
  (void)node.get_parameter("stall_stable_ms", c.stable_ms);
  (void)node.get_parameter("poll_period_ms", c.poll_ms);
  (void)node.get_parameter("seek_timeout_s", c.timeout_s);
  return c;
}

int run(const std::shared_ptr<rclcpp::Node> & node)
{
  std::string missing;
  if (!params_ok(*node, missing)) {
    std::cerr
      << "缺少 ROS 参数 \"" << missing
      << "\"；请加载配置文件，例如:\n  ros2 run arm_collision_calib collision_calib_node "
      << "--ros-args --params-file <path/to/arm_collision.yaml>\n";
    return 2;
  }

  std::string can;
  std::int32_t margin = 0;
  int id_z = 0;
  int id_j1 = 0;
  int id_j2 = 0;
  int tj1 = 0;
  int tj2 = 0;

  (void)node->get_parameter("can_interface", can);
  (void)node->get_parameter("limit_margin_units", margin);
  (void)node->get_parameter("motor_z_id", id_z);
  (void)node->get_parameter("motor_j1_id", id_j1);
  (void)node->get_parameter("motor_j2_id", id_j2);
  (void)node->get_parameter("torque_j1_ma", tj1);
  (void)node->get_parameter("torque_j2_ma", tj2);

  const auto uz = static_cast<std::uint8_t>(id_z);
  const auto uj1 = static_cast<std::uint8_t>(id_j1);
  const auto uj2 = static_cast<std::uint8_t>(id_j2);

  if (uz == 0 || uj1 == 0 || uj2 == 0) {
    std::cerr << "motor id invalid\n";
    return 2;
  }

  arm_collision_calib::BumpCfg bump = load_bump_cfg(*node);

  std::unique_ptr<arm_collision_calib::RobotArm> arm;
  try {
    arm = std::make_unique<arm_collision_calib::RobotArm>(
      can, uz, uj1, uj2, bump, margin,
      static_cast<std::uint16_t>(tj1), static_cast<std::uint16_t>(tj2));
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }

  if (!arm->calibrate()) {
    std::cerr << "calibrate failed\n";
    return 1;
  }
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("collision_calib", options);
  const int rc = run(node);
  rclcpp::shutdown();
  return rc;
}
