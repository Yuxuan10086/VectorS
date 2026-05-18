#include "robot_platform/platform.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("robot_platform", options);

  robot_platform::Platform platform(*node);
  if (!platform.open()) {
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "robot_platform 节点运行中（仅保持硬件打开，Ctrl+C 退出）");
  rclcpp::spin(node);
  platform.close();
  rclcpp::shutdown();
  return 0;
}
