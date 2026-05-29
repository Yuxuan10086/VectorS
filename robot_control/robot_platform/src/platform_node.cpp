#include "robot_platform/platform.hpp"
#include "robot_interfaces/srv/set_drive_mode.hpp"
#include "robot_interfaces/action/chassis_move.hpp"
#include "robot_interfaces/action/chassis_span.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace
{

bool twist_finite(const geometry_msgs::msg::Twist & t)
{
  return std::isfinite(t.linear.x) && std::isfinite(t.angular.z);
}

std::optional<std::array<double, 3>> spans_from_joint_state(
  const sensor_msgs::msg::JointState & msg,
  const std::vector<std::string> & joint_names)
{
  if (joint_names.size() != 3U) {
    return std::nullopt;
  }

  const bool names_empty = msg.name.empty();
  if (!names_empty) {
    std::array<double, 3> spans{};
    for (std::size_t i = 0; i < 3U; ++i) {
      auto it = std::find(msg.name.begin(), msg.name.end(), joint_names[i]);
      if (it == msg.name.end()) {
        return std::nullopt;
      }
      const auto idx = static_cast<std::size_t>(std::distance(msg.name.begin(), it));
      if (idx >= msg.position.size()) {
        return std::nullopt;
      }
      spans[i] = msg.position[idx];
    }
    return spans;
  }

  if (msg.position.size() >= 3U) {
    return std::array<double, 3>{msg.position[0], msg.position[1], msg.position[2]};
  }
  return std::nullopt;
}

bool range_ready(double lo, double hi)
{
  return hi > lo;
}

std::string arm_set_position_failure_detail(
  const scara_arm::RobotArm & arm, double z, double j1, double j2)
{
  const bool z_ready = range_ready(arm.reachable_span_min_z, arm.reachable_span_max_z);
  const bool j1_ready = range_ready(arm.reachable_span_min_j1, arm.reachable_span_max_j1);
  const bool j2_ready = range_ready(arm.reachable_span_min_j2, arm.reachable_span_max_j2);

  const bool z_oob = z_ready && (z < arm.reachable_span_min_z || z > arm.reachable_span_max_z);
  const bool j1_oob = j1_ready && (j1 < arm.reachable_span_min_j1 || j1 > arm.reachable_span_max_j1);
  const bool j2_oob = j2_ready && (j2 < arm.reachable_span_min_j2 || j2 > arm.reachable_span_max_j2);

  std::ostringstream oss;
  oss << "机械臂 set_position 失败: req[z=" << z << ", j1=" << j1 << ", j2=" << j2 << "], "
      << "reachable[z=(" << arm.reachable_span_min_z << "," << arm.reachable_span_max_z << "), "
      << "j1=(" << arm.reachable_span_min_j1 << "," << arm.reachable_span_max_j1 << "), "
      << "j2=(" << arm.reachable_span_min_j2 << "," << arm.reachable_span_max_j2 << ")], "
      << "range_ready[z=" << (z_ready ? "Y" : "N")
      << ", j1=" << (j1_ready ? "Y" : "N")
      << ", j2=" << (j2_ready ? "Y" : "N") << "], "
      << "out_of_range[z=" << (z_oob ? "Y" : "N")
      << ", j1=" << (j1_oob ? "Y" : "N")
      << ", j2=" << (j2_oob ? "Y" : "N") << "]";
  return oss.str();
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("robot_platform", options);

  if (!node->has_parameter("chassis_cmd_vel_topic")) {
    node->declare_parameter<std::string>("chassis_cmd_vel_topic", "/chassis/cmd_vel");
  }
  if (!node->has_parameter("arm_joint_command_topic")) {
    node->declare_parameter<std::string>("arm_joint_command_topic", "/arm/joint_command");
  }
  if (!node->has_parameter("arm_joint_names")) {
    node->declare_parameter<std::vector<std::string>>("arm_joint_names", {"z", "j1", "j2"});
  }
  if (!node->has_parameter("arm_auto_calibrate")) {
    node->declare_parameter<bool>("arm_auto_calibrate", false);
  }
  if (!node->has_parameter("imu_topic")) {
    node->declare_parameter<std::string>("imu_topic", "/imu/data_raw");
  }
  std::string chassis_topic;
  std::string arm_topic;
  std::vector<std::string> arm_joint_names;
  bool arm_auto_calibrate = false;
  std::string imu_topic;
  node->get_parameter("chassis_cmd_vel_topic", chassis_topic);
  node->get_parameter("arm_joint_command_topic", arm_topic);
  node->get_parameter("arm_joint_names", arm_joint_names);
  node->get_parameter("arm_auto_calibrate", arm_auto_calibrate);
  node->get_parameter("imu_topic", imu_topic);

  robot_platform::Platform platform(*node);
  if (!platform.open()) {
    rclcpp::shutdown();
    return 1;
  }
  if (arm_auto_calibrate) {
    RCLCPP_INFO(node->get_logger(), "arm_auto_calibrate=true，开始机械臂标定");
    if (!platform.arm().calibrate()) {
      RCLCPP_ERROR(node->get_logger(), "机械臂标定失败，退出");
      platform.close();
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "机械臂标定完成");
  } else {
    RCLCPP_WARN(
      node->get_logger(),
      "arm_auto_calibrate=false：机械臂未标定时 set_position 可能失败");
  }

  auto chassis_sub = node->create_subscription<geometry_msgs::msg::Twist>(
    chassis_topic, rclcpp::QoS(10),
    [&platform, node](const geometry_msgs::msg::Twist::SharedPtr msg) {
      if (!msg || !twist_finite(*msg)) {
        RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000,
          "忽略非法 Twist（需 finite linear.x / angular.z）");
        return;
      }
      RCLCPP_INFO(
        node->get_logger(),
        "收到底盘控制语义: linear.x=%.6f m/s, angular.z=%.6f rad/s -> 调用 SDK: DiffDriveChassis::set_twist(linear_x, omega)",
        msg->linear.x, msg->angular.z);
      const bool ok = platform.chassis().set_twist(msg->linear.x, msg->angular.z);
      if (!ok) {
        RCLCPP_WARN(
          node->get_logger(),
          "SDK 调用失败: DiffDriveChassis::set_twist(linear_x=%.6f, omega=%.6f)",
          msg->linear.x, msg->angular.z);
        RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000,
          "底盘 set_twist 失败");
      } else {
        RCLCPP_INFO(
          node->get_logger(),
          "SDK 调用成功: DiffDriveChassis::set_twist(linear_x=%.6f, omega=%.6f)",
          msg->linear.x, msg->angular.z);
      }
    });

  auto arm_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    arm_topic, rclcpp::QoS(10),
    [&platform, node, arm_joint_names](const sensor_msgs::msg::JointState::SharedPtr msg) {
      if (!msg) {
        return;
      }
      const auto spans = spans_from_joint_state(*msg, arm_joint_names);
      if (!spans) {
        RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000,
          "JointState 无法解析为 z/j1/j2 position（检查 name 或 position 长度）");
        return;
      }
      RCLCPP_INFO(
        node->get_logger(),
        "收到机械臂控制语义: z=%.6f, j1=%.6f, j2=%.6f -> 调用 SDK: RobotArm::set_position(span_z, span_j1, span_j2)",
        (*spans)[0], (*spans)[1], (*spans)[2]);
      const bool ok = platform.arm().set_position((*spans)[0], (*spans)[1], (*spans)[2]);
      if (!ok) {
        RCLCPP_WARN(
          node->get_logger(),
          "SDK 调用失败: RobotArm::set_position(z=%.6f, j1=%.6f, j2=%.6f)",
          (*spans)[0], (*spans)[1], (*spans)[2]);
        const auto detail = arm_set_position_failure_detail(
          platform.arm(), (*spans)[0], (*spans)[1], (*spans)[2]);
        RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000,
          "%s", detail.c_str());
      } else {
        RCLCPP_INFO(
          node->get_logger(),
          "SDK 调用成功: RobotArm::set_position(z=%.6f, j1=%.6f, j2=%.6f)",
          (*spans)[0], (*spans)[1], (*spans)[2]);
      }
    });

  // IMU 订阅：将最新 Yaw 注入到底盘 SDK（供 kMove 模式的 move/span 使用）
  auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::QoS(20),
    [&platform](const sensor_msgs::msg::Imu::SharedPtr msg) {
      if (!msg) return;
      const auto & q = msg->orientation;
      // 四元数转 Yaw（Z 轴航向，标准公式）
      double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      platform.chassis().update_imu_yaw(yaw);
    });

  // 模式切换服务：0=TWIST（带看门狗的速度模式），1=MOVE（阻塞移动模式）
  auto set_mode_srv = node->create_service<robot_interfaces::srv::SetDriveMode>(
    "/chassis/set_mode",
    [&platform, node](const std::shared_ptr<robot_interfaces::srv::SetDriveMode::Request> req,
                      std::shared_ptr<robot_interfaces::srv::SetDriveMode::Response> res) {
      chassis::DriveMode target;
      if (req->mode == robot_interfaces::srv::SetDriveMode::Request::TWIST) {
        target = chassis::DriveMode::kTwist;
      } else if (req->mode == robot_interfaces::srv::SetDriveMode::Request::MOVE) {
        target = chassis::DriveMode::kMove;
      } else {
        res->success = false;
        res->message = "无效的 mode 值（0=TWIST 或 1=MOVE）";
        return;
      }

      const bool ok = platform.chassis().set_mode(target);
      res->success = ok;
      if (ok) {
        res->message = (target == chassis::DriveMode::kTwist)
                         ? "已切换到 kTwist 模式（带 300ms 看门狗保护）"
                         : "已切换到 kMove 模式（阻塞移动，无看门狗）";
        RCLCPP_INFO(node->get_logger(), "底盘模式切换成功: %s", res->message.c_str());
      } else {
        res->message = "set_mode 调用失败（电机可能未就绪或参数无效）";
        RCLCPP_WARN(node->get_logger(), "底盘模式切换失败");
      }
    });

  // ChassisMove Action：仅在 kMove 模式下接受，Feedback 使用与 Goal 同单位的剩余距离
  using ChassisMove = robot_interfaces::action::ChassisMove;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<ChassisMove>;
  auto chassis_move_server = rclcpp_action::create_server<ChassisMove>(
    node,
    "/chassis/move",
    // goal callback
    [&platform, node](const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const ChassisMove::Goal> goal) {
      if (!goal) {
        return rclcpp_action::GoalResponse::REJECT;
      }
      if (platform.chassis().mode() != chassis::DriveMode::kMove) {
        RCLCPP_WARN(node->get_logger(), "ChassisMove goal 被拒绝：当前不是 kMove 模式");
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    // cancel callback
    [](const std::shared_ptr<GoalHandleMove> /*goal_handle*/) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    // execute callback
    [&platform, node](const std::shared_ptr<GoalHandleMove> goal_handle) {
      const auto goal = goal_handle->get_goal();
      auto result = std::make_shared<ChassisMove::Result>();

      // 初始 feedback：剩余距离 = 目标距离
      auto feedback = std::make_shared<ChassisMove::Feedback>();
      feedback->distance_remaining_m = goal->distance_m;
      goal_handle->publish_feedback(feedback);

      const bool ok = platform.chassis().move(goal->distance_m, goal->speed_mps);

      // stub 实现下直接置 0（真实实现应在运动循环中逐步更新）
      feedback->distance_remaining_m = 0.0;
      goal_handle->publish_feedback(feedback);

      result->success = ok;
      if (ok) {
        goal_handle->succeed(result);
      } else {
        goal_handle->abort(result);
      }
    }
  );  // close execute lambda + close create_server call

  // ChassisSpan Action：仅在 kMove 模式下接受，Feedback 使用与 Goal 同单位的剩余角度
  using ChassisSpan = robot_interfaces::action::ChassisSpan;
  using GoalHandleSpan = rclcpp_action::ServerGoalHandle<ChassisSpan>;
  auto chassis_span_server = rclcpp_action::create_server<ChassisSpan>(
    node,
    "/chassis/span",
    // goal callback
    [&platform, node](const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const ChassisSpan::Goal> goal) {
      if (!goal) {
        return rclcpp_action::GoalResponse::REJECT;
      }
      if (platform.chassis().mode() != chassis::DriveMode::kMove) {
        RCLCPP_WARN(node->get_logger(), "ChassisSpan goal 被拒绝：当前不是 kMove 模式");
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    // cancel callback
    [](const std::shared_ptr<GoalHandleSpan> /*goal_handle*/) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    // execute callback
    [&platform, node](const std::shared_ptr<GoalHandleSpan> goal_handle) {
      const auto goal = goal_handle->get_goal();
      auto result = std::make_shared<ChassisSpan::Result>();

      auto feedback = std::make_shared<ChassisSpan::Feedback>();
      feedback->angle_remaining_rad = goal->angle_rad;
      goal_handle->publish_feedback(feedback);

      const bool ok = platform.chassis().span(goal->angle_rad, goal->omega_radps);

      feedback->angle_remaining_rad = 0.0;
      goal_handle->publish_feedback(feedback);

      result->success = ok;
      if (ok) {
        goal_handle->succeed(result);
      } else {
        goal_handle->abort(result);
      }
    }
  );  // close execute lambda + close create_server call

  RCLCPP_INFO(
    node->get_logger(),
    "robot_platform 运行中：底盘 Twist [%s]，机械臂 JointState [%s]，IMU [%s]，模式切换服务 /chassis/set_mode，Actions: /chassis/move /chassis/span",
    chassis_topic.c_str(), arm_topic.c_str(), imu_topic.c_str());
  rclcpp::spin(node);
  platform.close();
  rclcpp::shutdown();
  return 0;
}
