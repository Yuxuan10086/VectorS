#include "robot_platform/platform.hpp"
#include "chassis/diff_drive_chassis.hpp"
#include "robot_interfaces/msg/motor_state.hpp"
#include "robot_interfaces/srv/set_drive_mode.hpp"
#include "robot_interfaces/srv/get_motor_state.hpp"
#include "robot_interfaces/srv/get_motor_speed_loop_pid.hpp"
#include "robot_interfaces/srv/set_motor_speed_loop_pid.hpp"
#include "robot_interfaces/action/chassis_move.hpp"
#include "robot_interfaces/action/chassis_span.hpp"
#include "robot_interfaces/action/arm_calibrate.hpp"
#include "robot_driver/pd42_motor.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
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

robot_interfaces::msg::MotorState fill_motor_state_msg(
  rclcpp::Node & node,
  std::uint8_t motor_id,
  robot_driver::Pd42Motor * motor,
  const std::optional<robot_driver::Pd42SystemParameters> & params)
{
  robot_interfaces::msg::MotorState out;
  out.motor_id = motor_id;
  out.stamp = node.get_clock()->now();
  if (!motor || !params) {
    out.success = false;
    out.message = motor ? motor->error_hint() : "unknown motor_id";
    out.error_code = motor ? motor->error_code() : 0U;
    out.error_hint = out.message;
    return out;
  }

  out.success = true;
  out.message = "ok";
  const auto & p = *params;
  out.bus_voltage_v = p.bus_voltage_v;
  out.phase_current_ma = p.phase_current_ma;
  out.flux_mwb = p.flux_mwb;
  out.phase_resistance_ohm = p.phase_resistance_ohm;
  out.phase_inductance_mh = p.phase_inductance_mh;
  out.rpm = p.rpm;
  out.target_position = p.target_position;
  out.position = p.position;
  out.position_error = p.position_error;
  out.pulse_count = p.pulse_count;
  out.enabled = p.enabled;
  out.arrived = p.arrived;
  out.stalled = p.stalled;
  out.addr_mode = p.addr_mode;
  out.error_code = motor->error_code();
  out.error_hint = motor->error_hint();
  return out;
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
  if (!node->has_parameter("imu_topic")) {
    node->declare_parameter<std::string>("imu_topic", "/imu/data_raw");
  }
  std::string chassis_topic;
  std::string arm_topic;
  std::vector<std::string> arm_joint_names;
  std::string imu_topic;
  node->get_parameter("chassis_cmd_vel_topic", chassis_topic);
  node->get_parameter("arm_joint_command_topic", arm_topic);
  node->get_parameter("arm_joint_names", arm_joint_names);
  node->get_parameter("imu_topic", imu_topic);

  // map -> base_link 位姿状态（仅由 move/span 命令更新位置；姿态始终由 IMU 原始 yaw - offset 确定）
  double map_x = 0.0;
  double map_y = 0.0;
  bool imu_initialized = false;

  robot_platform::Platform platform(*node);
  if (!platform.open()) {
    rclcpp::shutdown();
    return 1;
  }

  std::unordered_map<std::uint8_t, robot_driver::Pd42Motor *> motors_by_id;
  {
    const auto & cp = platform.chassis().params();
    motors_by_id[cp.left_id] = &platform.chassis().left_motor();
    motors_by_id[cp.right_id] = &platform.chassis().right_motor();
    const auto & ap = platform.arm().params();
    motors_by_id[ap.z.motor_id] = &platform.arm().motor_z();
    motors_by_id[ap.j1.motor_id] = &platform.arm().motor_j1();
    motors_by_id[ap.j2.motor_id] = &platform.arm().motor_j2();
  }

  auto lookup_motor = [&motors_by_id](std::uint8_t id) -> robot_driver::Pd42Motor * {
    const auto it = motors_by_id.find(id);
    return (it != motors_by_id.end()) ? it->second : nullptr;
  };

  auto get_state_srv = node->create_service<robot_interfaces::srv::GetMotorState>(
    "/motor/get_state",
    [&node, lookup_motor](const std::shared_ptr<robot_interfaces::srv::GetMotorState::Request> req,
                          std::shared_ptr<robot_interfaces::srv::GetMotorState::Response> res) {
      auto * motor = lookup_motor(req->motor_id);
      if (!motor) {
        res->state.motor_id = req->motor_id;
        res->state.stamp = node->get_clock()->now();
        res->state.success = false;
        res->state.message = "unknown motor_id=" + std::to_string(req->motor_id);
        res->state.error_hint = res->state.message;
        return;
      }
      res->state = fill_motor_state_msg(*node, req->motor_id, motor, motor->read_system_parameters());
    });

  auto get_pid_srv = node->create_service<robot_interfaces::srv::GetMotorSpeedLoopPid>(
    "/motor/get_speed_loop_pid",
    [&node, lookup_motor](const std::shared_ptr<robot_interfaces::srv::GetMotorSpeedLoopPid::Request> req,
                          std::shared_ptr<robot_interfaces::srv::GetMotorSpeedLoopPid::Response> res) {
      auto * motor = lookup_motor(req->motor_id);
      if (!motor) {
        res->success = false;
        res->message = "unknown motor_id=" + std::to_string(req->motor_id);
        return;
      }
      const auto pid = motor->read_speed_loop_pid();
      if (!pid) {
        res->success = false;
        res->message = motor->error_hint();
        return;
      }
      res->success = true;
      res->message = "ok";
      res->p = pid->p;
      res->i = pid->i;
      res->d = pid->d;
    });

  auto set_pid_srv = node->create_service<robot_interfaces::srv::SetMotorSpeedLoopPid>(
    "/motor/set_speed_loop_pid",
    [&node, lookup_motor](const std::shared_ptr<robot_interfaces::srv::SetMotorSpeedLoopPid::Request> req,
                          std::shared_ptr<robot_interfaces::srv::SetMotorSpeedLoopPid::Response> res) {
      auto * motor = lookup_motor(req->motor_id);
      if (!motor) {
        res->success = false;
        res->message = "unknown motor_id=" + std::to_string(req->motor_id);
        return;
      }
      if (!motor->set_speed_loop_pid(req->p, req->i, req->d)) {
        res->success = false;
        res->message = motor->error_hint();
        return;
      }
      if (!motor->save_parameters()) {
        res->success = false;
        res->message = std::string("PID 已写入但掉电保存失败: ") + motor->error_hint();
        return;
      }
      const auto pid = motor->read_speed_loop_pid();
      if (!pid) {
        res->success = false;
        res->message = motor->error_hint();
        return;
      }
      res->success = true;
      res->message = "ok";
      res->p = pid->p;
      res->i = pid->i;
      res->d = pid->d;
    });

  // 不再支持启动自动标定。请通过 /arm/calibrate Action 手动触发（带进度反馈）

  auto chassis_sub = node->create_subscription<geometry_msgs::msg::Twist>(
    chassis_topic, rclcpp::QoS(10),
    [&platform, node](const geometry_msgs::msg::Twist::SharedPtr msg) {
      if (!msg || !twist_finite(*msg)) {
        RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000,
          "忽略非法 Twist（需 finite linear.x / angular.z）");
        return;
      }
      const bool ok = platform.chassis().set_twist(msg->linear.x, msg->angular.z);
      if (!ok) {
        RCLCPP_WARN(
          node->get_logger(),
          "SDK 调用失败: DiffDriveChassis::set_twist(linear_x=%.6f, omega=%.6f)",
          msg->linear.x, msg->angular.z);
        RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000,
          "底盘 set_twist 失败");
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
      }
    });

  // IMU 订阅：将最新 Yaw 同时注入 Platform（内部转发给底盘 SDK 供 move/span 闭环 + 维护 offset）
  auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::QoS(20),
    [&platform, node, &imu_initialized](const sensor_msgs::msg::Imu::SharedPtr msg) {
      if (!msg) return;
      const auto & q = msg->orientation;
      // 四元数转 Yaw（Z 轴航向，标准公式）
      double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      platform.update_imu_yaw(yaw);

      if (!imu_initialized) {
        platform.reset_imu_zero();
        imu_initialized = true;
        RCLCPP_INFO(node->get_logger(), "IMU 首次数据到达，已自动重置零点，初始位姿与 map 重合（x=0,y=0,yaw=0）");
      }
    });

  // TF 广播器：7Hz 发布 map -> base_link
  // 姿态 = 当前 IMU yaw - offset；位置仅在 move/span 成功后按命令增量更新
  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  auto tf_timer = node->create_wall_timer(
    std::chrono::milliseconds(1000 / 7),
    [tf_broadcaster, node, &platform, &map_x, &map_y]() {
      const double yaw = platform.get_corrected_imu_yaw();
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = node->now();
      t.header.frame_id = "map";
      t.child_frame_id = "base_link";
      t.transform.translation.x = map_x;
      t.transform.translation.y = map_y;
      t.transform.translation.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      tf_broadcaster->sendTransform(t);
    });

  // 重置 IMU 零点服务（调用后把 offset 设为当前偏航角，使当前朝向在 map 中 yaw=0，位置保持）
  auto reset_imu_zero_srv = node->create_service<std_srvs::srv::Trigger>(
    "/imu/reset_zero",
    [&platform, node](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      platform.reset_imu_zero();
      res->success = true;
      res->message = "IMU 零点已重置（offset=当前 yaw），map->base_link 的 yaw 归零，位置保持不变";
      RCLCPP_INFO(node->get_logger(), "已处理 /imu/reset_zero 请求");
    });

  // 模式切换服务：0=TWIST（带看门狗的速度模式），1=MOVE（阻塞移动模式）
  auto set_mode_srv = node->create_service<robot_interfaces::srv::SetDriveMode>(
    "/chassis/set_mode",
    [&platform, node](const std::shared_ptr<robot_interfaces::srv::SetDriveMode::Request> req,
                      std::shared_ptr<robot_interfaces::srv::SetDriveMode::Response> res) {
      const std::string mode_str = (req->mode == 0) ? "TWIST(0)" : (req->mode == 1 ? "MOVE(1)" : std::to_string(req->mode));
      RCLCPP_INFO(node->get_logger(), "收到模式切换请求: mode=%s", mode_str.c_str());

      chassis::DriveMode target;
      if (req->mode == robot_interfaces::srv::SetDriveMode::Request::TWIST) {
        target = chassis::DriveMode::kTwist;
      } else if (req->mode == robot_interfaces::srv::SetDriveMode::Request::MOVE) {
        target = chassis::DriveMode::kMove;
      } else {
        res->success = false;
        res->message = "无效的 mode 值（0=TWIST 或 1=MOVE）";
        RCLCPP_ERROR(node->get_logger(), "模式切换失败: 无效 mode=%s", mode_str.c_str());
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
        res->message = "set_mode 调用失败（电机可能未就绪或参数无效，请查看电机 error_code）";
        RCLCPP_ERROR(node->get_logger(), "底盘模式切换失败: mode=%s, 错误=%s", mode_str.c_str(), res->message.c_str());
      }
    });

  // ChassisMove/Span 使用可重入回调组 + 工作线程，避免 execute 阻塞导致 cancel 无法执行
  auto chassis_action_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

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
    [&platform, node](const std::shared_ptr<GoalHandleMove> /*goal_handle*/) {
      platform.chassis().cancel_motion();
      RCLCPP_INFO(node->get_logger(), "ChassisMove 收到取消请求，已中止底盘运动");
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    // accepted callback：工作线程执行 move，本回调立即返回以便 cancel 可并发执行
    [&platform, node, &map_x, &map_y](const std::shared_ptr<GoalHandleMove> goal_handle) {
      const auto goal = goal_handle->get_goal();
      const double target_m = std::abs(goal->distance_m);

      auto feedback = std::make_shared<ChassisMove::Feedback>();
      feedback->distance_remaining_m = target_m;
      feedback->left_encoder_pos = 0;
      feedback->right_encoder_pos = 0;
      goal_handle->publish_feedback(feedback);

      std::thread worker([&platform, node, goal_handle, goal, &map_x, &map_y]() {
        const double start_yaw = platform.get_corrected_imu_yaw();
        const bool ok = platform.chassis().move(
          goal->distance_m, goal->speed_mps,
          [&platform, goal_handle](const chassis::MoveProgress & p) {
            if (goal_handle->is_canceling()) {
              platform.chassis().cancel_motion();
            }
            auto fb = std::make_shared<ChassisMove::Feedback>();
            fb->distance_remaining_m = p.distance_remaining_m;
            fb->left_encoder_pos = p.left_encoder_pos;
            fb->right_encoder_pos = p.right_encoder_pos;
            goal_handle->publish_feedback(fb);
          });

        if (ok) {
          map_x += goal->distance_m * std::cos(start_yaw);
          map_y += goal->distance_m * std::sin(start_yaw);
        }

        auto final_fb = std::make_shared<ChassisMove::Feedback>();
        final_fb->distance_remaining_m = 0.0;
        final_fb->left_encoder_pos = 0;
        final_fb->right_encoder_pos = 0;
        goal_handle->publish_feedback(final_fb);

        auto result = std::make_shared<ChassisMove::Result>();
        result->success = ok;
        // canceled() 仅当 goal 已进入 CANCELING；motion_aborted 单独为真时须 abort，否则会崩
        if (goal_handle->is_canceling()) {
          goal_handle->canceled(result);
          RCLCPP_INFO(node->get_logger(), "ChassisMove 已取消");
        } else if (ok) {
          goal_handle->succeed(result);
          RCLCPP_INFO(node->get_logger(), "ChassisMove 成功，更新 map 位置 -> (%.3f, %.3f)", map_x, map_y);
        } else {
          (void)platform.chassis().motion_aborted();
          goal_handle->abort(result);
        }
      });
      worker.detach();
    },
    rcl_action_server_get_default_options(),
    chassis_action_cb_group
  );

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
    [&platform, node](const std::shared_ptr<GoalHandleSpan> /*goal_handle*/) {
      platform.chassis().cancel_motion();
      RCLCPP_INFO(node->get_logger(), "ChassisSpan 收到取消请求，已中止底盘运动");
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    // accepted callback
    [&platform, node](const std::shared_ptr<GoalHandleSpan> goal_handle) {
      const auto goal = goal_handle->get_goal();

      auto feedback = std::make_shared<ChassisSpan::Feedback>();
      feedback->angle_remaining_rad = goal->angle_rad;
      goal_handle->publish_feedback(feedback);

      std::thread worker([&platform, node, goal_handle, goal]() {
        const bool ok = platform.chassis().span(goal->angle_rad, goal->omega_radps);

        auto final_fb = std::make_shared<ChassisSpan::Feedback>();
        final_fb->angle_remaining_rad = 0.0;
        goal_handle->publish_feedback(final_fb);

        auto result = std::make_shared<ChassisSpan::Result>();
        result->success = ok;
        if (goal_handle->is_canceling()) {
          goal_handle->canceled(result);
          RCLCPP_INFO(node->get_logger(), "ChassisSpan 已取消");
        } else if (ok) {
          goal_handle->succeed(result);
          RCLCPP_INFO(node->get_logger(), "ChassisSpan 成功，航向已由 IMU 实时更新（map->base_link 将在 7Hz 刷新）");
        } else {
          (void)platform.chassis().motion_aborted();
          goal_handle->abort(result);
        }
      });
      worker.detach();
    },
    rcl_action_server_get_default_options(),
    chassis_action_cb_group
  );

  // ArmCalibrate Action：触发机械臂整臂标定（Z->J2->J1），带实时进度 Feedback（每个轴完成时推送一次）
  using ArmCalibrate = robot_interfaces::action::ArmCalibrate;
  using GoalHandleCalib = rclcpp_action::ServerGoalHandle<ArmCalibrate>;
  auto arm_calibrate_server = rclcpp_action::create_server<ArmCalibrate>(
    node,
    "/arm/calibrate",
    // goal callback - 总是接受（标定可随时触发）
    [node](const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const ArmCalibrate::Goal> /*goal*/) {
      RCLCPP_INFO(node->get_logger(), "收到 /arm/calibrate 目标请求");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    // cancel callback
    [node](const std::shared_ptr<GoalHandleCalib> /*goal_handle*/) {
      RCLCPP_WARN(node->get_logger(), "/arm/calibrate 收到取消请求（当前实现会继续完成标定）");
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    // execute callback（使用独立线程避免阻塞 executor）
    [&platform, node](const std::shared_ptr<GoalHandleCalib> goal_handle) {
      auto result = std::make_shared<ArmCalibrate::Result>();

      // 初始 feedback
      auto fb = std::make_shared<ArmCalibrate::Feedback>();
      fb->current_step = "开始整臂标定...";
      fb->progress = 0.0;
      fb->z_done = false;
      fb->j1_done = false;
      fb->j2_done = false;
      goal_handle->publish_feedback(fb);

      std::atomic<bool> success{false};
      std::atomic<int> completed_axes{0};

      // 启动工作线程执行实际标定 + 实时反馈
      std::thread worker([&platform, node, goal_handle, &success, &completed_axes]() {
        auto on_axis = [goal_handle, node, &completed_axes](const std::string & axis) {
          auto f = std::make_shared<ArmCalibrate::Feedback>();
          f->completed_axis = axis;
          const int done = completed_axes.fetch_add(1) + 1;  // 完成数量
          f->progress = done / 3.0;
          f->z_done = (done >= 1);
          f->j2_done = (done >= 2);
          f->j1_done = (done >= 3);
          if (axis == "z") f->current_step = "Z 轴标定完成";
          else if (axis == "j2") f->current_step = "J2 轴标定完成";
          else if (axis == "j1") f->current_step = "J1 轴标定完成";
          goal_handle->publish_feedback(f);
          RCLCPP_INFO(node->get_logger(), "机械臂标定进度: %s 完成 (%.0f%%)", axis.c_str(), f->progress * 100.0);
        };

        const bool ok = platform.arm().calibrate(on_axis);
        success.store(ok);

        auto final_fb = std::make_shared<ArmCalibrate::Feedback>();
        final_fb->progress = 1.0;
        final_fb->z_done = true;
        final_fb->j1_done = true;
        final_fb->j2_done = true;
        final_fb->current_step = ok ? "标定成功完成" : "标定失败";
        goal_handle->publish_feedback(final_fb);

        auto res = std::make_shared<ArmCalibrate::Result>();
        res->success = ok;
        res->message = ok ? "机械臂标定成功，所有轴已完成碰停与限位设置" :
                            "机械臂标定失败（详见日志中的电机 error_code）";
        if (ok) {
          goal_handle->succeed(res);
        } else {
          goal_handle->abort(res);
        }
      });

      worker.detach();  // 让线程后台运行，execute 立即返回
    }
  );

  RCLCPP_INFO(
    node->get_logger(),
    "robot_platform 运行中：底盘 Twist [%s]，机械臂 JointState [%s]，IMU [%s]，7Hz map->base_link TF，"
    "服务: /chassis/set_mode /imu/reset_zero /motor/get_state /motor/get_speed_loop_pid /motor/set_speed_loop_pid，"
    "Actions: /chassis/move /chassis/span /arm/calibrate",
    chassis_topic.c_str(), arm_topic.c_str(), imu_topic.c_str());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  platform.close();
  rclcpp::shutdown();
  return 0;
}
