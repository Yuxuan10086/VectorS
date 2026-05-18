#include "robot_platform/platform.hpp"

#include <rclcpp/rclcpp.hpp>

#include <stdexcept>

namespace robot_platform
{

namespace
{

std::uint16_t clamp_u16(int v)
{
  if (v <= 0) {
    return 0U;
  }
  if (v > 65535) {
    return 65535U;
  }
  return static_cast<std::uint16_t>(v);
}

std::uint8_t clamp_u8(int v)
{
  if (v <= 0) {
    return 0U;
  }
  if (v > 255) {
    return 255U;
  }
  return static_cast<std::uint8_t>(v);
}

}  // namespace

Platform::Platform(rclcpp::Node & node)
: node_(node)
{
}

Platform::~Platform()
{
  close();
}

bool Platform::load_chassis_params(chassis::DiffDriveParams & out)
{
  int left_id = 0;
  int right_id = 0;
  bool left_inv = false;
  double wheel_d = 0.0;
  double track = 0.0;
  double travel = 0.0;
  double pivot = 0.0;
  int accel = 0;

  if (!node_.get_parameter("motor_left_id", left_id) ||
    !node_.get_parameter("motor_right_id", right_id) ||
    !node_.get_parameter("left_reverse_inverted", left_inv) ||
    !node_.get_parameter("wheel_diameter_m", wheel_d) ||
    !node_.get_parameter("track_m", track) ||
    !node_.get_parameter("travel_speed_m_s", travel) ||
    !node_.get_parameter("pivot_wheel_m_s", pivot) ||
    !node_.get_parameter("drive_accel", accel)) {
    RCLCPP_ERROR(node_.get_logger(), "缺少底盘参数，请加载 chassis/config/chassis.yaml");
    return false;
  }

  out.left_id = static_cast<std::uint8_t>(left_id);
  out.right_id = static_cast<std::uint8_t>(right_id);
  out.left_reverse_inverted = left_inv;
  out.wheel_diameter_m = wheel_d;
  out.track_m = track;
  out.travel_speed_m_s = travel;
  out.pivot_wheel_m_s = pivot;
  out.accel = clamp_u8(accel);

  if (out.left_id == 0U || out.right_id == 0U) {
    RCLCPP_ERROR(node_.get_logger(), "motor_left_id / motor_right_id 不可为 0");
    return false;
  }
  return true;
}

bool Platform::create_arm()
{
  std::int32_t margin = 0;
  int id_z = 0;
  int id_j1 = 0;
  int id_j2 = 0;
  int tzu = 0;
  int tzd = 0;
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

  const char * const keys[] = {
    "limit_margin_units",
    "motor_z_id", "motor_j1_id", "motor_j2_id",
    "torque_z_up_ma", "torque_z_down_ma", "torque_j1_ma", "torque_j2_ma",
    "stall_current_z_ma", "stall_current_j1_ma", "stall_current_j2_ma",
    "position_speed_rpm_z", "position_speed_rpm_j1", "position_speed_rpm_j2",
    "position_accel_z", "position_accel_j1", "position_accel_j2",
    "bump_speed_rpm_z", "bump_speed_rpm_j1", "bump_speed_rpm_j2",
    "span_z", "span_joint1", "span_joint2",
  };
  for (const char * key : keys) {
    if (!node_.has_parameter(key)) {
      RCLCPP_ERROR(node_.get_logger(), "缺少机械臂参数 \"%s\"，请加载 scara_arm/config/scara_arm.yaml", key);
      return false;
    }
  }

  (void)node_.get_parameter("limit_margin_units", margin);
  (void)node_.get_parameter("motor_z_id", id_z);
  (void)node_.get_parameter("motor_j1_id", id_j1);
  (void)node_.get_parameter("motor_j2_id", id_j2);
  (void)node_.get_parameter("torque_z_up_ma", tzu);
  (void)node_.get_parameter("torque_z_down_ma", tzd);
  (void)node_.get_parameter("torque_j1_ma", tj1);
  (void)node_.get_parameter("torque_j2_ma", tj2);
  (void)node_.get_parameter("stall_current_z_ma", z_stall);
  (void)node_.get_parameter("stall_current_j1_ma", j1_stall);
  (void)node_.get_parameter("stall_current_j2_ma", j2_stall);
  (void)node_.get_parameter("position_speed_rpm_z", ps_z);
  (void)node_.get_parameter("position_speed_rpm_j1", ps_j1);
  (void)node_.get_parameter("position_speed_rpm_j2", ps_j2);
  (void)node_.get_parameter("position_accel_z", pa_z);
  (void)node_.get_parameter("position_accel_j1", pa_j1);
  (void)node_.get_parameter("position_accel_j2", pa_j2);
  (void)node_.get_parameter("bump_speed_rpm_z", bump_z);
  (void)node_.get_parameter("bump_speed_rpm_j1", bump_j1);
  (void)node_.get_parameter("bump_speed_rpm_j2", bump_j2);
  (void)node_.get_parameter("span_z", span_z);
  (void)node_.get_parameter("span_joint1", span_joint1);
  (void)node_.get_parameter("span_joint2", span_joint2);

  const auto uz = static_cast<std::uint8_t>(id_z);
  const auto uj1 = static_cast<std::uint8_t>(id_j1);
  const auto uj2 = static_cast<std::uint8_t>(id_j2);
  if (uz == 0U || uj1 == 0U || uj2 == 0U) {
    RCLCPP_ERROR(node_.get_logger(), "motor_z_id / motor_j1_id / motor_j2_id 均不可为 0");
    return false;
  }

  try {
    arm_ = std::make_unique<scara_arm::RobotArm>(
      can_,
      uz, uj1, uj2, margin, span_z, span_joint1, span_joint2,
      clamp_u16(tzu), clamp_u16(tzd), static_cast<std::uint16_t>(tj1),
      static_cast<std::uint16_t>(tj2),
      static_cast<std::uint16_t>(z_stall),
      static_cast<std::uint16_t>(j1_stall), static_cast<std::uint16_t>(j2_stall),
      clamp_u16(ps_z), clamp_u16(ps_j1), clamp_u16(ps_j2),
      clamp_u8(pa_z), clamp_u8(pa_j1), clamp_u8(pa_j2),
      clamp_u16(bump_z), clamp_u16(bump_j1), clamp_u16(bump_j2));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_.get_logger(), "构造 RobotArm 失败: %s", e.what());
    return false;
  }
  return true;
}

bool Platform::open()
{
  if (open_) {
    return true;
  }

  if (!can_.open()) {
    RCLCPP_ERROR(node_.get_logger(), "无法打开 CAN");
    return false;
  }

  chassis::DiffDriveParams cp{};
  if (!load_chassis_params(cp)) {
    can_.close();
    return false;
  }

  chassis_ = std::make_unique<chassis::DiffDriveChassis>(can_, cp);
  if (!chassis_->initialize()) {
    RCLCPP_ERROR(node_.get_logger(), "底盘 initialize 失败");
    chassis_.reset();
    can_.close();
    return false;
  }

  if (!create_arm()) {
    chassis_.reset();
    can_.close();
    return false;
  }

  open_ = true;
  RCLCPP_INFO(node_.get_logger(), "Platform 已打开（CAN + 底盘 + 机械臂）");
  return true;
}

void Platform::close()
{
  if (!open_) {
    return;
  }
  if (chassis_) {
    (void)chassis_->stop();
  }
  arm_.reset();
  chassis_.reset();
  if (can_.is_open()) {
    can_.close();
  }
  open_ = false;
}

robot_driver::CanInterface & Platform::can()
{
  return can_;
}

chassis::DiffDriveChassis & Platform::chassis()
{
  if (!chassis_) {
    throw std::runtime_error("Platform::chassis() 在 open() 之前调用");
  }
  return *chassis_;
}

scara_arm::RobotArm & Platform::arm()
{
  if (!arm_) {
    throw std::runtime_error("Platform::arm() 在 open() 之前调用");
  }
  return *arm_;
}

}  // namespace robot_platform
