#include "robot_platform/platform.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
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
  double wheel_d = 0.0;
  double track = 0.0;
  int accel = 0;
  int64_t encoder_pulses = 0;
  double odom_factor = 1.0;

  if (!node_.get_parameter("motor_left_id", left_id) ||
    !node_.get_parameter("motor_right_id", right_id) ||
    !node_.get_parameter("wheel_diameter_m", wheel_d) ||
    !node_.get_parameter("track_m", track) ||
    !node_.get_parameter("drive_accel", accel) ||
    !node_.get_parameter("encoder_pulses_per_rev", encoder_pulses) ||
    !node_.get_parameter("odometry_correction_factor", odom_factor)) {
    RCLCPP_ERROR(node_.get_logger(), "缺少底盘参数，请加载 chassis/config/chassis.yaml");
    return false;
  }

  out.left_id = static_cast<std::uint8_t>(left_id);
  out.right_id = static_cast<std::uint8_t>(right_id);
  out.wheel_diameter_m = wheel_d;
  out.track_m = track;
  out.accel = clamp_u8(accel);
  out.encoder_pulses_per_rev = static_cast<double>(encoder_pulses);
  out.odometry_correction_factor = odom_factor;

  if (out.left_id == 0U || out.right_id == 0U) {
    RCLCPP_ERROR(node_.get_logger(), "motor_left_id / motor_right_id 不可为 0");
    return false;
  }
  if (out.encoder_pulses_per_rev <= 0.0) {
    RCLCPP_ERROR(node_.get_logger(), "encoder_pulses_per_rev 必须大于 0（应与 0x2A 每圈脉冲一致，如 52100）");
    return false;
  }
  if (out.odometry_correction_factor <= 0.0) {
    RCLCPP_ERROR(node_.get_logger(), "odometry_correction_factor 必须大于 0");
    return false;
  }
  return true;
}

bool Platform::load_scara_arm_params(scara_arm::ScaraArmParams & out)
{
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

  int margin = 0;
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

  const auto margin_units = static_cast<std::int32_t>(margin);

  out.z.motor_id = static_cast<std::uint8_t>(id_z);
  out.z.limit_margin_units = margin_units;
  out.z.span_max = span_z;
  out.z.position_speed_rpm = clamp_u16(ps_z);
  out.z.position_accel = clamp_u8(pa_z);
  out.z.stall_current_ma = clamp_u16(z_stall);
  out.z.bump_speed_rpm = clamp_u16(bump_z);

  out.j1.motor_id = static_cast<std::uint8_t>(id_j1);
  out.j1.limit_margin_units = margin_units;
  out.j1.span_max = span_joint1;
  out.j1.position_speed_rpm = clamp_u16(ps_j1);
  out.j1.position_accel = clamp_u8(pa_j1);
  out.j1.stall_current_ma = clamp_u16(j1_stall);
  out.j1.bump_speed_rpm = clamp_u16(bump_j1);

  out.j2.motor_id = static_cast<std::uint8_t>(id_j2);
  out.j2.limit_margin_units = margin_units;
  out.j2.span_max = span_joint2;
  out.j2.position_speed_rpm = clamp_u16(ps_j2);
  out.j2.position_accel = clamp_u8(pa_j2);
  out.j2.stall_current_ma = clamp_u16(j2_stall);
  out.j2.bump_speed_rpm = clamp_u16(bump_j2);

  out.torque_z_up_ma = clamp_u16(tzu);
  out.torque_z_down_ma = clamp_u16(tzd);
  out.torque_j1_ma = clamp_u16(tj1);
  out.torque_j2_ma = clamp_u16(tj2);

  if (out.z.motor_id == 0U || out.j1.motor_id == 0U || out.j2.motor_id == 0U) {
    RCLCPP_ERROR(node_.get_logger(), "motor_z_id / motor_j1_id / motor_j2_id 均不可为 0");
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
  // 默认构造为 kMove 模式，这里显式切换到 kTwist（带看门狗的速度控制）
  if (!chassis_->set_mode(chassis::DriveMode::kTwist)) {
    const auto left_code  = chassis_->left_motor_error_code();
    const auto left_hint  = chassis_->left_motor_error_hint();
    const auto right_code = chassis_->right_motor_error_code();
    const auto right_hint = chassis_->right_motor_error_hint();
    RCLCPP_ERROR(
      node_.get_logger(),
      "底盘 set_mode(kTwist) 失败。"
      "左电机 error_code=0x%02X (%s)；右电机 error_code=0x%02X (%s)。"
      "请检查 CAN 总线、电机上电、接线与 ID 配置。",
      left_code, left_hint.c_str(), right_code, right_hint.c_str());
    chassis_.reset();
    can_.close();
    return false;
  }

  scara_arm::ScaraArmParams ap{};
  if (!load_scara_arm_params(ap)) {
    chassis_.reset();
    can_.close();
    return false;
  }

  try {
    arm_ = std::make_unique<scara_arm::RobotArm>(can_, ap);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_.get_logger(), "构造 RobotArm 失败: %s", e.what());
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
    // 关闭前切回 move 模式（关闭看门狗），实际电机停止由析构负责
    (void)chassis_->set_mode(chassis::DriveMode::kMove);
  }
  arm_.reset();
  chassis_.reset();
  if (can_.is_open()) {
    can_.close();
  }
  open_ = false;
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

void Platform::update_imu_yaw(double raw_yaw)
{
  current_imu_yaw_raw_.store(raw_yaw, std::memory_order_relaxed);
  if (chassis_) {
    chassis_->update_imu_yaw(raw_yaw);
  }
}

void Platform::reset_imu_zero()
{
  imu_yaw_offset_ = current_imu_yaw_raw_.load(std::memory_order_relaxed);
  RCLCPP_INFO(node_.get_logger(), "IMU 零点已重置，offset = %.6f rad", imu_yaw_offset_);
}

double Platform::get_corrected_imu_yaw() const
{
  return current_imu_yaw_raw_.load(std::memory_order_relaxed) - imu_yaw_offset_;
}

}  // namespace robot_platform
