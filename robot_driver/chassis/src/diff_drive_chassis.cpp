#include "chassis/diff_drive_chassis.hpp"

#include "robot_driver/pd42_motor.hpp"

#include <cmath>
#include <thread>

namespace chassis
{

struct DiffDriveChassis::Motors
{
  std::unique_ptr<robot_driver::Pd42Motor> left;
  std::unique_ptr<robot_driver::Pd42Motor> right;
};

DiffDriveChassis::DiffDriveChassis(robot_driver::CanInterface & can, DiffDriveParams params)
: can_(can), params_(params), motors_(std::make_unique<Motors>())
{
}

DiffDriveChassis::~DiffDriveChassis() = default;

bool DiffDriveChassis::left_rev(bool nominal_reverse) const
{
  return params_.left_reverse_inverted ? !nominal_reverse : nominal_reverse;
}

float DiffDriveChassis::rpm_from_wheel_ms(double v_ms) const
{
  const double v = std::abs(v_ms);
  if (params_.wheel_diameter_m <= 0.0) {
    return 0.F;
  }
  return static_cast<float>(v * 60.0 / (M_PI * params_.wheel_diameter_m));
}

bool DiffDriveChassis::sleep_seconds(double sec)
{
  if (sec <= 0.0) {
    return true;
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(sec));
  return true;
}

bool DiffDriveChassis::zero_speed()
{
  if (!motors_->left || !motors_->right) {
    return false;
  }
  return motors_->left->set_speed(0.F, left_rev(false), params_.accel) &&
         motors_->right->set_speed(0.F, false, params_.accel);
}

bool DiffDriveChassis::initialize()
{
  if (params_.left_id == 0U || params_.right_id == 0U) {
    return false;
  }
  if (params_.wheel_diameter_m <= 0.0 || params_.track_m <= 0.0) {
    return false;
  }
  if (params_.travel_speed_m_s <= 0.0 || params_.pivot_wheel_m_s <= 0.0) {
    return false;
  }

  motors_->left = std::make_unique<robot_driver::Pd42Motor>(can_, params_.left_id);
  motors_->right = std::make_unique<robot_driver::Pd42Motor>(can_, params_.right_id);

  if (!motors_->left->initialize() || !motors_->right->initialize()) {
    return false;
  }
  return motors_->left->set_mode(robot_driver::Pd42CommMode::kSpeed) &&
         motors_->right->set_mode(robot_driver::Pd42CommMode::kSpeed);
}

bool DiffDriveChassis::stop()
{
  return zero_speed();
}

bool DiffDriveChassis::go(double meters)
{
  const float rpm = rpm_from_wheel_ms(params_.travel_speed_m_s);
  const double dur = std::abs(meters) / params_.travel_speed_m_s;
  const bool reverse = (meters < 0.0);
  if (!motors_->left->set_speed(rpm, left_rev(reverse), params_.accel) ||
      !motors_->right->set_speed(rpm, reverse, params_.accel)) {
    return false;
  }
  (void)sleep_seconds(dur);
  return zero_speed();
}

bool DiffDriveChassis::rotate(double deg)
{
  const double v = params_.pivot_wheel_m_s;
  const double omega = 2.0 * v / params_.track_m;
  const double theta = std::abs(deg) * M_PI / 180.0;
  const double dur = theta / omega;
  const float rpm = rpm_from_wheel_ms(v);
  const bool ccw = (deg > 0.0);
  if (!motors_->left->set_speed(rpm, left_rev(ccw), params_.accel) ||
      !motors_->right->set_speed(rpm, !ccw, params_.accel)) {
    return false;
  }
  (void)sleep_seconds(dur);
  return zero_speed();
}

bool DiffDriveChassis::spin(double deg)
{
  return rotate(deg);
}

}  // namespace chassis
