#ifndef CHASSIS__DIFF_DRIVE_CHASSIS_HPP_
#define CHASSIS__DIFF_DRIVE_CHASSIS_HPP_

#include <cstdint>
#include <memory>

#include "robot_driver/can_interface.hpp"

namespace chassis
{

/** 底盘运动学/电机参数，由调用方从 ROS 参数填入，库内不写业务默认值 */
struct DiffDriveParams
{
  std::uint8_t left_id{};
  std::uint8_t right_id{};
  bool left_reverse_inverted{false};
  double wheel_diameter_m{0.0};
  double track_m{0.0};
  double travel_speed_m_s{0.0};
  double pivot_wheel_m_s{0.0};
  std::uint8_t accel{0};
};

/** 前驱差速底盘：共用 CanInterface，不拥有 CAN */
class DiffDriveChassis
{
public:
  DiffDriveChassis(robot_driver::CanInterface & can, DiffDriveParams params);
  ~DiffDriveChassis();

  DiffDriveChassis(const DiffDriveChassis &) = delete;
  DiffDriveChassis & operator=(const DiffDriveChassis &) = delete;

  bool initialize();
  bool stop();
  bool go(double meters);
  bool rotate(double deg);
  bool spin(double deg);

  robot_driver::CanInterface & can() { return can_; }
  const DiffDriveParams & params() const { return params_; }

private:
  bool left_rev(bool nominal_reverse) const;
  float rpm_from_wheel_ms(double v_ms) const;
  bool zero_speed();
  bool sleep_seconds(double sec);

  robot_driver::CanInterface & can_;
  DiffDriveParams params_;
  struct Motors;
  std::unique_ptr<Motors> motors_;
};

}  // namespace chassis

#endif  // CHASSIS__DIFF_DRIVE_CHASSIS_HPP_
