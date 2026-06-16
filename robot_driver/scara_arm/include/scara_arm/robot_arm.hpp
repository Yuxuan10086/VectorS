// SCARA 机械臂（三关节 Z / J1 / J2）
#ifndef SCARA_ARM__ROBOT_ARM_HPP_
#define SCARA_ARM__ROBOT_ARM_HPP_

#include <cstdint>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "robot_driver/can_interface.hpp"

#include "scara_arm/arm_joint.hpp"
#include "scara_arm/motion_playback.hpp"

namespace robot_driver
{
class Pd42Motor;
}  // namespace robot_driver

namespace scara_arm
{

/** 整臂参数：三轴关节包 + 标定碰停扭矩（由调用方从 ROS 参数填入） */
struct ScaraArmParams
{
  ArmJointParams z;
  ArmJointParams j1;
  ArmJointParams j2;
  std::uint16_t torque_z_up_ma{};
  std::uint16_t torque_z_down_ma{};
  std::uint16_t torque_j1_ma{};
  std::uint16_t torque_j2_ma{};
};

class RobotArm
{
public:
  RobotArm(robot_driver::CanInterface & can, ScaraArmParams params);

  ~RobotArm();

  RobotArm(const RobotArm &) = delete;
  RobotArm & operator=(const RobotArm &) = delete;

  const ScaraArmParams & params() const { return params_; }

  // Z 碰停与限位 → J2 碰停与限位 → J2 半行程 → J1 碰停与限位（细节见 robot_arm.cpp）
  // 可选 on_axis_complete 回调：每当一个轴（"z"、"j2"、"j1"）标定成功后立即调用，用于 Action Feedback
  bool calibrate(std::function<void(const std::string & axis)> on_axis_complete = nullptr);

  /** 三轴同时 set_position；整臂标定后 span 须在各类 reachable_span_* 内，否则 false */
  bool set_position(double span_z, double span_joint1, double span_joint2);

  /** J1/J2 零力矩拖动录制：须已标定；action_name 写入 CSV 文件名 */
  bool start_motion_recording(const std::string & action_name);

  /** 结束录制、恢复 J1/J2 位置模式，轨迹写入包内 recordings/ CSV；无内存缓存 */
  bool finish_motion_recording();

  bool is_motion_recording() const noexcept { return recording_.load(); }

  /** 从 CSV 绝对路径播放 J1/J2 轨迹（首点到位门控 + 段定时下发） */
  bool play_motion_file(
    const std::string & csv_path,
    const MotionPlaybackParams & params,
    std::function<void(const MotionPlayFeedback &)> on_feedback = nullptr,
    std::function<bool()> should_cancel = nullptr);

  bool is_motion_playing() const noexcept { return playing_.load(); }

  /** `calibrate()` 内由各轴 `set_limits()` 单侧裕量与名义 `span_max` 写入；未标定前为 0 */
  double reachable_span_min_z = 0.0;
  double reachable_span_max_z = 0.0;
  double reachable_span_min_j1 = 0.0;
  double reachable_span_max_j1 = 0.0;
  double reachable_span_min_j2 = 0.0;
  double reachable_span_max_j2 = 0.0;

  ArmJoint & joint_z() noexcept { return *jz_; }
  ArmJoint & joint1() noexcept { return *j1_; }
  ArmJoint & joint2() noexcept { return *j2_; }

  robot_driver::Pd42Motor & motor_z() noexcept { return *mz_; }
  robot_driver::Pd42Motor & motor_j1() noexcept { return *m1_; }
  robot_driver::Pd42Motor & motor_j2() noexcept { return *m2_; }

private:
  struct MotionSample
  {
    double t_sec;
    double span_j1;
    double span_j2;
    double vel_j1;
    double vel_j2;
    double acc_j1;
    double acc_j2;
  };

  void motion_record_loop();
  void abort_motion_recording_no_save();
  void abort_motion_playing();

  bool set_position_j1_j2(
    double span_j1, double span_j2,
    std::optional<std::uint16_t> rpm_j1, std::optional<std::uint16_t> rpm_j2,
    std::optional<std::uint8_t> accel_j1, std::optional<std::uint8_t> accel_j2);

  bool wait_j1_j2_arrived(
    const MotionPlaybackParams & params, double span_j1, double span_j2);

  robot_driver::CanInterface & can_;
  ScaraArmParams params_;
  std::unique_ptr<robot_driver::Pd42Motor> mz_;
  std::unique_ptr<robot_driver::Pd42Motor> m1_;
  std::unique_ptr<robot_driver::Pd42Motor> m2_;
  std::unique_ptr<ArmJoint> jz_;
  std::unique_ptr<ArmJoint> j1_;
  std::unique_ptr<ArmJoint> j2_;
  std::uint16_t torque_z_up_ma_;
  std::uint16_t torque_z_down_ma_;
  std::uint16_t torque_j1_ma_;
  std::uint16_t torque_j2_ma_;

  std::mutex op_mutex_;
  std::thread record_thread_;
  std::atomic<bool> record_finish_requested_{true};
  std::atomic<bool> recording_{false};
  std::string recording_action_name_;
  std::vector<MotionSample> record_buffer_;

  std::atomic<bool> playing_{false};
  std::atomic<bool> play_cancel_requested_{false};
};

}  // namespace scara_arm

#endif  // SCARA_ARM__ROBOT_ARM_HPP_
