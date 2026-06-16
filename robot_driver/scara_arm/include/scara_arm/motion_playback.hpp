// 动作 CSV 加载、预处理与段参数（无 ROS）
#ifndef SCARA_ARM__MOTION_PLAYBACK_HPP_
#define SCARA_ARM__MOTION_PLAYBACK_HPP_

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace scara_arm
{

class ArmJoint;

/** 轨迹采样点（CSV 一行） */
struct MotionTrajectorySample
{
  double t_sec{};
  double span_j1{};
  double span_j2{};
  double vel_j1{};
  double vel_j2{};
  double acc_j1{};
  double acc_j2{};
};

/** 播放配置（可由 YAML / platform 注入） */
struct MotionPlaybackParams
{
  int window_k{2};
  int arrived_poll_ms{20};
  double arrived_timeout_sec{15.0};
  std::uint16_t rpm_min{5};
  double stationary_span_eps{0.05};
  double acc_span_per_s2_max{500.0};
  std::uint8_t accel_min{5};
  std::uint8_t accel_max{40};
  double segment_time_min_sec{0.05};
};

enum class MotionPlayPhase
{
  Loading,
  WaitingFirstArrived,
  Playing,
  Done,
  Error,
};

struct MotionPlayFeedback
{
  MotionPlayPhase phase{MotionPlayPhase::Loading};
  float progress{0.f};
  int index{0};
  int total{0};
  std::string message;
};

/** 执行段：目标 span + 本段运动参数 */
struct MotionPlaybackSegment
{
  double span_j1{};
  double span_j2{};
  std::uint16_t rpm_j1{};
  std::uint16_t rpm_j2{};
  std::uint8_t accel_j1{};
  std::uint8_t accel_j2{};
  double duration_sec{};
};

/** 从 CSV 加载；旧三列文件 vel/acc 置 0 由预处理补全 */
bool load_motion_trajectory_csv(const std::string & path, std::vector<MotionTrajectorySample> & out);

/**
 * 预处理：补 vel/acc、滑动窗口、合并静止点、生成段列表。
 * j1/j2 用于脉冲换算与 rpm 上限。
 */
bool prepare_motion_trajectory(
  const std::vector<MotionTrajectorySample> & raw,
  const MotionPlaybackParams & params,
  ArmJoint & j1,
  ArmJoint & j2,
  std::vector<MotionTrajectorySample> & processed_points,
  std::vector<MotionPlaybackSegment> & segments,
  std::string & error_message);

}  // namespace scara_arm

#endif  // SCARA_ARM__MOTION_PLAYBACK_HPP_
