#include "scara_arm/robot_arm.hpp"

#include "scara_arm/motion_playback.hpp"
#include "robot_driver/can_interface.hpp"
#include "robot_driver/pd42_motor.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>

#ifndef SCARA_ARM_RECORDINGS_DIR
#error "SCARA_ARM_RECORDINGS_DIR must be defined by CMake"
#endif

namespace scara_arm
{

namespace
{
constexpr auto kMotionRecordPeriod = std::chrono::milliseconds(100);

void print_calib_fail(const char * step, const robot_driver::Pd42Motor & m)
{
  std::cerr << "calibrate failed: " << step << " (motor_id=" << static_cast<int>(m.motor_id())
            << ", err=" << static_cast<int>(m.error_code()) << ")";
  if (m.error_code() == 0) {
    std::cerr << " — 无驱动应答错误：多为碰停寻边超时（超时内相电流未超标定阈值）、"
                 "bump_speed_rpm_z=0、或读相电流连续失败；见 arm_joint.cpp stall_seek_by_speed_\n";
  } else {
    std::cerr << '\n' << m.error_hint() << '\n';
  }
}

ArmJointParams with_joint_tag(ArmJointParams p, const char * tag)
{
  if (p.joint_tag == nullptr) {
    p.joint_tag = tag;
  }
  return p;
}

std::optional<double> read_joint_span(ArmJoint & joint)
{
  const auto h_opt = joint.forward_stop_pulse();
  if (!h_opt) {
    return std::nullopt;
  }
  const std::int32_t H = *h_opt;
  if (H <= 0) {
    return std::nullopt;
  }
  const auto pulse_opt = joint.motor().pos();
  if (!pulse_opt) {
    return std::nullopt;
  }
  const double span_max = joint.span_max();
  if (span_max <= 0.0 || std::isnan(span_max)) {
    return std::nullopt;
  }
  return (static_cast<double>(*pulse_opt) / static_cast<double>(H)) * span_max;
}

bool set_motor_position_mode(robot_driver::Pd42Motor & motor)
{
  return motor.set_mode(robot_driver::Pd42CommMode::kPosition);
}

bool enter_motor_torque_zero(robot_driver::Pd42Motor & motor)
{
  return motor.set_mode(robot_driver::Pd42CommMode::kTorque) && motor.set_torque(false, 0U);
}

std::string sanitize_action_name(const std::string & name)
{
  std::string out;
  out.reserve(name.size());
  for (unsigned char c : name) {
    if (c <= 31U || c == 127U) {
      continue;
    }
    if (c == '/' || c == '\\' || c == ':' || c == '*' || c == '?' || c == '"' || c == '<' ||
        c == '>' || c == '|')
    {
      continue;
    }
    out.push_back(static_cast<char>(c));
  }
  while (!out.empty() && (out.front() == ' ' || out.front() == '\t')) {
    out.erase(out.begin());
  }
  while (!out.empty() && (out.back() == ' ' || out.back() == '\t')) {
    out.pop_back();
  }
  return out;
}

std::string motion_recording_filename(const std::string & action_name)
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf{};
  localtime_r(&t, &tm_buf);
  std::ostringstream name;
  name << action_name << '_' << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << ".csv";
  return name.str();
}

struct MotionSampleRow
{
  double t_sec;
  double span_j1;
  double span_j2;
  double vel_j1;
  double vel_j2;
  double acc_j1;
  double acc_j2;
};

bool write_motion_recording_csv(
  const std::string & action_name, const std::vector<MotionSampleRow> & samples)
{
  namespace fs = std::filesystem;
  const fs::path dir(SCARA_ARM_RECORDINGS_DIR);
  std::error_code ec;
  fs::create_directories(dir, ec);
  if (ec) {
    std::cerr << "motion recording: cannot create directory " << dir << ": " << ec.message() << '\n';
    return false;
  }

  const fs::path path = dir / motion_recording_filename(action_name);
  std::ofstream out(path);
  if (!out) {
    std::cerr << "motion recording: cannot open " << path << " for write\n";
    return false;
  }

  out << "t_sec,span_j1,span_j2,vel_j1,vel_j2,acc_j1,acc_j2\n";
  out << std::setprecision(17);
  for (const auto & s : samples) {
    out << s.t_sec << ',' << s.span_j1 << ',' << s.span_j2 << ',' << s.vel_j1 << ',' << s.vel_j2
        << ',' << s.acc_j1 << ',' << s.acc_j2 << '\n';
  }
  if (!out.good()) {
    std::cerr << "motion recording: write failed for " << path << '\n';
    return false;
  }
  std::cout << "motion recording saved: " << path << " (" << samples.size() << " samples)\n";
  return true;
}
}  // namespace

RobotArm::RobotArm(robot_driver::CanInterface & can, ScaraArmParams params)
: can_(can),
  params_(params),
  torque_z_up_ma_(params.torque_z_up_ma),
  torque_z_down_ma_(params.torque_z_down_ma),
  torque_j1_ma_(params.torque_j1_ma),
  torque_j2_ma_(params.torque_j2_ma)
{
  if (params_.z.motor_id == 0U || params_.j1.motor_id == 0U || params_.j2.motor_id == 0U) {
    throw std::invalid_argument("motor_z_id / motor_j1_id / motor_j2_id 均不可为 0");
  }

  const ArmJointParams z = with_joint_tag(params_.z, "Z");
  const ArmJointParams j1 = with_joint_tag(params_.j1, "J1");
  const ArmJointParams j2 = with_joint_tag(params_.j2, "J2");

  mz_ = std::make_unique<robot_driver::Pd42Motor>(can_, z.motor_id, z.stall_current_ma);
  jz_ = std::make_unique<ArmJoint>(*mz_, z);

  m1_ = std::make_unique<robot_driver::Pd42Motor>(can_, j1.motor_id, j1.stall_current_ma);
  j1_ = std::make_unique<ArmJoint>(*m1_, j1);

  m2_ = std::make_unique<robot_driver::Pd42Motor>(can_, j2.motor_id, j2.stall_current_ma);
  j2_ = std::make_unique<ArmJoint>(*m2_, j2);
}

RobotArm::~RobotArm()
{
  abort_motion_recording_no_save();
  abort_motion_playing();
}

void RobotArm::abort_motion_playing()
{
  play_cancel_requested_.store(true);
  playing_.store(false);
}

void RobotArm::abort_motion_recording_no_save()
{
  if (!recording_.load()) {
    return;
  }
  record_finish_requested_.store(true);
  if (record_thread_.joinable()) {
    record_thread_.join();
  }
  (void)set_motor_position_mode(*m1_);
  (void)set_motor_position_mode(*m2_);
  recording_.store(false);
  record_buffer_.clear();
  recording_action_name_.clear();
}

void RobotArm::motion_record_loop()
{
  using clock = std::chrono::steady_clock;
  const clock::time_point t0 = clock::now();
  double prev_t = 0.0;
  double prev_j1 = 0.0;
  double prev_j2 = 0.0;
  double prev_v1 = 0.0;
  double prev_v2 = 0.0;
  bool has_prev = false;

  while (!record_finish_requested_.load()) {
    std::this_thread::sleep_for(kMotionRecordPeriod);
    if (record_finish_requested_.load()) {
      break;
    }

    const double t_sec = std::chrono::duration<double>(clock::now() - t0).count();

    std::lock_guard<std::mutex> lock(op_mutex_);
    if (!recording_.load()) {
      break;
    }

    const auto span_j1 = read_joint_span(*j1_);
    const auto span_j2 = read_joint_span(*j2_);
    if (!span_j1 || !span_j2) {
      continue;
    }

    MotionSample sample{};
    sample.t_sec = t_sec;
    sample.span_j1 = *span_j1;
    sample.span_j2 = *span_j2;
    if (!has_prev) {
      sample.vel_j1 = 0.0;
      sample.vel_j2 = 0.0;
      sample.acc_j1 = 0.0;
      sample.acc_j2 = 0.0;
    } else {
      const double dt = t_sec - prev_t;
      if (dt > 0.0) {
        sample.vel_j1 = (sample.span_j1 - prev_j1) / dt;
        sample.vel_j2 = (sample.span_j2 - prev_j2) / dt;
        sample.acc_j1 = (sample.vel_j1 - prev_v1) / dt;
        sample.acc_j2 = (sample.vel_j2 - prev_v2) / dt;
      }
    }
    prev_t = t_sec;
    prev_j1 = sample.span_j1;
    prev_j2 = sample.span_j2;
    prev_v1 = sample.vel_j1;
    prev_v2 = sample.vel_j2;
    has_prev = true;
    record_buffer_.push_back(sample);
  }
}

bool RobotArm::start_motion_recording(const std::string & action_name)
{
  const std::string safe_name = sanitize_action_name(action_name);
  if (safe_name.empty()) {
    std::cerr << "motion recording: invalid action_name (empty after sanitize)\n";
    return false;
  }

  if (recording_.load() || playing_.load()) {
    return false;
  }

  if (record_thread_.joinable()) {
    record_finish_requested_.store(true);
    record_thread_.join();
    record_finish_requested_.store(false);
    recording_.store(false);
    record_buffer_.clear();
    recording_action_name_.clear();
  }

  {
    std::lock_guard<std::mutex> lock(op_mutex_);
    if (recording_.load()) {
      return false;
    }
    if (!j1_->forward_stop_pulse() || !j2_->forward_stop_pulse()) {
      std::cerr << "motion recording: J1/J2 not calibrated (forward_stop_pulse missing)\n";
      return false;
    }

    if (!enter_motor_torque_zero(*m1_)) {
      return false;
    }
    if (!enter_motor_torque_zero(*m2_)) {
      (void)set_motor_position_mode(*m1_);
      return false;
    }

    recording_action_name_ = safe_name;
    record_buffer_.clear();
    record_finish_requested_.store(false);
    recording_.store(true);
  }

  record_thread_ = std::thread(&RobotArm::motion_record_loop, this);
  return true;
}

bool RobotArm::finish_motion_recording()
{
  std::vector<MotionSample> samples;
  std::string action_name;
  {
    std::lock_guard<std::mutex> lock(op_mutex_);
    if (!recording_.load()) {
      return false;
    }
    record_finish_requested_.store(true);
  }

  if (record_thread_.joinable()) {
    record_thread_.join();
  }

  {
    std::lock_guard<std::mutex> lock(op_mutex_);
    (void)set_motor_position_mode(*m1_);
    (void)set_motor_position_mode(*m2_);
    samples.swap(record_buffer_);
    action_name = recording_action_name_;
    recording_action_name_.clear();
    recording_.store(false);
    record_finish_requested_.store(true);
  }

  if (action_name.empty()) {
    return false;
  }

  std::vector<MotionSampleRow> rows;
  rows.reserve(samples.size());
  for (const auto & s : samples) {
    rows.push_back(MotionSampleRow{
      s.t_sec, s.span_j1, s.span_j2, s.vel_j1, s.vel_j2, s.acc_j1, s.acc_j2});
  }
  return write_motion_recording_csv(action_name, rows);
}

bool RobotArm::calibrate(std::function<void(const std::string & axis)> on_axis_complete)
{
  if (recording_.load() || playing_.load()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(op_mutex_);

  auto notify = [&](const char * axis) {
    if (on_axis_complete) {
      on_axis_complete(std::string(axis));
    }
  };

  if (!jz_->bump(0, torque_z_up_ma_)) {
    print_calib_fail("Z bump forward", jz_->motor());
    return false;
  }
  if (!jz_->bump(1, torque_z_down_ma_)) {
    print_calib_fail("Z bump reverse", jz_->motor());
    return false;
  }
  if (const auto mz = jz_->set_limits()) {
    reachable_span_min_z = *mz;
    reachable_span_max_z = jz_->span_max() - *mz;
  } else {
    print_calib_fail("Z set_limits", jz_->motor());
    return false;
  }
  if (!jz_->set_position(jz_->span_max() / 2.0)) {
    print_calib_fail("Z set_position", jz_->motor());
    return false;
  }
  notify("z");   // Z 轴标定完成

  if (!j1_->bump(1, torque_j1_ma_)) {
    print_calib_fail("J1 bump reverse", j1_->motor());
    return false;
  }
  if (!j2_->bump(1, torque_j2_ma_)) {
    print_calib_fail("J2 bump reverse", j2_->motor());
    return false;
  }
  if (!j2_->bump(0, torque_j2_ma_)) {
    print_calib_fail("J2 bump forward", j2_->motor());
    return false;
  }
  if (const auto m2 = j2_->set_limits()) {
    reachable_span_min_j2 = *m2;
    reachable_span_max_j2 = j2_->span_max() - *m2;
  } else {
    print_calib_fail("J2 set_limits", j2_->motor());
    return false;
  }

  (void)j2_->set_position(j2_->span_max() / 2.0);
  notify("j2");  // J2 轴标定完成

  if (!j1_->bump(0, torque_j1_ma_)) {
    print_calib_fail("J1 bump forward", j1_->motor());
    return false;
  }
  if (const auto m1 = j1_->set_limits()) {
    reachable_span_min_j1 = *m1;
    reachable_span_max_j1 = j1_->span_max() - *m1;
  } else {
    print_calib_fail("J1 set_limits", j1_->motor());
    return false;
  }
  j1_->set_position(j1_->span_max() / 2.0);
  notify("j1");  // J1 轴标定完成

  return true;
}

bool RobotArm::set_position(double span_z, double span_joint1, double span_joint2)
{
  if (recording_.load() || playing_.load()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(op_mutex_);

  if (reachable_span_max_z > reachable_span_min_z) {
    if (span_z < reachable_span_min_z || span_z > reachable_span_max_z) {
      return false;
    }
  }
  if (reachable_span_max_j1 > reachable_span_min_j1) {
    if (span_joint1 < reachable_span_min_j1 || span_joint1 > reachable_span_max_j1) {
      return false;
    }
  }
  if (reachable_span_max_j2 > reachable_span_min_j2) {
    if (span_joint2 < reachable_span_min_j2 || span_joint2 > reachable_span_max_j2) {
      return false;
    }
  }
  return jz_->set_position(span_z) && j1_->set_position(span_joint1) && j2_->set_position(span_joint2);
}

bool RobotArm::set_position_j1_j2(
  double span_j1, double span_j2,
  std::optional<std::uint16_t> rpm_j1, std::optional<std::uint16_t> rpm_j2,
  std::optional<std::uint8_t> accel_j1, std::optional<std::uint8_t> accel_j2)
{
  if (reachable_span_max_j1 > reachable_span_min_j1) {
    if (span_j1 < reachable_span_min_j1 || span_j1 > reachable_span_max_j1) {
      return false;
    }
  }
  if (reachable_span_max_j2 > reachable_span_min_j2) {
    if (span_j2 < reachable_span_min_j2 || span_j2 > reachable_span_max_j2) {
      return false;
    }
  }
  return j1_->set_position(span_j1, rpm_j1, accel_j1) &&
         j2_->set_position(span_j2, rpm_j2, accel_j2);
}

bool RobotArm::wait_j1_j2_arrived(
  const MotionPlaybackParams & params, double span_j1, double span_j2)
{
  using clock = std::chrono::steady_clock;
  const auto deadline = clock::now() +
    std::chrono::duration_cast<clock::duration>(
      std::chrono::duration<double>(params.arrived_timeout_sec));
  const auto poll = std::chrono::milliseconds(params.arrived_poll_ms);

  while (clock::now() < deadline) {
    if (play_cancel_requested_.load()) {
      return false;
    }
    if (j1_->is_at_target_span(span_j1) && j2_->is_at_target_span(span_j2)) {
      return true;
    }
    std::this_thread::sleep_for(poll);
  }
  return false;
}

bool RobotArm::play_motion_file(
  const std::string & csv_path,
  const MotionPlaybackParams & params,
  std::function<void(const MotionPlayFeedback &)> on_feedback,
  std::function<bool()> should_cancel)
{
  auto emit = [&](MotionPlayPhase phase, float progress, int index, int total, const std::string & msg) {
    if (!on_feedback) {
      return;
    }
    MotionPlayFeedback fb;
    fb.phase = phase;
    fb.progress = progress;
    fb.index = index;
    fb.total = total;
    fb.message = msg;
    on_feedback(fb);
  };

  auto cancelled = [&]() {
    return play_cancel_requested_.load() || (should_cancel && should_cancel());
  };

  if (recording_.load() || playing_.load()) {
    emit(MotionPlayPhase::Error, 0.f, 0, 0, "正在录制或播放");
    return false;
  }

  emit(MotionPlayPhase::Loading, 0.f, 0, 0, "加载轨迹");

  std::vector<MotionTrajectorySample> raw;
  if (!load_motion_trajectory_csv(csv_path, raw)) {
    emit(MotionPlayPhase::Error, 0.f, 0, 0, "无法加载 CSV");
    return false;
  }

  std::vector<MotionTrajectorySample> processed;
  std::vector<MotionPlaybackSegment> segments;
  std::string prep_err;
  if (!prepare_motion_trajectory(raw, params, *j1_, *j2_, processed, segments, prep_err)) {
    emit(MotionPlayPhase::Error, 0.f, 0, 0, prep_err);
    return false;
  }

  std::lock_guard<std::mutex> lock(op_mutex_);
  if (recording_.load() || playing_.load()) {
    emit(MotionPlayPhase::Error, 0.f, 0, 0, "状态冲突");
    return false;
  }
  play_cancel_requested_.store(false);
  playing_.store(true);

  const int total_segments = static_cast<int>(segments.size());
  const auto & first = processed.front();

  if (!set_position_j1_j2(first.span_j1, first.span_j2, std::nullopt, std::nullopt, std::nullopt,
                          std::nullopt))
  {
    playing_.store(false);
    emit(MotionPlayPhase::Error, 0.f, 0, total_segments, "首点下发失败");
    return false;
  }

  emit(
    MotionPlayPhase::WaitingFirstArrived, 0.f, 0, total_segments, "等待首点到位");

  if (!wait_j1_j2_arrived(params, first.span_j1, first.span_j2)) {
    playing_.store(false);
  emit(MotionPlayPhase::Error, 0.f, 0, total_segments, "首点到位超时或已取消");
    return false;
  }

  if (cancelled()) {
    playing_.store(false);
    emit(MotionPlayPhase::Error, 0.f, 0, total_segments, "已取消");
    return false;
  }

  emit(MotionPlayPhase::Playing, 0.f, 0, total_segments, "开始播放");

  for (int i = 0; i < total_segments; ++i) {
    if (cancelled()) {
      playing_.store(false);
      emit(MotionPlayPhase::Error, static_cast<float>(i) / static_cast<float>(total_segments), i,
           total_segments, "播放已取消");
      return false;
    }

    const auto & seg = segments[static_cast<std::size_t>(i)];
    if (!set_position_j1_j2(
          seg.span_j1, seg.span_j2, seg.rpm_j1, seg.rpm_j2, seg.accel_j1, seg.accel_j2))
    {
      playing_.store(false);
      emit(MotionPlayPhase::Error, static_cast<float>(i) / static_cast<float>(total_segments), i,
           total_segments, "路点下发失败");
      return false;
    }

    const float progress = static_cast<float>(i + 1) / static_cast<float>(total_segments);
    emit(
      MotionPlayPhase::Playing, progress, i + 1, total_segments,
      "播放中 " + std::to_string(i + 1) + "/" + std::to_string(total_segments));

    if (seg.duration_sec > 0.0) {
      std::this_thread::sleep_for(
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(seg.duration_sec)));
    }
  }

  playing_.store(false);
  emit(MotionPlayPhase::Done, 1.f, total_segments, total_segments, "播放完成");
  return true;
}

}  // namespace scara_arm
