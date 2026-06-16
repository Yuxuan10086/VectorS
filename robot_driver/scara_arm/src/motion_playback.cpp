#include "scara_arm/motion_playback.hpp"

#include "scara_arm/arm_joint.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace scara_arm
{

namespace
{
constexpr double kPulsePerRev = 51200.0;

std::string trim_copy(const std::string & s)
{
  std::size_t start = 0;
  while (start < s.size() && (s[start] == ' ' || s[start] == '\t')) {
    ++start;
  }
  std::size_t end = s.size();
  while (end > start && (s[end - 1] == ' ' || s[end - 1] == '\t' || s[end - 1] == '\r')) {
    --end;
  }
  return s.substr(start, end - start);
}

bool parse_csv_line(const std::string & line, MotionTrajectorySample & out, bool & has_vel_acc)
{
  if (trim_copy(line).empty()) {
    return false;
  }
  std::stringstream ss(line);
  std::string cell;
  std::vector<std::string> cols;
  while (std::getline(ss, cell, ',')) {
    cols.push_back(trim_copy(cell));
  }
  if (cols.size() < 3) {
    return false;
  }
  try {
    out.t_sec = std::stod(cols[0]);
    out.span_j1 = std::stod(cols[1]);
    out.span_j2 = std::stod(cols[2]);
    if (cols.size() >= 7) {
      out.vel_j1 = std::stod(cols[3]);
      out.vel_j2 = std::stod(cols[4]);
      out.acc_j1 = std::stod(cols[5]);
      out.acc_j2 = std::stod(cols[6]);
      has_vel_acc = true;
    } else {
      out.vel_j1 = 0.0;
      out.vel_j2 = 0.0;
      out.acc_j1 = 0.0;
      out.acc_j2 = 0.0;
      has_vel_acc = false;
    }
  } catch (...) {
    return false;
  }
  return true;
}

void fill_derivatives(std::vector<MotionTrajectorySample> & pts)
{
  if (pts.empty()) {
    return;
  }
  pts[0].vel_j1 = 0.0;
  pts[0].vel_j2 = 0.0;
  pts[0].acc_j1 = 0.0;
  pts[0].acc_j2 = 0.0;
  for (std::size_t i = 1; i < pts.size(); ++i) {
    const double dt = pts[i].t_sec - pts[i - 1].t_sec;
    if (dt <= 0.0) {
      pts[i].vel_j1 = 0.0;
      pts[i].vel_j2 = 0.0;
      pts[i].acc_j1 = 0.0;
      pts[i].acc_j2 = 0.0;
      continue;
    }
    const double v1 = (pts[i].span_j1 - pts[i - 1].span_j1) / dt;
    const double v2 = (pts[i].span_j2 - pts[i - 1].span_j2) / dt;
    pts[i].vel_j1 = v1;
    pts[i].vel_j2 = v2;
    pts[i].acc_j1 = (v1 - pts[i - 1].vel_j1) / dt;
    pts[i].acc_j2 = (v2 - pts[i - 1].vel_j2) / dt;
  }
}

double window_mean(const std::vector<double> & data, int i, int k)
{
  const int n = static_cast<int>(data.size());
  const int lo = std::max(0, i - k);
  const int hi = std::min(n - 1, i + k);
  double sum = 0.0;
  for (int j = lo; j <= hi; ++j) {
    sum += data[j];
  }
  return sum / static_cast<double>(hi - lo + 1);
}

void smooth_window(std::vector<MotionTrajectorySample> & pts, int k)
{
  if (k <= 0 || pts.size() < 2) {
    return;
  }
  std::vector<double> v1, v2, a1, a2;
  v1.reserve(pts.size());
  v2.reserve(pts.size());
  a1.reserve(pts.size());
  a2.reserve(pts.size());
  for (const auto & p : pts) {
    v1.push_back(p.vel_j1);
    v2.push_back(p.vel_j2);
    a1.push_back(p.acc_j1);
    a2.push_back(p.acc_j2);
  }
  for (std::size_t i = 0; i < pts.size(); ++i) {
    pts[i].vel_j1 = window_mean(v1, static_cast<int>(i), k);
    pts[i].vel_j2 = window_mean(v2, static_cast<int>(i), k);
    pts[i].acc_j1 = window_mean(a1, static_cast<int>(i), k);
    pts[i].acc_j2 = window_mean(a2, static_cast<int>(i), k);
  }
}

void merge_stationary(std::vector<MotionTrajectorySample> & pts, double eps)
{
  if (pts.size() < 2) {
    return;
  }
  std::vector<MotionTrajectorySample> merged;
  merged.push_back(pts.front());
  for (std::size_t i = 1; i < pts.size(); ++i) {
    const auto & prev = merged.back();
    const auto & cur = pts[i];
    if (std::abs(cur.span_j1 - prev.span_j1) < eps && std::abs(cur.span_j2 - prev.span_j2) < eps) {
      merged.back() = cur;
      merged.back().t_sec = cur.t_sec;
      continue;
    }
    merged.push_back(cur);
  }
  pts = std::move(merged);
}

std::uint16_t clamp_u16(std::int64_t v, std::uint16_t lo, std::uint16_t hi)
{
  if (v < static_cast<std::int64_t>(lo)) {
    return lo;
  }
  if (v > static_cast<std::int64_t>(hi)) {
    return hi;
  }
  return static_cast<std::uint16_t>(v);
}

std::uint8_t map_accel(double acc_mag, const MotionPlaybackParams & params)
{
  const double max_a = std::max(params.acc_span_per_s2_max, 1.0);
  const double t = std::min(1.0, acc_mag / max_a);
  const double mapped =
    params.accel_min + t * static_cast<double>(params.accel_max - params.accel_min);
  return static_cast<std::uint8_t>(std::llround(mapped));
}

double pulse_speed_from_span_vel(double span_vel, double span_max, std::int32_t H)
{
  if (span_max <= 0.0 || H <= 0) {
    return 0.0;
  }
  return (span_vel / span_max) * static_cast<double>(H);
}

}  // namespace

bool load_motion_trajectory_csv(const std::string & path, std::vector<MotionTrajectorySample> & out)
{
  out.clear();
  std::ifstream in(path);
  if (!in) {
    return false;
  }
  std::string line;
  if (!std::getline(in, line)) {
    return false;
  }
  const std::string header = trim_copy(line);
  if (header.find("span_j1") == std::string::npos) {
    return false;
  }
  bool file_has_vel = header.find("vel_j1") != std::string::npos;
  while (std::getline(in, line)) {
    MotionTrajectorySample sample{};
    bool row_has_vel = false;
    if (!parse_csv_line(line, sample, row_has_vel)) {
      continue;
    }
    if (!file_has_vel) {
      row_has_vel = false;
    }
    out.push_back(sample);
  }
  return !out.empty();
}

bool prepare_motion_trajectory(
  const std::vector<MotionTrajectorySample> & raw,
  const MotionPlaybackParams & params,
  ArmJoint & j1,
  ArmJoint & j2,
  std::vector<MotionTrajectorySample> & processed_points,
  std::vector<MotionPlaybackSegment> & segments,
  std::string & error_message)
{
  processed_points = raw;
  segments.clear();
  if (processed_points.empty()) {
    error_message = "轨迹无采样点";
    return false;
  }
  if (!j1.forward_stop_pulse() || !j2.forward_stop_pulse()) {
    error_message = "J1/J2 未标定";
    return false;
  }

  fill_derivatives(processed_points);
  smooth_window(processed_points, params.window_k);
  merge_stationary(processed_points, params.stationary_span_eps);

  if (processed_points.size() < 2) {
    error_message = "预处理后有效点不足 2";
    return false;
  }

  const std::uint16_t rpm_max_j1 = j1.position_speed_rpm();
  const std::uint16_t rpm_max_j2 = j2.position_speed_rpm();
  const double H1 = static_cast<double>(*j1.forward_stop_pulse());
  const double H2 = static_cast<double>(*j2.forward_stop_pulse());
  const double smax1 = j1.span_max();
  const double smax2 = j2.span_max();

  for (std::size_t i = 0; i + 1 < processed_points.size(); ++i) {
    const auto & a = processed_points[i];
    const auto & b = processed_points[i + 1];

    const auto p1_a = j1.span_to_pulse(a.span_j1);
    const auto p1_b = j1.span_to_pulse(b.span_j1);
    const auto p2_a = j2.span_to_pulse(a.span_j2);
    const auto p2_b = j2.span_to_pulse(b.span_j2);
    if (!p1_a || !p1_b || !p2_a || !p2_b) {
      error_message = "路点 span 无法映射为脉冲";
      return false;
    }

    const double dp1 = std::abs(static_cast<double>(*p1_b - *p1_a));
    const double dp2 = std::abs(static_cast<double>(*p2_b - *p2_a));

    double T = std::max(b.t_sec - a.t_sec, params.segment_time_min_sec);

    const double pdot1 = std::abs(pulse_speed_from_span_vel(a.vel_j1, smax1, static_cast<std::int32_t>(H1)));
    const double pdot2 = std::abs(pulse_speed_from_span_vel(a.vel_j2, smax2, static_cast<std::int32_t>(H2)));
    if (pdot1 > 1.0) {
      T = std::max(T, dp1 / pdot1);
    }
    if (pdot2 > 1.0) {
      T = std::max(T, dp2 / pdot2);
    }

    const double rpm1_need = (dp1 * 60.0) / (T * kPulsePerRev);
    const double rpm2_need = (dp2 * 60.0) / (T * kPulsePerRev);
    if (rpm1_need > rpm_max_j1 || rpm2_need > rpm_max_j2) {
      const double scale1 = rpm1_need > rpm_max_j1 ? rpm_max_j1 / rpm1_need : 1.0;
      const double scale2 = rpm2_need > rpm_max_j2 ? rpm_max_j2 / rpm2_need : 1.0;
      const double scale = std::min(scale1, scale2);
      T /= std::max(scale, 1e-6);
    }

    const std::uint16_t rpm1 = clamp_u16(
      static_cast<std::int64_t>(std::llround((dp1 * 60.0) / (T * kPulsePerRev))),
      params.rpm_min, rpm_max_j1);
    const std::uint16_t rpm2 = clamp_u16(
      static_cast<std::int64_t>(std::llround((dp2 * 60.0) / (T * kPulsePerRev))),
      params.rpm_min, rpm_max_j2);

    MotionPlaybackSegment seg;
    seg.span_j1 = b.span_j1;
    seg.span_j2 = b.span_j2;
    seg.rpm_j1 = rpm1;
    seg.rpm_j2 = rpm2;
    seg.accel_j1 = map_accel(std::abs(a.acc_j1), params);
    seg.accel_j2 = map_accel(std::abs(a.acc_j2), params);
    seg.duration_sec = T;
    segments.push_back(seg);
  }
  return true;
}

}  // namespace scara_arm
