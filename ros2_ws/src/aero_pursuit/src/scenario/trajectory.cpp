#include "aero_pursuit/scenario/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aero_pursuit::scenario
{

double wrapAngle(double a)
{
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a <= 0.0) {
    a += 2.0 * M_PI;
  }
  return a - M_PI;
}

// -- SampledPath ---------------------------------------------------------------------

SampledPath::SampledPath(std::vector<Eigen::Vector3d> points, bool closed)
: points_(std::move(points)), closed_(closed)
{
  if (points_.empty()) {
    throw std::invalid_argument("path without points");
  }
  // Removes repeated vertices (zero-length segments have no tangent).
  std::vector<Eigen::Vector3d> unique{points_.front()};
  for (std::size_t i = 1; i < points_.size(); ++i) {
    if ((points_[i] - unique.back()).norm() > 1e-6) {
      unique.push_back(points_[i]);
    }
  }
  points_ = std::move(unique);
  closed_ = closed_ && points_.size() > 2;
  if (closed_ && (points_.back() - points_.front()).norm() > 1e-6) {
    points_.push_back(points_.front());
  }
  arc_.assign(points_.size(), 0.0);
  for (std::size_t i = 1; i < points_.size(); ++i) {
    arc_[i] = arc_[i - 1] + (points_[i] - points_[i - 1]).norm();
  }
}

std::size_t SampledPath::segment(double s) const
{
  // Last i with arc_[i] <= s, limited to the second-to-last vertex.
  const auto it = std::upper_bound(arc_.begin(), arc_.end(), s);
  const auto i = static_cast<std::size_t>(std::max<std::ptrdiff_t>(0, it - arc_.begin() - 1));
  return std::min(i, points_.size() - 2);
}

Eigen::Vector3d SampledPath::position(double s) const
{
  if (points_.size() == 1) {
    return points_.front();
  }
  s = std::clamp(s, 0.0, length());
  const std::size_t i = segment(s);
  const double len = arc_[i + 1] - arc_[i];
  const double u = len > 0.0 ? (s - arc_[i]) / len : 0.0;
  return points_[i] + u * (points_[i + 1] - points_[i]);
}

Eigen::Vector3d SampledPath::tangent(double s) const
{
  if (points_.size() == 1) {
    return Eigen::Vector3d::Zero();
  }
  const std::size_t i = segment(std::clamp(s, 0.0, length()));
  return (points_[i + 1] - points_[i]).normalized();
}

double SampledPath::project(const Eigen::Vector3d & p) const
{
  if (points_.size() == 1) {
    return 0.0;
  }
  double best_d = INFINITY;
  double best_s = 0.0;
  for (std::size_t i = 0; i + 1 < points_.size(); ++i) {
    const Eigen::Vector3d ab = points_[i + 1] - points_[i];
    const double u = std::clamp((p - points_[i]).dot(ab) / ab.squaredNorm(), 0.0, 1.0);
    const double d = (points_[i] + u * ab - p).squaredNorm();
    if (d < best_d) {
      best_d = d;
      best_s = arc_[i] + u * (arc_[i + 1] - arc_[i]);
    }
  }
  return best_s;
}

std::vector<Eigen::Vector3d> catmullRom(
  const std::vector<Eigen::Vector3d> & control, bool closed, int samples_per_segment)
{
  const std::size_t n = control.size();
  if (n < 3) {
    return control;
  }
  auto at = [&](std::ptrdiff_t i) -> const Eigen::Vector3d & {
      if (closed) {
        return control[static_cast<std::size_t>((i % static_cast<std::ptrdiff_t>(n) + n) % n)];
      }
      return control[static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(i, 0, n - 1))];
    };
  // Centripetal parameterization (alpha = 0.5): t_{k+1} = t_k + |P_{k+1} - P_k|^0.5.
  auto knot = [](double t, const Eigen::Vector3d & a, const Eigen::Vector3d & b) {
      return t + std::max(std::sqrt((b - a).norm()), 1e-4);
    };
  std::vector<Eigen::Vector3d> out;
  const std::size_t segments = closed ? n : n - 1;
  out.reserve(segments * static_cast<std::size_t>(samples_per_segment) + 1);
  for (std::size_t seg = 0; seg < segments; ++seg) {
    const auto i = static_cast<std::ptrdiff_t>(seg);
    const Eigen::Vector3d & p0 = at(i - 1);
    const Eigen::Vector3d & p1 = at(i);
    const Eigen::Vector3d & p2 = at(i + 1);
    const Eigen::Vector3d & p3 = at(i + 2);
    const double t0 = 0.0;
    const double t1 = knot(t0, p0, p1);
    const double t2 = knot(t1, p1, p2);
    const double t3 = knot(t2, p2, p3);
    for (int k = 0; k < samples_per_segment; ++k) {
      const double t = t1 + (t2 - t1) * k / samples_per_segment;
      const Eigen::Vector3d a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1;
      const Eigen::Vector3d a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2;
      const Eigen::Vector3d a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3;
      const Eigen::Vector3d b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2;
      const Eigen::Vector3d b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3;
      out.push_back((t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2);
    }
  }
  out.push_back(closed ? control.front() : control.back());
  return out;
}

// -- Trajectory ------------------------------------------------------------------------

Trajectory::Trajectory(SampledPath path, std::vector<PathKnot> knots)
: path_(std::move(path)), knots_(std::move(knots))
{
  if (knots_.empty()) {
    knots_.push_back({0.0, 0.0, 1.0});
  }
  knots_.front().arc = 0.0;
  for (auto & k : knots_) {
    if (!(k.speed > 0.0)) {
      throw std::invalid_argument("trajectory speed must be > 0");
    }
    k.hold = std::max(0.0, k.hold);
  }
  const double length = path_.length();
  // Final knot: arrival at the end of the curve (with the wait of the last point, if any).
  if (knots_.size() > 1 && knots_.back().arc >= length - 1e-6) {
    knots_.back().arc = length;
  } else {
    knots_.push_back({length, 0.0, knots_.back().speed});
  }
  knot_time_.assign(knots_.size(), 0.0);
  for (std::size_t i = 1; i < knots_.size(); ++i) {
    const auto & prev = knots_[i - 1];
    knot_time_[i] = knot_time_[i - 1] + prev.hold + std::max(0.0, knots_[i].arc - prev.arc) / prev.speed;
  }
  knot_time_.back() += knots_.back().hold;
}

Trajectory Trajectory::hover(const Eigen::Vector3d & point, double duration)
{
  return Trajectory(SampledPath({point}, false), {{0.0, std::max(0.0, duration), 1.0}});
}

Trajectory Trajectory::line(const Eigen::Vector3d & from, const Eigen::Vector3d & to, double speed)
{
  return Trajectory(SampledPath({from, to}, false), {{0.0, 0.0, speed}});
}

Reference Trajectory::sample(double t) const
{
  Reference ref;
  ref.has_position = true;
  t = std::clamp(t, 0.0, duration());
  // Knot i: arrives at knot_time_[i], waits hold and goes on to i+1.
  const auto it = std::upper_bound(knot_time_.begin(), knot_time_.end(), t);
  const std::size_t i = static_cast<std::size_t>(std::max<std::ptrdiff_t>(0, it - knot_time_.begin() - 1));
  if (i + 1 >= knots_.size()) {
    ref.position = path_.position(path_.length());
    return ref;
  }
  const PathKnot & k = knots_[i];
  const double moving = t - knot_time_[i] - k.hold;
  if (moving < 0.0) {  // Still waiting at the knot
    ref.position = path_.position(k.arc);
    return ref;
  }
  const double s = std::min(k.arc + moving * k.speed, knots_[i + 1].arc);
  ref.position = path_.position(s);
  if (s < knots_[i + 1].arc) {
    ref.velocity = path_.tangent(s) * k.speed;
  }
  return ref;
}

}  // namespace aero_pursuit::scenario
