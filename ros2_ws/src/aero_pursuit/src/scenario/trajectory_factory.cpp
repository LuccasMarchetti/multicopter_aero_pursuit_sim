#include "aero_pursuit/scenario/trajectory_factory.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <random>
#include <stdexcept>

namespace aero_pursuit::scenario
{
namespace
{

constexpr int kCurveSamples = 180;  ///< Samples per lap of the parametric curves

using Curve = std::vector<Eigen::Vector3d>;

/// Shape frame -> world: Rz(heading) Ry(pitch) Rx(roll).
Eigen::Vector3d orient(const Eigen::Vector3d & v, const TrajectorySpec & s)
{
  return (Eigen::AngleAxisd(s.heading, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(s.pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(s.roll, Eigen::Vector3d::UnitX())) * v;
}

/// Samples f(u), u in [0, 2pi), around the center with the shape orientation.
template<typename F>
Curve parametric(const TrajectorySpec & s, int samples, F && f)
{
  Curve c;
  c.reserve(static_cast<std::size_t>(samples));
  for (int k = 0; k < samples; ++k) {
    const double u = 2.0 * M_PI * k / samples;
    c.push_back(s.center + orient(f(u), s));
  }
  return c;
}

/// Single-speed trajectory along a curve.
Trajectory constantSpeed(const Curve & c, bool closed, double speed)
{
  return Trajectory(SampledPath(c, closed), {{0.0, 0.0, speed}});
}

Trajectory waypoints(const TrajectorySpec & s)
{
  Curve control;
  for (const auto & w : s.points) {
    control.push_back(w.position);
  }
  const bool spline = s.interpolation == "spline" && control.size() >= 3;
  constexpr int kSplineSamples = 24;
  const Curve curve = spline ? catmullRom(control, s.closed, kSplineSamples) : control;
  SampledPath path(curve, s.closed);

  // One knot per waypoint (own speed/wait) at the corresponding arc of the curve.
  std::vector<PathKnot> knots;
  const std::size_t n = s.points.size();
  for (std::size_t i = 0; i < n; ++i) {
    double arc = 0.0;
    if (i > 0) {
      arc = spline ? path.project(control[i]) : SampledPath(
        Curve(control.begin(), control.begin() + static_cast<std::ptrdiff_t>(i) + 1), false).length();
      if (spline) {
        // The projection of a control point falls on the sampled vertex itself; guarantees
        // monotonicity on curves that pass close to themselves.
        arc = std::max(arc, knots.back().arc);
      }
    }
    const auto & w = s.points[i];
    knots.push_back({arc, w.hold, w.speed > 0.0 ? w.speed : s.speed});
  }
  if (s.closed && n > 0) {
    knots.push_back({path.length(), s.points.front().hold, knots.front().speed});
  }
  return Trajectory(std::move(path), std::move(knots));
}

/// Points with a time each (CSV t,x,y,z): one knot per point with the speed that reaches the
/// next point at its time (piecewise constant speed); a point repeated in place is a wait.
Trajectory timed(const TrajectorySpec & s)
{
  Curve control;
  for (const auto & w : s.points) {
    control.push_back(w.position);
  }
  const bool spline = s.interpolation == "spline" && control.size() >= 3;
  constexpr int kSplineSamples = 8;
  const Curve curve = spline ? catmullRom(control, false, kSplineSamples) : control;
  SampledPath path(curve, false);

  std::vector<double> arc(control.size(), 0.0);
  for (std::size_t i = 1; i < control.size(); ++i) {
    arc[i] = spline ? std::max(path.project(control[i]), arc[i - 1]) :
      arc[i - 1] + (control[i] - control[i - 1]).norm();
  }
  constexpr double kStill = 1e-4;  ///< [m] below this a segment is a wait
  std::vector<PathKnot> knots;
  for (std::size_t i = 0; i + 1 < control.size(); ++i) {
    const double dt = *s.points[i + 1].time - *s.points[i].time;
    const double ds = arc[i + 1] - arc[i];
    if (ds < kStill) {
      knots.push_back({arc[i], dt, 1.0});
    } else {
      knots.push_back({arc[i], 0.0, ds / dt});
    }
  }
  knots.push_back({path.length(), 0.0, knots.back().speed});
  return Trajectory(std::move(path), std::move(knots));
}

}  // namespace

Trajectory buildTrajectory(const TrajectorySpec & spec, const Eigen::Vector3d & anchor)
{
  TrajectorySpec s = spec;
  if (s.relative) {
    s.center += anchor;
    for (auto & w : s.points) {
      w.position += anchor;
    }
  }
  if (s.type == "waypoints") {
    if (s.points.empty()) {
      throw std::invalid_argument("waypoints without points");
    }
    return waypoints(s);
  }
  if (s.type == "timed") {
    if (s.points.size() < 2) {
      throw std::invalid_argument("timed needs at least two points");
    }
    return timed(s);
  }
  if (s.type == "line") {
    if (s.points.size() < 2) {
      throw std::invalid_argument("line needs two points");
    }
    const Curve c = {s.points[0].position, s.points[1].position};
    // Back and forth: polyline A -> B -> A (open, so as not to collapse the closed path).
    return s.closed ? constantSpeed({c[0], c[1], c[0]}, false, s.speed) : constantSpeed(c, false, s.speed);
  }
  if (s.type == "hover") {
    const Eigen::Vector3d p = s.points.empty() ? (spec.relative ? anchor : s.center) : s.points[0].position;
    return Trajectory::hover(p, s.duration);
  }
  if (s.type == "polygon") {
    const double dir = s.clockwise ? -1.0 : 1.0;
    Curve c;
    for (int k = 0; k < s.sides; ++k) {
      const double a = s.phase + dir * 2.0 * M_PI * k / s.sides;
      c.push_back(s.center + orient({s.radius * std::cos(a), s.radius * std::sin(a), 0.0}, s));
    }
    return constantSpeed(c, true, s.speed);
  }
  if (s.type == "circle") {
    const double dir = s.clockwise ? -1.0 : 1.0;
    return constantSpeed(parametric(s, kCurveSamples, [&](double u) {
        const double a = s.phase + dir * u;
        return Eigen::Vector3d(s.radius * std::cos(a), s.radius * std::sin(a), 0.0);
      }), true, s.speed);
  }
  if (s.type == "figure8") {
    // Lemniscate of Gerono: x = a sin(u), y = b sin(u) cos(u); z oscillates twice per lap.
    return constantSpeed(parametric(s, kCurveSamples, [&](double u) {
        return Eigen::Vector3d(s.size.x() * std::sin(u), s.size.y() * std::sin(u) * std::cos(u),
          s.size.z() * std::sin(2.0 * u));
      }), true, s.speed);
  }
  if (s.type == "lissajous") {
    const Eigen::Vector3d f = s.frequency.cwiseMax(0.0);
    const int samples = kCurveSamples * std::max(1, static_cast<int>(std::ceil(f.maxCoeff())));
    return constantSpeed(parametric(s, samples, [&](double u) {
        return Eigen::Vector3d(
          s.size.x() * std::sin(f.x() * u + s.phase3.x()),
          s.size.y() * std::sin(f.y() * u + s.phase3.y()),
          s.size.z() * std::sin(f.z() * u + s.phase3.z()));
      }), true, s.speed);
  }
  if (s.type == "helix") {
    // Climbs `height` in `turns` laps and descends along the same path: closes at the start.
    const double dir = s.clockwise ? -1.0 : 1.0;
    const int samples = std::max(8, static_cast<int>(kCurveSamples * std::max(0.1, s.turns)));
    Curve up;
    for (int k = 0; k <= samples; ++k) {
      const double u = static_cast<double>(k) / samples;
      const double a = s.phase + dir * 2.0 * M_PI * s.turns * u;
      up.push_back(s.center + orient({s.radius * std::cos(a), s.radius * std::sin(a), s.height * u}, s));
    }
    Curve c = up;
    c.insert(c.end(), up.rbegin() + 1, up.rend());
    return constantSpeed(c, false, s.speed);
  }
  if (s.type == "random") {
    std::mt19937 rng(s.seed);
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    Curve control;
    for (int k = 0; k < s.count; ++k) {
      control.push_back(s.center + orient(
        Eigen::Vector3d(uni(rng) * s.size.x(), uni(rng) * s.size.y(), uni(rng) * s.size.z()), s));
    }
    return constantSpeed(catmullRom(control, true, 24), true, s.speed);
  }
  throw std::invalid_argument("unknown trajectory type: " + s.type);
}

}  // namespace aero_pursuit::scenario
