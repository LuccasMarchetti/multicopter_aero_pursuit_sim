#pragma once

#include "aero_pursuit/evasion/agent_state.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <vector>

// Pre-planned trajectories of a scenario's drones. ROS independent: the same
// code generates the reference at runtime (scenario_runner) and the preview
// (scenario_tool / GCS), so what is drawn is exactly what is flown.
namespace aero_pursuit::scenario
{

using evasion::AgentState;

/// Reference to follow at an instant (world ENU frame).
struct Reference
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  bool has_position = false;    ///< false: velocity only (reactive behaviors)
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();  ///< Direct feed [m/s]
  double yaw = 0.0;             ///< Rumo ENU [rad]
  bool has_yaw = false;         ///< false: keeps the current heading
};

/// Angle in (-pi, pi].
double wrapAngle(double a);

/// Polyline parameterized by arc length.
class SampledPath
{
public:
  SampledPath() = default;
  /// `points` with at least one vertex; `closed` joins the last to the first.
  SampledPath(std::vector<Eigen::Vector3d> points, bool closed);

  double length() const {return arc_.empty() ? 0.0 : arc_.back();}
  bool closed() const {return closed_;}
  const std::vector<Eigen::Vector3d> & points() const {return points_;}

  /// Position and unit tangent at `s` (saturated to [0, length]).
  Eigen::Vector3d position(double s) const;
  Eigen::Vector3d tangent(double s) const;

  /// Arc length up to the vertex closest to `p` (projection on the segments).
  double project(const Eigen::Vector3d & p) const;

private:
  std::size_t segment(double s) const;

  std::vector<Eigen::Vector3d> points_;  ///< Closed: the first one is repeated at the end
  std::vector<double> arc_;              ///< Cumulative arc at each vertex
  bool closed_ = false;
};

/// Centripetal Catmull-Rom spline (no loops or cusps) through the control points.
std::vector<Eigen::Vector3d> catmullRom(
  const std::vector<Eigen::Vector3d> & control, bool closed, int samples_per_segment = 16);

/// Speed profile knot: at arc `arc` the drone waits `hold` seconds and
/// goes on at `speed` until the next knot.
struct PathKnot
{
  double arc = 0.0;
  double hold = 0.0;
  double speed = 1.0;
};

/// Traversal of a curve at constant speed per section, with stops.
/// One pass lasts `duration()`; repetitions (laps) are up to the plan.
class Trajectory
{
public:
  Trajectory() = default;
  /// `knots` in increasing arc; the first must be at 0. Without knots: a single speed.
  Trajectory(SampledPath path, std::vector<PathKnot> knots);

  /// Stop of `duration` seconds at `point`.
  static Trajectory hover(const Eigen::Vector3d & point, double duration);
  /// Straight segment at constant speed.
  static Trajectory line(const Eigen::Vector3d & from, const Eigen::Vector3d & to, double speed);

  double duration() const {return knot_time_.empty() ? 0.0 : knot_time_.back();}
  const SampledPath & path() const {return path_;}
  Eigen::Vector3d start() const {return path_.position(0.0);}
  Eigen::Vector3d end() const {return path_.position(path_.length());}

  /// Arrival time at each knot (waypoints/timed: one per point, plus the end) [s].
  const std::vector<double> & knotTimes() const {return knot_time_;}

  /// Reference at `t` [s] since the start of the pass (saturated to [0, duration]).
  Reference sample(double t) const;

private:
  SampledPath path_;
  std::vector<PathKnot> knots_;   ///< + final knot at length()
  std::vector<double> knot_time_; ///< Arrival at each knot (before the wait); last = duration
};

}  // namespace aero_pursuit::scenario
