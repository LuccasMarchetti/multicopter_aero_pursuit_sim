#pragma once

#include <Eigen/Dense>

#include <limits>

namespace aero_pursuit::control
{

/// Turn rate of a horizontal velocity (course angle derivative), first-order filtered.
/// Used for the target: its estimated absolute velocity (pursuer velocity + relative
/// estimate) gives how fast it is turning, without knowing its heading.
class CourseRateEstimator
{
public:
  struct Params
  {
    double filter_time = 0.5;  ///< [s]
    double min_speed = 0.3;    ///< Below this the course is undefined [m/s]
  };

  CourseRateEstimator() : CourseRateEstimator(Params{}) {}
  explicit CourseRateEstimator(const Params & params) : params_(params) {}

  void reset();

  /// New velocity sample (ENU) after dt seconds. Returns the filtered rate (ENU, + left)
  /// [rad/s], NaN while unknown.
  double update(const Eigen::Vector3d & velocity, double dt);

  double rate() const {return rate_;}

private:
  Params params_;
  double course_ = std::numeric_limits<double>::quiet_NaN();
  double rate_ = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace aero_pursuit::control
