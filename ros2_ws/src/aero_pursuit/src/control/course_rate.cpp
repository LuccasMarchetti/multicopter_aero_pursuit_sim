#include "aero_pursuit/control/course_rate.hpp"

#include <cmath>

namespace aero_pursuit::control
{

void CourseRateEstimator::reset()
{
  course_ = std::numeric_limits<double>::quiet_NaN();
  rate_ = std::numeric_limits<double>::quiet_NaN();
}

double CourseRateEstimator::update(const Eigen::Vector3d & velocity, double dt)
{
  if (!velocity.allFinite() || velocity.head<2>().norm() < params_.min_speed) {
    reset();  // Hovering/slow: the course is noise
    return rate_;
  }
  const double course = std::atan2(velocity.y(), velocity.x());
  if (std::isfinite(course_) && dt > 1e-6) {
    const double d = std::atan2(std::sin(course - course_), std::cos(course - course_));
    const double sample = d / dt;
    const double k = dt / (params_.filter_time + dt);
    rate_ = std::isfinite(rate_) ? rate_ + k * (sample - rate_) : sample;
  }
  course_ = course;
  return rate_;
}

}  // namespace aero_pursuit::control
