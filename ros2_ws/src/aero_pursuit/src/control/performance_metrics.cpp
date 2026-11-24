#include "aero_pursuit/control/performance_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace aero_pursuit::control
{

namespace
{
double rms(double sum_sq, double time)
{
  return time > 0.0 ? std::sqrt(sum_sq / time) : std::numeric_limits<double>::quiet_NaN();
}
}  // namespace

void PerformanceMetrics::reset()
{
  *this = PerformanceMetrics{};
}

void PerformanceMetrics::update(
  const Eigen::Vector3d & position_error, const Eigen::Vector3d & velocity_error,
  double los_error, const Eigen::Vector3d & acceleration, double dt)
{
  if (dt <= 0.0) {
    return;
  }
  // Time integrals (robust to cycle jitter): RMSE = sqrt(∫e² dt / T).
  time_ += dt;
  sum_pos2_ += position_error.squaredNorm() * dt;
  sum_vel2_ += velocity_error.squaredNorm() * dt;
  if (std::isfinite(los_error)) {
    los_time_ += dt;
    sum_los2_ += los_error * los_error * dt;
  }
  max_position_ = std::max(max_position_, position_error.norm());
  const double a = acceleration.norm();
  effort_l2_ += a * a * dt;
  effort_l1_ += a * dt;
  ++samples_;
}

double PerformanceMetrics::rmsePosition() const {return rms(sum_pos2_, time_);}
double PerformanceMetrics::rmseVelocity() const {return rms(sum_vel2_, time_);}
double PerformanceMetrics::rmseLos() const {return rms(sum_los2_, los_time_);}
double PerformanceMetrics::rmseImage() const {return rms(sum_image2_, image_time_);}

void PerformanceMetrics::updateImage(double azimuth, double elevation, double dt)
{
  if (dt <= 0.0 || !std::isfinite(azimuth) || !std::isfinite(elevation)) {
    return;
  }
  image_time_ += dt;
  sum_image2_ += (azimuth * azimuth + elevation * elevation) * dt;
}

}  // namespace aero_pursuit::control
