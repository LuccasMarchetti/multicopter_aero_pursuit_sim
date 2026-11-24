#pragma once

#include <Eigen/Dense>

#include <cstddef>

namespace aero_pursuit::control
{

/// Quantitative performance metrics accumulated over an engagement:
/// RMSE of position (standoff point), of velocity and of the line-of-sight
/// angle, and control effort (∫|a|² dt and ∫|a| dt). Samples with a non-finite
/// angular error (LOS unavailable) are left out of the angular RMSE.
class PerformanceMetrics
{
public:
  void reset();

  void update(
    const Eigen::Vector3d & position_error, const Eigen::Vector3d & velocity_error,
    double los_error, const Eigen::Vector3d & acceleration, double dt);

  double rmsePosition() const;
  double rmseVelocity() const;
  double rmseLos() const;
  /// Off-axis angle of the target in the image, sqrt(az² + el²) [rad] (updateImage samples).
  double rmseImage() const;
  /// Image error sample (azimuth, elevation) [rad]; non-finite samples are left out.
  void updateImage(double azimuth, double elevation, double dt);
  double maxPositionError() const {return max_position_;}
  double effortL2() const {return effort_l2_;}  ///< ∫|a|² dt [m²/s³]
  double effortL1() const {return effort_l1_;}  ///< ∫|a| dt [m/s]
  double duration() const {return time_;}
  std::size_t samples() const {return samples_;}

private:
  double time_ = 0.0;
  double los_time_ = 0.0;
  double sum_pos2_ = 0.0;
  double sum_vel2_ = 0.0;
  double sum_los2_ = 0.0;
  double image_time_ = 0.0;
  double sum_image2_ = 0.0;
  double max_position_ = 0.0;
  double effort_l2_ = 0.0;
  double effort_l1_ = 0.0;
  std::size_t samples_ = 0;
};

}  // namespace aero_pursuit::control
