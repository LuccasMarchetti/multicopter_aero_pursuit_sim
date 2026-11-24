#pragma once

#include <Eigen/Dense>

namespace aero_pursuit::estimation
{

/// Target tracking filter in the pursuer-centred ENU frame: relative position and
/// velocity, plus (for models that have it) the absolute target acceleration.
class TargetFilter
{
public:
  using CovariancePV = Eigen::Matrix<double, 6, 6>;

  virtual ~TargetFilter() = default;

  virtual void reset(const Eigen::Vector3d & position) = 0;
  virtual void invalidate() = 0;
  virtual bool initialized() const = 0;
  /// Prediction; `frame_dv` = change of the pursuer velocity over the interval (known input).
  virtual void predict(double dt, const Eigen::Vector3d & frame_dv) = 0;
  /// Position update with optional chi² gating (0 = off). False if the measurement was rejected.
  virtual bool update(const Eigen::Vector3d & z, double gate_chi2) = 0;

  virtual Eigen::Vector3d position() const = 0;
  virtual Eigen::Vector3d velocity() const = 0;
  /// Absolute target acceleration (zero for the constant-velocity model).
  virtual Eigen::Vector3d acceleration() const {return Eigen::Vector3d::Zero();}
  virtual CovariancePV covariancePV() const = 0;
  virtual Eigen::Matrix3d covarianceA() const {return Eigen::Matrix3d::Zero();}
};

}  // namespace aero_pursuit::estimation
