#pragma once

#include "aero_pursuit/estimation/target_filter.hpp"

#include <Eigen/Dense>

namespace aero_pursuit::estimation
{

/// Constant-velocity 3D Kalman filter (state [p, v], measurement [p]).
/// Implementation with fixed-size matrices (no dynamic allocation).
class ConstantVelocityKF : public TargetFilter
{
public:
  using State = Eigen::Matrix<double, 6, 1>;
  using Covariance = Eigen::Matrix<double, 6, 6>;

  struct Params
  {
    double process_noise_std = 0.5;      ///< White acceleration standard deviation [m/s²]
    double measurement_noise_std = 0.05; ///< Position measurement standard deviation [m]
    double initial_velocity_std = 1.0;   ///< Initial velocity uncertainty [m/s]
  };

  ConstantVelocityKF() : ConstantVelocityKF(Params{}) {}
  explicit ConstantVelocityKF(const Params & params);

  void setParams(const Params & params) {params_ = params;}
  const Params & params() const {return params_;}

  void reset(const Eigen::Vector3d & position) override;
  void invalidate() override {initialized_ = false;}
  bool initialized() const override {return initialized_;}

  void predict(double dt);

  /// Prediction with a known input: the state is relative to a frame that
  /// accelerates (the pursuer) and whose velocity changed by `frame_dv` over the interval.
  /// With a linear change of the frame velocity:
  ///   p += dt v - dt/2 Δv_ref,   v -= Δv_ref
  /// This way the process noise models only the target acceleration, not that of the
  /// pursuer itself (which is measured).
  void predict(double dt, const Eigen::Vector3d & frame_dv) override;

  /// Corrects with a position measurement. If gate_chi2 > 0, rejects measurements whose
  /// squared Mahalanobis distance exceeds the threshold. Returns true if the measurement was accepted.
  bool update(const Eigen::Vector3d & z, double gate_chi2 = 0.0) override;

  Eigen::Vector3d position() const override {return x_.head<3>();}
  Eigen::Vector3d velocity() const override {return x_.tail<3>();}
  CovariancePV covariancePV() const override {return P_;}
  const State & state() const {return x_;}
  const Covariance & covariance() const {return P_;}

private:
  Params params_;
  State x_ = State::Zero();
  Covariance P_ = Covariance::Identity();
  bool initialized_ = false;
};

}  // namespace aero_pursuit::estimation
