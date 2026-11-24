#pragma once

#include "aero_pursuit/estimation/target_filter.hpp"

#include <Eigen/Dense>

namespace aero_pursuit::estimation
{

/// Singer-model 3D Kalman filter: state [p_rel, v_rel, a_target] (9), measurement p_rel.
/// The target acceleration is a first-order Gauss-Markov process
///   ȧ = -a/τ + w,  stationary standard deviation σ_a,
/// so τ → ∞ approaches constant acceleration and τ → 0 the constant-velocity model.
/// Position and velocity are relative to the pursuer (as in ConstantVelocityKF, whose
/// velocity change enters as a known input); the acceleration is the absolute one.
class SingerKF : public TargetFilter
{
public:
  using State = Eigen::Matrix<double, 9, 1>;
  using Covariance = Eigen::Matrix<double, 9, 9>;

  struct Params
  {
    double accel_std = 1.0;              ///< σ_a: stationary target acceleration std [m/s²]
    double accel_tau = 1.0;              ///< τ: acceleration correlation time [s]
    double measurement_noise_std = 0.05; ///< Position measurement std [m]
    double initial_velocity_std = 1.0;   ///< Initial velocity uncertainty [m/s]
  };

  SingerKF() : SingerKF(Params{}) {}
  explicit SingerKF(const Params & params) : params_(params) {}

  void setParams(const Params & params) {params_ = params;}
  const Params & params() const {return params_;}

  void reset(const Eigen::Vector3d & position) override;
  void invalidate() override {initialized_ = false;}
  bool initialized() const override {return initialized_;}
  void predict(double dt, const Eigen::Vector3d & frame_dv) override;
  bool update(const Eigen::Vector3d & z, double gate_chi2 = 0.0) override;

  Eigen::Vector3d position() const override {return x_.segment<3>(0);}
  Eigen::Vector3d velocity() const override {return x_.segment<3>(3);}
  Eigen::Vector3d acceleration() const override {return x_.segment<3>(6);}
  CovariancePV covariancePV() const override {return P_.topLeftCorner<6, 6>();}
  Eigen::Matrix3d covarianceA() const override {return P_.bottomRightCorner<3, 3>();}

private:
  Params params_;
  State x_ = State::Zero();
  Covariance P_ = Covariance::Identity();
  bool initialized_ = false;
};

}  // namespace aero_pursuit::estimation
