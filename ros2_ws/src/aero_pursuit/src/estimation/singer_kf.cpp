#include "aero_pursuit/estimation/singer_kf.hpp"

#include <algorithm>
#include <cmath>

namespace aero_pursuit::estimation
{

void SingerKF::reset(const Eigen::Vector3d & position)
{
  x_.setZero();
  x_.segment<3>(0) = position;
  const double r = params_.measurement_noise_std * params_.measurement_noise_std;
  const double v = params_.initial_velocity_std * params_.initial_velocity_std;
  const double a = params_.accel_std * params_.accel_std;
  P_.setZero();
  P_.block<3, 3>(0, 0).diagonal().setConstant(r);
  P_.block<3, 3>(3, 3).diagonal().setConstant(v);
  P_.block<3, 3>(6, 6).diagonal().setConstant(a);
  initialized_ = true;
}

void SingerKF::predict(double T, const Eigen::Vector3d & frame_dv)
{
  if (!initialized_ || T <= 0.0) {
    return;
  }
  const double tau = std::max(params_.accel_tau, 1e-3);
  const double alpha = 1.0 / tau;
  const double e = std::exp(-alpha * T);
  // Per-axis transition (Singer, 1970).
  Eigen::Matrix3d f;
  f << 1.0, T, (alpha * T - 1.0 + e) / (alpha * alpha),
    0.0, 1.0, (1.0 - e) / alpha,
    0.0, 0.0, e;
  // Process noise: continuous white jerk density q = 2 σ_a² / τ; for the camera/TF
  // periods (T ≪ τ) the Singer Q equals the Wiener-jerk Q below to first order.
  const double q = 2.0 * params_.accel_std * params_.accel_std * alpha;
  const double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
  Eigen::Matrix3d qa;
  qa << T5 / 20.0, T4 / 8.0, T3 / 6.0,
    T4 / 8.0, T3 / 3.0, T2 / 2.0,
    T3 / 6.0, T2 / 2.0, T;
  qa *= q;

  Covariance F = Covariance::Zero();
  Covariance Q = Covariance::Zero();
  for (int i = 0; i < 3; ++i) {
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        F(3 * r + i, 3 * c + i) = f(r, c);
        Q(3 * r + i, 3 * c + i) = qa(r, c);
      }
    }
  }
  x_ = F * x_;
  // Known input: the frame (pursuer) velocity changed linearly by frame_dv.
  x_.segment<3>(0) -= 0.5 * T * frame_dv;
  x_.segment<3>(3) -= frame_dv;
  P_ = F * P_ * F.transpose() + Q;
}

bool SingerKF::update(const Eigen::Vector3d & z, double gate_chi2)
{
  if (!initialized_) {
    reset(z);
    return true;
  }
  const double r = params_.measurement_noise_std * params_.measurement_noise_std;
  const Eigen::Vector3d y = z - x_.segment<3>(0);
  Eigen::Matrix3d S = P_.topLeftCorner<3, 3>();
  S.diagonal().array() += r;
  const Eigen::LLT<Eigen::Matrix3d> S_llt(S);
  if (gate_chi2 > 0.0 && y.dot(S_llt.solve(y)) > gate_chi2) {
    return false;
  }
  const Eigen::Matrix<double, 9, 3> PHt = P_.leftCols<3>();
  const Eigen::Matrix<double, 9, 3> K = S_llt.solve(PHt.transpose()).transpose();
  x_ += K * y;
  P_ -= K * PHt.transpose();
  P_ = 0.5 * (P_ + P_.transpose());
  return true;
}

}  // namespace aero_pursuit::estimation
