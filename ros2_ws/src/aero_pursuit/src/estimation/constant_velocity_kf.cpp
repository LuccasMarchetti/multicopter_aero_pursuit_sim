#include "aero_pursuit/estimation/constant_velocity_kf.hpp"

namespace aero_pursuit::estimation
{

ConstantVelocityKF::ConstantVelocityKF(const Params & params)
: params_(params) {}

void ConstantVelocityKF::reset(const Eigen::Vector3d & position)
{
  x_.head<3>() = position;
  x_.tail<3>().setZero();

  const double r_var = params_.measurement_noise_std * params_.measurement_noise_std;
  const double v_var = params_.initial_velocity_std * params_.initial_velocity_std;
  P_.setZero();
  P_.topLeftCorner<3, 3>().diagonal().setConstant(r_var);
  P_.bottomRightCorner<3, 3>().diagonal().setConstant(v_var);
  initialized_ = true;
}

void ConstantVelocityKF::predict(double dt)
{
  if (!initialized_ || dt <= 0.0) {
    return;
  }

  // x = F x, with F = [I dt*I; 0 I]
  x_.head<3>() += dt * x_.tail<3>();

  // P = F P F' + Q (discrete white acceleration)
  Covariance F = Covariance::Identity();
  F.topRightCorner<3, 3>().diagonal().setConstant(dt);

  const double q = params_.process_noise_std * params_.process_noise_std;
  const double dt2 = dt * dt;
  Covariance Q = Covariance::Zero();
  Q.topLeftCorner<3, 3>().diagonal().setConstant(0.25 * dt2 * dt2 * q);
  Q.topRightCorner<3, 3>().diagonal().setConstant(0.5 * dt2 * dt * q);
  Q.bottomLeftCorner<3, 3>().diagonal().setConstant(0.5 * dt2 * dt * q);
  Q.bottomRightCorner<3, 3>().diagonal().setConstant(dt2 * q);

  P_ = F * P_ * F.transpose() + Q;
}

void ConstantVelocityKF::predict(double dt, const Eigen::Vector3d & frame_dv)
{
  if (!initialized_ || dt <= 0.0) {
    return;
  }
  predict(dt);
  x_.head<3>() -= 0.5 * dt * frame_dv;
  x_.tail<3>() -= frame_dv;
}

bool ConstantVelocityKF::update(const Eigen::Vector3d & z, double gate_chi2)
{
  if (!initialized_) {
    reset(z);
    return true;
  }

  // H = [I 0] => the operations reduce to blocks of P.
  const double r_var = params_.measurement_noise_std * params_.measurement_noise_std;
  const Eigen::Vector3d y = z - x_.head<3>();
  Eigen::Matrix3d S = P_.topLeftCorner<3, 3>();
  S.diagonal().array() += r_var;

  const Eigen::LLT<Eigen::Matrix3d> S_llt(S);
  if (gate_chi2 > 0.0 && y.dot(S_llt.solve(y)) > gate_chi2) {
    return false;
  }

  // K = P H' S^-1
  const Eigen::Matrix<double, 6, 3> PHt = P_.leftCols<3>();
  const Eigen::Matrix<double, 6, 3> K = S_llt.solve(PHt.transpose()).transpose();

  x_ += K * y;
  P_ -= K * PHt.transpose();
  P_ = 0.5 * (P_ + P_.transpose());  // Keeps numerical symmetry
  return true;
}

}  // namespace aero_pursuit::estimation
