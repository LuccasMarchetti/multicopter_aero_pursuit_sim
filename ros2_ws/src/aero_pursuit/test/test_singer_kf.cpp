#include "aero_pursuit/control/pursuit_types.hpp"
#include "aero_pursuit/estimation/constant_velocity_kf.hpp"
#include "aero_pursuit/estimation/singer_kf.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>

using aero_pursuit::estimation::ConstantVelocityKF;
using aero_pursuit::estimation::SingerKF;
using aero_pursuit::estimation::TargetFilter;

namespace
{
/// Tracks a target on a circle (centripetal acceleration v²/R) with noisy position
/// measurements; returns the RMS position-prediction error 0.5 s ahead (the controller
/// horizon scale), after a warm-up.
double circlePredictionRms(TargetFilter & f, bool use_accel, double noise)
{
  const double R = 5.0, w = 0.4, dt = 1.0 / 30.0, ahead = 0.5;
  std::mt19937 rng(1);
  std::normal_distribution<double> n(0.0, noise);
  auto pos = [&](double t) {return Eigen::Vector3d(R * std::cos(w * t), R * std::sin(w * t), 0.0);};
  double sum = 0.0;
  int count = 0;
  for (int k = 0; k < 900; ++k) {
    const double t = k * dt;
    const Eigen::Vector3d z = pos(t) + Eigen::Vector3d(n(rng), n(rng), n(rng));
    if (k == 0) {
      f.reset(z);
      continue;
    }
    f.predict(dt, Eigen::Vector3d::Zero());
    f.update(z, 0.0);
    if (k > 300) {
      aero_pursuit::control::PursuitState s;
      s.p_rel = f.position();
      s.v_target = f.velocity();
      s.a_target = use_accel ? f.acceleration() : Eigen::Vector3d::Zero();
      s.accel_tau = 2.0;
      const Eigen::Vector3d predicted = s.p_rel + s.targetDisplacement(ahead);
      sum += (predicted - pos(t + ahead)).squaredNorm();
      ++count;
    }
  }
  return std::sqrt(sum / count);
}
}  // namespace

TEST(SingerKF, EstimatesConstantAcceleration)
{
  SingerKF::Params p;
  p.accel_std = 1.0;
  p.accel_tau = 5.0;
  p.measurement_noise_std = 0.01;
  SingerKF f(p);
  const Eigen::Vector3d a(0.8, -0.4, 0.2);
  const double dt = 0.02;
  for (int k = 0; k <= 400; ++k) {
    const double t = k * dt;
    const Eigen::Vector3d z = 0.5 * a * t * t + Eigen::Vector3d(1.0, 0.0, 0.0) * t;
    if (k == 0) {
      f.reset(z);
    } else {
      f.predict(dt, Eigen::Vector3d::Zero());
      f.update(z, 0.0);
    }
  }
  EXPECT_LT((f.acceleration() - a).norm(), 0.05);
  EXPECT_LT((f.velocity() - (Eigen::Vector3d(1.0, 0.0, 0.0) + a * 8.0)).norm(), 0.05);
}

TEST(SingerKF, FrameVelocityChangeIsAKnownInput)
{
  // Static target, pursuer accelerates: relative velocity follows -Δv_frame and the
  // absolute target acceleration stays zero.
  SingerKF f;
  f.reset(Eigen::Vector3d(10.0, 0.0, 0.0));
  const double dt = 0.05;
  Eigen::Vector3d chaser = Eigen::Vector3d::Zero(), v_chaser = Eigen::Vector3d::Zero();
  for (int k = 0; k < 100; ++k) {
    const Eigen::Vector3d dv(0.05, 0.0, 0.0);
    chaser += (v_chaser + 0.5 * dv) * dt;
    v_chaser += dv;
    f.predict(dt, dv);
    f.update(Eigen::Vector3d(10.0, 0.0, 0.0) - chaser, 0.0);
  }
  EXPECT_LT((f.velocity() + v_chaser).norm(), 0.05);
  EXPECT_LT(f.acceleration().norm(), 0.1);
}

TEST(SingerKF, BetterPredictionOnTurningTargetWithAcceleration)
{
  // Same measurements; the acceleration-aware prediction beats constant velocity on a
  // circle (the benefit shrinks as the measurement noise grows).
  ConstantVelocityKF::Params cp;
  cp.process_noise_std = 0.5;
  cp.measurement_noise_std = 0.05;
  ConstantVelocityKF cv(cp);
  SingerKF::Params sp;
  sp.accel_std = 1.0;
  sp.accel_tau = 2.0;
  sp.measurement_noise_std = 0.05;
  SingerKF singer(sp);
  const double e_cv = circlePredictionRms(cv, false, 0.05);
  const double e_singer = circlePredictionRms(singer, true, 0.05);
  EXPECT_LT(e_singer, 0.6 * e_cv) << "cv " << e_cv << " singer " << e_singer;
}

TEST(PursuitState, ZeroAccelerationKeepsConstantVelocityPrediction)
{
  aero_pursuit::control::PursuitState s;
  s.v_target = Eigen::Vector3d(1.0, 2.0, 0.0);
  EXPECT_TRUE(s.targetDisplacement(0.7).isApprox(s.v_target * 0.7));
  EXPECT_TRUE(s.targetVelocityAt(0.7).isApprox(s.v_target));
  s.a_target = Eigen::Vector3d(0.0, 0.0, 1.0);
  s.accel_tau = 1e6;  // ~constant acceleration: 0.5 a t²
  EXPECT_NEAR(s.targetDisplacement(2.0).z(), 2.0, 1e-3);
  EXPECT_NEAR(s.targetVelocityAt(2.0).z(), 2.0, 1e-3);
}
