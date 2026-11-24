#include "aero_pursuit/estimation/constant_velocity_kf.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

using aero_pursuit::estimation::ConstantVelocityKF;

TEST(ConstantVelocityKF, ConvergesToConstantVelocity)
{
  ConstantVelocityKF kf;
  const Eigen::Vector3d v_true(1.0, -0.5, 0.2);
  const double dt = 0.033;
  Eigen::Vector3d p(5.0, 0.0, 1.0);
  kf.reset(p);
  for (int k = 0; k < 300; ++k) {
    p += v_true * dt;
    kf.predict(dt);
    ASSERT_TRUE(kf.update(p));
  }
  EXPECT_LT((kf.position() - p).norm(), 0.02);
  EXPECT_LT((kf.velocity() - v_true).norm(), 0.05);
}

TEST(ConstantVelocityKF, GateRejectsOutlier)
{
  ConstantVelocityKF kf;
  kf.reset(Eigen::Vector3d(5.0, 0.0, 0.0));
  for (int k = 0; k < 50; ++k) {
    kf.predict(0.033);
    kf.update(Eigen::Vector3d(5.0, 0.0, 0.0), 16.27);
  }
  kf.predict(0.033);
  EXPECT_FALSE(kf.update(Eigen::Vector3d(15.0, 0.0, 0.0), 16.27));
  EXPECT_NEAR(kf.position().x(), 5.0, 0.05);
}

TEST(ConstantVelocityKF, FrameVelocityInputTracksManeuveringChaser)
{
  // Stationary target; pursuer oscillating (±2 m/s). With the pursuer velocity
  // as a known input, the relative velocity follows the manoeuvre and the gating
  // does not reject measurements.
  ConstantVelocityKF kf;
  const double dt = 0.033;
  Eigen::Vector3d p_rel(5.0, 0.0, 0.0);
  Eigen::Vector3d v_c = Eigen::Vector3d::Zero();
  kf.reset(p_rel);
  double max_err = 0.0;
  for (int k = 0; k < 300; ++k) {
    const Eigen::Vector3d v_next(2.0 * std::sin(2.5 * (k + 1) * dt), 0.0, 0.0);
    p_rel -= 0.5 * dt * (v_c + v_next);
    kf.predict(dt, v_next - v_c);
    v_c = v_next;
    ASSERT_TRUE(kf.update(p_rel, 16.27));
    if (k > 30) {
      max_err = std::max(max_err, (kf.velocity() + v_c).norm());
    }
  }
  EXPECT_LT(max_err, 0.1);
}
