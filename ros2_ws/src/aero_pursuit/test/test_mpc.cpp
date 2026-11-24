#include "aero_pursuit/control/mpc.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <string>
#include <thread>

using namespace aero_pursuit::control;

namespace
{
constexpr double kDt = 0.05;  // node cycle

/// Target azimuth in the image of the forward camera (0 = centered, + = left) [rad].
double azimuth(const PursuitState & s)
{
  const double c = std::cos(s.heading), sn = std::sin(s.heading);
  return std::atan2(-sn * s.p_rel.x() + c * s.p_rel.y(), c * s.p_rel.x() + sn * s.p_rel.y());
}

struct Trace
{
  double max_abs_ax = 0.0;
  double max_abs_r = 0.0;
  double max_abs_azimuth = 0.0;  ///< after the first second
  double first_r = 0.0;
};

/// Closed loop with the MPC's own model: target at constant velocity v_t, pursuer driven by the
/// commanded acceleration, heading following the commanded yaw (as the PX4 does). Returns the
/// final state.
PursuitState simulate(Mpc & mpc, PursuitState s, double seconds, Trace * tr = nullptr)
{
  const int steps = static_cast<int>(seconds / kDt);
  for (int i = 0; i < steps; ++i) {
    const auto cmd = mpc.compute(s, kDt);
    EXPECT_EQ(mpc.lastStatus(), 0);
    if (tr) {
      tr->max_abs_ax = std::max(tr->max_abs_ax, std::abs(cmd.acceleration.x()));
      tr->max_abs_r = std::max(tr->max_abs_r, std::abs(cmd.yaw_rate));
      if (i == 0) {
        tr->first_r = cmd.yaw_rate;
      }
      if (i * kDt >= 1.0) {
        tr->max_abs_azimuth = std::max(tr->max_abs_azimuth, std::abs(azimuth(s)));
      }
    }
    // Relative dynamics: p_rel += v_rel dt - a dt²/2, v_rel -= a dt.
    s.p_rel += s.v_rel * kDt - 0.5 * cmd.acceleration * kDt * kDt;
    s.v_rel -= cmd.acceleration * kDt;
    if (cmd.yaw) {
      s.heading = *cmd.yaw;
    }
  }
  return s;
}
}  // namespace

TEST(Mpc, ConvergesToStandoffBehindStaticTarget)
{
  Mpc mpc;
  PursuitState s;
  s.standoff = 5.0;
  s.p_rel = Eigen::Vector3d(12.0, 3.0, 1.0);
  const auto end = simulate(mpc, s, 15.0);
  EXPECT_NEAR(end.p_rel.norm(), 5.0, 0.05);
  EXPECT_LT(end.v_rel.norm(), 0.05);
}

TEST(Mpc, MatchesMovingTargetVelocity)
{
  Mpc mpc;
  PursuitState s;
  s.standoff = 5.0;
  s.p_rel = Eigen::Vector3d(10.0, 0.0, 0.0);
  s.v_target = Eigen::Vector3d(1.0, 0.5, 0.0);
  s.v_rel = s.v_target;  // pursuer starts at rest
  const auto end = simulate(mpc, s, 20.0);
  EXPECT_NEAR(end.p_rel.norm(), 5.0, 0.1);
  EXPECT_LT(end.v_rel.norm(), 0.05);
}

TEST(Mpc, RespectsAccelerationBoundFromGcs)
{
  Mpc mpc;
  mpc.setParameter("a_max_xy", 1.0);
  PursuitState s;
  s.p_rel = Eigen::Vector3d(60.0, 0.0, 0.0);  // far: the bound is active
  Trace tr;
  simulate(mpc, s, 2.0, &tr);
  EXPECT_LE(tr.max_abs_ax, 1.0 + 1e-6);
  EXPECT_GT(tr.max_abs_ax, 0.99);

  mpc.eraseParameter("a_max_xy");  // back to the default compiled from pursuit_ocp.py (3)
  Trace tr2;
  simulate(mpc, s, 0.5, &tr2);
  EXPECT_GT(tr2.max_abs_ax, 2.9);
}

TEST(Mpc, HorizontalReferenceMatchesTargetAltitude)
{
  Mpc mpc;  // ref_horizontal defaults to 1
  PursuitState s;
  s.standoff = 5.0;
  s.p_rel = Eigen::Vector3d(8.0, 2.0, -3.0);  // target 3 m below
  const auto end = simulate(mpc, s, 15.0);
  EXPECT_NEAR(end.p_rel.z(), 0.0, 0.05);                 // same altitude
  EXPECT_NEAR(end.p_rel.head<2>().norm(), 5.0, 0.05);    // standoff measured horizontally
}

// 3D mode: the standoff is measured along the 3D line of sight. The reference turns with the line
// of sight every cycle, so the elevation is not held exactly (the drift seen in simulation).
TEST(Mpc, LineOfSightReferenceUses3dDistance)
{
  Mpc mpc;
  mpc.setParameter("ref_horizontal", 0.0);
  PursuitState s;
  s.standoff = 5.0;
  s.p_rel = Eigen::Vector3d(8.0, 0.0, -6.0);  // 10 m along a LOS 36.9 deg down
  const auto end = simulate(mpc, s, 15.0);
  EXPECT_NEAR(end.p_rel.norm(), 5.0, 0.05);
  EXPECT_NEAR(std::atan2(-end.p_rel.z(), end.p_rel.x()), std::atan2(6.0, 8.0), 0.05);
  EXPECT_LT(end.p_rel.z(), -2.0);  // still well below: the altitude is not matched
}

// N/dt from the GCS: a variant generated and compiled at run time (~3 s), then a switch; going
// back to the compiled-in default is immediate.
TEST(Mpc, SwitchesHorizonAtRunTime)
{
  Mpc mpc;
  EXPECT_EQ(mpc.horizonSteps(), 20);
  mpc.setParameter("N", 30.0);
  mpc.setParameter("dt", 0.05);
  PursuitState s;
  s.p_rel = Eigen::Vector3d(10.0, 0.0, 0.0);
  std::string log;
  for (int i = 0; i < 1200 && mpc.horizonSteps() != 30; ++i) {  // up to 60 s
    mpc.compute(s, kDt);
    for (const auto & m : mpc.takeMessages()) {
      log += m + "\n";
    }
    if (log.find("cannot generate") != std::string::npos) {
      GTEST_SKIP() << "no acados generator on this computer: " << log;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  ASSERT_EQ(mpc.horizonSteps(), 30) << log;
  EXPECT_DOUBLE_EQ(mpc.horizonStep(), 0.05);
  const auto cmd = mpc.compute(s, kDt);
  EXPECT_EQ(mpc.lastStatus(), 0);
  EXPECT_EQ(cmd.predicted_path.size(), 30u);
  EXPECT_NE(mpc.label().find("N=30"), std::string::npos);

  // A second run-time variant while one compiled at run time is active (regression: the .so of
  // a variant being built was loaded twice and the active one unloaded under the solver).
  mpc.setParameter("N", 25.0);
  for (int i = 0; i < 1200 && mpc.horizonSteps() != 25; ++i) {
    mpc.compute(s, kDt);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  ASSERT_EQ(mpc.horizonSteps(), 25);
  EXPECT_EQ(mpc.compute(s, kDt).predicted_path.size(), 25u);
  EXPECT_EQ(mpc.lastStatus(), 0);

  mpc.eraseParameter("N");
  mpc.eraseParameter("dt");
  mpc.compute(s, kDt);
  EXPECT_EQ(mpc.horizonSteps(), 20);
}

// Option B: the heading is a state tracked toward the predicted line of sight.
TEST(Mpc, YawTrackingTurnsToTheTarget)
{
  Mpc mpc;
  mpc.setParameter("yaw_mode", 1.0);
  PursuitState s;
  s.heading = 0.0;                            // facing east
  s.p_rel = Eigen::Vector3d(0.0, 10.0, 0.0);  // target to the north
  Trace tr;
  const auto end = simulate(mpc, s, 6.0, &tr);
  EXPECT_NEAR(end.heading, M_PI_2, 0.03);
  EXPECT_LE(tr.max_abs_r, 1.2 + 1e-6);        // r_max
  EXPECT_GT(tr.first_r, 0.0);                 // counterclockwise
}

// Unwrap: heading 3.0 rad, target at -3.0 rad: the short way is +0.28 rad across +-pi.
TEST(Mpc, YawTrackingTakesTheShortWayAcrossPi)
{
  Mpc mpc;
  mpc.setParameter("yaw_mode", 1.0);
  PursuitState s;
  s.heading = 3.0;
  s.p_rel = 10.0 * Eigen::Vector3d(std::cos(-3.0), std::sin(-3.0), 0.0);
  Trace tr;
  const auto end = simulate(mpc, s, 3.0, &tr);
  EXPECT_GT(tr.first_r, 0.0);                          // counterclockwise, through pi
  EXPECT_LT(std::abs(std::remainder(end.heading - std::atan2(end.p_rel.y(), end.p_rel.x()), 2 * M_PI)), 0.05);
}

// Option C: the target starts outside the usable field of view and the yaw is slow (r_max
// small). B only turns (the translation ignores the camera); C also moves the pursuer so the
// target comes back into the image sooner.
TEST(Mpc, FieldOfViewConstraintBringsTheTargetIntoTheImage)
{
  auto run = [](double yaw_mode) {
      Mpc mpc;
      mpc.setParameter("yaw_mode", yaw_mode);
      mpc.setParameter("r_max", 0.15);
      PursuitState s;
      s.heading = 0.0;
      const double bearing = 50.0 * M_PI / 180.0;   // 50 deg to the left, half FOV is 34.5
      s.p_rel = 8.0 * Eigen::Vector3d(std::cos(bearing), std::sin(bearing), 0.0);
      Trace tr;
      double out_time = 0.0;                         // time with the target outside the image
      for (int i = 0; i < static_cast<int>(6.0 / kDt); ++i) {
        s = simulate(mpc, s, kDt, &tr);
        if (std::abs(azimuth(s)) > 34.5 * M_PI / 180.0) {
          out_time += kDt;
        }
      }
      return out_time;
    };
  const double out_b = run(1.0);
  const double out_c = run(2.0);
  std::printf("time with the target outside the image: B %.2f s, C %.2f s\n", out_b, out_c);
  EXPECT_GT(out_b, 0.5);
  EXPECT_LT(out_c, 0.5 * out_b);
}

// Mode 3: the relative heading is optimized inside the solver (cos/sin residuals).
TEST(Mpc, OptimalYawTurnsToTheTarget)
{
  Mpc mpc;
  mpc.setParameter("yaw_mode", 3.0);
  PursuitState s;
  s.heading = 0.0;
  s.p_rel = Eigen::Vector3d(0.0, 10.0, 0.0);
  Trace tr;
  const auto end = simulate(mpc, s, 6.0, &tr);
  EXPECT_NEAR(std::remainder(end.heading - std::atan2(end.p_rel.y(), end.p_rel.x()), 2 * M_PI), 0.0, 0.03);
  EXPECT_LE(tr.max_abs_r, 1.2 + 1e-6);
  EXPECT_GT(tr.first_r, 0.0);
}

// Mode 3 needs no unwrap: the cos/sin residuals are periodic; across +-pi it takes the short way.
TEST(Mpc, OptimalYawTakesTheShortWayAcrossPi)
{
  Mpc mpc;
  mpc.setParameter("yaw_mode", 3.0);
  PursuitState s;
  s.heading = 3.0;
  s.p_rel = 10.0 * Eigen::Vector3d(std::cos(-3.0), std::sin(-3.0), 0.0);
  Trace tr;
  const auto end = simulate(mpc, s, 3.0, &tr);
  EXPECT_GT(tr.first_r, 0.0);
  EXPECT_LT(std::abs(std::remainder(end.heading - std::atan2(end.p_rel.y(), end.p_rel.x()), 2 * M_PI)), 0.05);
}

// B vs optimal vs their field-of-view versions: target 50 deg to the side, slow yaw. Time with the
// target outside the image (printed for the comparison); the optimal cost and the constraint
// both use the translation to help.
TEST(Mpc, YawModesComparisonOutOfImage)
{
  auto run = [](double yaw_mode) {
      Mpc mpc;
      mpc.setParameter("yaw_mode", yaw_mode);
      mpc.setParameter("r_max", 0.15);
      PursuitState s;
      s.heading = 0.0;
      const double bearing = 50.0 * M_PI / 180.0;
      s.p_rel = 8.0 * Eigen::Vector3d(std::cos(bearing), std::sin(bearing), 0.0);
      Trace tr;
      double out_time = 0.0;
      for (int i = 0; i < static_cast<int>(6.0 / kDt); ++i) {
        s = simulate(mpc, s, kDt, &tr);
        if (std::abs(azimuth(s)) > 34.5 * M_PI / 180.0) {
          out_time += kDt;
        }
      }
      return out_time;
    };
  const double b = run(1.0), b_fov = run(2.0), opt = run(3.0), opt_fov = run(4.0);
  std::printf("outside the image: B %.2f s | B+FOV %.2f s | optimal %.2f s | optimal+FOV %.2f s\n",
    b, b_fov, opt, opt_fov);
  EXPECT_LE(opt, b + 1e-9);        // optimizing the relative heading never does worse than B here
  EXPECT_LT(opt_fov, b);
}

TEST(Mpc, PredictedPathCoversTheHorizon)
{
  Mpc mpc;
  PursuitState s;
  s.p_rel = Eigen::Vector3d(10.0, 0.0, 0.0);
  const auto cmd = mpc.compute(s, kDt);
  EXPECT_EQ(cmd.predicted_path.size(), 20u);
  EXPECT_GT(cmd.predicted_path.back().x(), 0.5);  // moves toward the target
  EXPECT_GT(cmd.acceleration.x(), 0.0);
  EXPECT_FALSE(cmd.yaw.has_value());              // yaw is not the MPC's (held by the node)
}

TEST(Mpc, WithoutDepthCommandsHover)
{
  auto law = std::make_unique<Mpc>();
  PursuitState s;
  s.has_position = false;
  s.box = TargetBox{};
  const auto cmd = law->compute(s, kDt);
  EXPECT_TRUE(cmd.velocity.isZero());
  EXPECT_EQ(law->name(), "mpc");
}

