#include "aero_pursuit/control/gimbal_pitch.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace aero_pursuit::control;

namespace
{
constexpr double kDeg = M_PI / 180.0;
}

// Level body, target 10 m ahead and 2 m up: the camera tilts up by atan(2/10).
TEST(GimbalPitch, PointsUpAtATargetAbove)
{
  const Eigen::Quaterniond level = Eigen::Quaterniond::Identity();  // FLU = ENU (facing east)
  EXPECT_NEAR(gimbalPitchToward(Eigen::Vector3d(10, 0, 2), level), std::atan2(2.0, 10.0), 1e-9);
  EXPECT_NEAR(gimbalPitchToward(Eigen::Vector3d(10, 0, -3), level), std::atan2(-3.0, 10.0), 1e-9);
}

// Body pitched 10 deg nose down (accelerating forward), target level ahead: the gimbal tilts the
// camera 10 deg UP relative to the body, so it still looks at the target.
TEST(GimbalPitch, CompensatesTheBodyTilt)
{
  const Eigen::Quaterniond nose_down(Eigen::AngleAxisd(10.0 * kDeg, Eigen::Vector3d::UnitY()));
  EXPECT_NEAR(gimbalPitchToward(Eigen::Vector3d(10, 0, 0), nose_down), 10.0 * kDeg, 1e-9);
}

// Heading does not matter for the pitch: the same target seen by a body facing it.
TEST(GimbalPitch, IndependentOfTheHeading)
{
  const Eigen::Quaterniond facing_north(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
  EXPECT_NEAR(gimbalPitchToward(Eigen::Vector3d(0, 10, 2), facing_north), std::atan2(2.0, 10.0), 1e-9);
}

// The pivot is not at the body origin: seen from the pivot (0.165 m higher), a target at the
// origin height is slightly below.
TEST(GimbalPitch, AccountsForThePivotOffset)
{
  const Eigen::Vector3d mount(0.15, 0.03, 0.165);
  EXPECT_NEAR(gimbalPitchToward(Eigen::Vector3d(5, 0.03, 0), Eigen::Quaterniond::Identity(), mount),
    std::atan2(-0.165, 4.85), 1e-9);
}

TEST(GimbalPitch, StepTowardLimitsRateAndRange)
{
  EXPECT_NEAR(stepToward(0.0, 1.0, 0.5, 0.1, -1.5, 0.5), 0.05, 1e-12);    // rate limited
  EXPECT_NEAR(stepToward(0.49, 1.0, 10.0, 0.1, -1.5, 0.5), 0.5, 1e-12);  // clamped to the limit
  EXPECT_NEAR(stepToward(0.2, 0.21, 0.5, 0.1, -1.5, 0.5), 0.21, 1e-12);  // reaches it
}
