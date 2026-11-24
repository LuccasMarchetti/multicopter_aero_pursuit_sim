#include "aero_pursuit/common/frames.hpp"

#include <gtest/gtest.h>

using namespace aero_pursuit::frames;

TEST(Frames, IdentityAttitudeFacesNorth)
{
  // PX4 yaw = 0 => nose to the North => FLU x points to ENU +y.
  const auto q = px4AttitudeToEnuFlu({1.0f, 0.0f, 0.0f, 0.0f});
  EXPECT_TRUE((q * Eigen::Vector3d::UnitX()).isApprox(Eigen::Vector3d::UnitY(), 1e-9));
  EXPECT_TRUE((q * Eigen::Vector3d::UnitY()).isApprox(-Eigen::Vector3d::UnitX(), 1e-9));
  EXPECT_TRUE((q * Eigen::Vector3d::UnitZ()).isApprox(Eigen::Vector3d::UnitZ(), 1e-9));
}

TEST(Frames, YawEastAndRoundTrip)
{
  // NED yaw = +90° (East) => FLU x points to ENU +x.
  const float s = static_cast<float>(M_SQRT1_2);
  const auto q = px4AttitudeToEnuFlu({s, 0.0f, 0.0f, s});
  EXPECT_TRUE((q * Eigen::Vector3d::UnitX()).isApprox(Eigen::Vector3d::UnitX(), 1e-6));
  EXPECT_NEAR(enuDirectionToNedYaw(Eigen::Vector3d::UnitX()), M_PI_2, 1e-12);

  const Eigen::Vector3d v(1.0, 2.0, 3.0);
  EXPECT_TRUE(nedToEnu(enuToNed(v)).isApprox(v));
}
