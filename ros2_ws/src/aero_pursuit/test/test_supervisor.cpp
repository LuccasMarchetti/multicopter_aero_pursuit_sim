#include "aero_pursuit/control/pursuit_supervisor.hpp"

#include <gtest/gtest.h>

using aero_pursuit::control::PursuitMode;
using aero_pursuit::control::PursuitSupervisor;

TEST(PursuitSupervisor, FullLifecycle)
{
  PursuitSupervisor sup({0.3, 1.5});

  EXPECT_EQ(sup.update(true, std::nullopt), PursuitMode::kIdle);

  EXPECT_EQ(sup.update(true, 0.05), PursuitMode::kTracking);
  EXPECT_TRUE(sup.engaged());

  EXPECT_EQ(sup.update(true, 0.1), PursuitMode::kTracking);
  EXPECT_FALSE(sup.engaged());

  EXPECT_EQ(sup.update(true, 0.8), PursuitMode::kCoasting);
  EXPECT_FALSE(sup.engaged());

  EXPECT_EQ(sup.update(true, 2.0), PursuitMode::kLost);
  EXPECT_FALSE(sup.active());

  // An old estimate does not re-engage; only a new measurement does.
  EXPECT_EQ(sup.update(true, 1.0), PursuitMode::kLost);
  EXPECT_EQ(sup.update(true, 0.02), PursuitMode::kTracking);
  EXPECT_TRUE(sup.engaged());

  EXPECT_EQ(sup.update(false, 0.02), PursuitMode::kDisabled);
  EXPECT_FALSE(sup.active());
}

TEST(PursuitSupervisor, StaleTargetNeverEngages)
{
  PursuitSupervisor sup({0.3, 1.5});
  EXPECT_EQ(sup.update(true, 1.0), PursuitMode::kIdle);
  EXPECT_FALSE(sup.active());
}

TEST(PursuitSupervisor, BearingFallbackApproachesUntilDepthReturns)
{
  PursuitSupervisor sup({0.3, 1.5, 0.5});

  // Distant target: only a line of sight (no depth) => APPROACH engages.
  EXPECT_EQ(sup.update(true, std::nullopt, 0.05), PursuitMode::kApproach);
  EXPECT_TRUE(sup.engaged());
  EXPECT_TRUE(sup.active());

  // Depth comes back: TRACKING without re-engaging (reference continuity).
  EXPECT_EQ(sup.update(true, 0.02, 0.05), PursuitMode::kTracking);
  EXPECT_FALSE(sup.engaged());
  EXPECT_EQ(sup.previous(), PursuitMode::kApproach);

  // Depth disappears again, recent direction: APPROACH has priority over COASTING.
  EXPECT_EQ(sup.update(true, 0.6, 0.05), PursuitMode::kApproach);

  // The direction also ages: does not extrapolate an old 3D estimate from APPROACH.
  EXPECT_EQ(sup.update(true, 0.9, 0.8), PursuitMode::kLost);
}
