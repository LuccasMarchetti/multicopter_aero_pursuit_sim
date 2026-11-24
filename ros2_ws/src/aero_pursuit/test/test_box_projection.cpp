#include "aero_pursuit/perception/box_projection.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace aero_pursuit::perception;

namespace
{
// Optical ← model for a target in front of the camera, with no relative rotation beyond the axis swap
// (FLU model: x forward, y left, z up → optical: x right, y down, z forward).
cv::Matx33d opticalFromFlu() {return {0, -1, 0, 0, 0, -1, 1, 0, 0};}
}  // namespace

TEST(BoxProjection, OrientedBoxMatchesAnalyticSize)
{
  const PinholeIntrinsics k{1000.0, 1000.0, 960.0, 540.0};
  ModelBox b;
  b.size = {0.5, 0.8, 0.2};  // 0.8 m span (y), 0.2 m high
  const auto p = projectOrientedBox(k, opticalFromFlu(), {0.0, 0.0, 10.0}, b, {1920, 1080});
  ASSERT_TRUE(p.has_value());
  // Visible width = span (0.8 m) plus the perspective of the box depth (0.5 m).
  const double w_near = 1000.0 * 0.8 / (10.0 - 0.25);
  EXPECT_NEAR(p->box.width, w_near, 1e-6);
  EXPECT_NEAR(p->box.height, 1000.0 * 0.2 / (10.0 - 0.25), 1e-6);
  EXPECT_NEAR(p->box.x + p->box.width / 2.0, 960.0, 1e-6);
  EXPECT_TRUE(p->center_inside);
  EXPECT_NEAR(p->depth, 10.0, 1e-12);
}

TEST(BoxProjection, YawChangesApparentWidth)
{
  const PinholeIntrinsics k{1000.0, 1000.0, 960.0, 540.0};
  ModelBox b;
  b.size = {2.0, 0.2, 0.2};  // body long in x (forward)
  // Seen from the front: width ~ 0.2 m; rotated 90° in yaw: ~ 2 m.
  const auto front = projectOrientedBox(k, opticalFromFlu(), {0, 0, 20}, b, {1920, 1080});
  const cv::Matx33d yaw90{0, -1, 0, 1, 0, 0, 0, 0, 1};
  const auto side = projectOrientedBox(k, opticalFromFlu() * yaw90, {0, 0, 20}, b, {1920, 1080});
  ASSERT_TRUE(front && side);
  EXPECT_LT(front->box.width, 15.0);
  EXPECT_GT(side->box.width, 90.0);
}

TEST(BoxProjection, BehindCameraOrTooClose)
{
  const PinholeIntrinsics k{1000.0, 1000.0, 960.0, 540.0};
  ModelBox b;
  EXPECT_FALSE(projectOrientedBox(k, opticalFromFlu(), {0, 0, -5}, b, {1920, 1080}).has_value());
  EXPECT_FALSE(projectOrientedBox(k, opticalFromFlu(), {0, 0, 0.4}, b, {1920, 1080}, 0.0, 0.3)
    .has_value());
}
