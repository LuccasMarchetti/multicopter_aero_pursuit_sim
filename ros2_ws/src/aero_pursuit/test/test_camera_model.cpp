#include "aero_pursuit/perception/camera_model.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using namespace aero_pursuit::perception;

TEST(CameraModel, RobustDepthIgnoresInvalidPixels)
{
  cv::Mat depth(10, 10, CV_32FC1, cv::Scalar(8.0f));
  depth(cv::Rect(0, 0, 10, 2)) = std::numeric_limits<float>::infinity();
  depth(cv::Rect(0, 2, 10, 1)) = 0.0f;
  depth(cv::Rect(0, 3, 10, 2)) = 4.0f;  // Target closer than the background

  std::vector<float> scratch;
  const auto d = robustDepth(depth, cv::Rect(0, 0, 10, 10), 10.0, 30.0, scratch);
  ASSERT_TRUE(d.has_value());
  EXPECT_FLOAT_EQ(*d, 4.0f);
  EXPECT_FALSE(robustDepth(depth, cv::Rect(0, 0, 10, 2), 10.0, 30.0, scratch).has_value());
}

TEST(CameraModel, BackprojectAndMapToDepth)
{
  const PinholeIntrinsics rgb{1000.0, 1000.0, 960.0, 540.0};
  const PinholeIntrinsics depth{500.0, 500.0, 320.0, 240.0};
  const auto p = rgb.backproject(1060.0, 540.0, 10.0);
  EXPECT_NEAR(p.x, 1.0, 1e-9);
  EXPECT_NEAR(p.y, 0.0, 1e-9);

  const cv::Rect b = mapBoxToDepth({960, 540, 100, 50}, {1920, 1080}, {640, 480}, rgb, depth);
  EXPECT_EQ(b, cv::Rect(320, 240, 50, 25));
}

TEST(CameraModel, BearingFromPixel)
{
  const PinholeIntrinsics k{500.0, 500.0, 320.0, 240.0};

  const auto c = bearingFromPixel(k, 320.0, 240.0);
  EXPECT_NEAR(c.azimuth, 0.0, 1e-12);
  EXPECT_NEAR(c.elevation, 0.0, 1e-12);
  EXPECT_NEAR(c.direction.z, 1.0, 1e-12);

  // Pixel right of and above the center: negative azimuth (right), positive elevation.
  const auto b = bearingFromPixel(k, 820.0, 240.0 - 500.0);
  EXPECT_NEAR(b.azimuth, -M_PI / 4.0, 1e-12);
  EXPECT_GT(b.elevation, 0.0);
  EXPECT_NEAR(cv::norm(b.direction), 1.0, 1e-12);
  // cos(angle to the axis) = cos(az)·cos(el)
  EXPECT_NEAR(b.direction.z, std::cos(b.azimuth) * std::cos(b.elevation), 1e-12);
}

TEST(CameraModel, DepthCoverageSeparatesRange)
{
  // Gazebo: +inf beyond far (19.1 m), −inf before near; far may also come saturated.
  cv::Mat depth(10, 10, CV_32FC1, cv::Scalar(std::numeric_limits<float>::infinity()));
  depth(cv::Rect(0, 0, 10, 1)) = 19.1f;                                      // saturated at far
  depth(cv::Rect(0, 1, 10, 1)) = -std::numeric_limits<float>::infinity();    // before near
  depth(cv::Rect(0, 2, 10, 1)) = 12.0f;                                      // valid
  const auto c = depthCoverage(depth, cv::Rect(0, 0, 10, 10), 0.2f, 19.1f);
  EXPECT_EQ(c.total, 100);
  EXPECT_EQ(c.valid, 10);
  EXPECT_EQ(c.below, 10);
  EXPECT_EQ(c.beyond, 80);
  std::vector<float> scratch;
  // Pixels saturated at far do not count as distance.
  const auto d = robustDepth(depth, cv::Rect(0, 0, 10, 2), 10.0, 30.0, scratch, 0.2f, 19.1f);
  EXPECT_FALSE(d.has_value());
}
