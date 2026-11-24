#include "aero_pursuit/perception/appearance_model.hpp"

#include <gtest/gtest.h>

#include <opencv2/imgproc.hpp>

using aero_pursuit::perception::AppearanceModel;

namespace
{
/// Sky gradient with a dark "drone" (body + arms) at `at`; grass below y = 300.
cv::Mat scene(cv::Point at)
{
  cv::Mat img(480, 640, CV_8UC3);
  for (int y = 0; y < img.rows; ++y) {
    const cv::Scalar c = y < 300 ? cv::Scalar(235 - y / 6, 205 - y / 8, 170) : cv::Scalar(90, 160, 110);
    img.row(y).setTo(c);
  }
  cv::rectangle(img, cv::Rect(at.x - 12, at.y - 6, 24, 12), cv::Scalar(30, 30, 30), cv::FILLED);
  cv::line(img, at + cv::Point(-30, -10), at + cv::Point(30, 10), cv::Scalar(40, 40, 200), 3);
  cv::line(img, at + cv::Point(-30, 10), at + cv::Point(30, -10), cv::Scalar(200, 60, 40), 3);
  return img;
}
}  // namespace

TEST(AppearanceModel, SameObjectIsSimilarBackgroundIsNot)
{
  AppearanceModel m;
  const cv::Rect box(290, 170, 60, 40);
  m.reset(scene({320, 190}), box);
  const cv::Mat moved = scene({360, 200});
  const double same = m.similarity(moved, cv::Rect(330, 180, 60, 40));
  const double sky = m.similarity(moved, cv::Rect(100, 40, 60, 40));
  const double grass = m.similarity(moved, cv::Rect(100, 380, 60, 40));
  EXPECT_GT(same, 0.7);
  EXPECT_LT(sky, 0.35);
  EXPECT_LT(grass, 0.35);
}
