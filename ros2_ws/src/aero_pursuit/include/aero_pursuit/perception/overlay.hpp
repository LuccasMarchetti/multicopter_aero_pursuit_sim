#pragma once

#include <opencv2/core.hpp>

#include <string>

namespace aero_pursuit::perception
{

/// Text with a semi-transparent background for readability over the image.
void drawLabel(
  cv::Mat & img, const std::string & text, cv::Point origin, double font_scale,
  const cv::Scalar & color, int thickness = 2);

}  // namespace aero_pursuit::perception
