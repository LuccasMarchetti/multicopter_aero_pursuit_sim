#include "aero_pursuit/perception/overlay.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>

namespace aero_pursuit::perception
{

void drawLabel(
  cv::Mat & img, const std::string & text, cv::Point origin, double font_scale,
  const cv::Scalar & color, int thickness)
{
  constexpr int kFont = cv::FONT_HERSHEY_SIMPLEX;
  int baseline = 0;
  const cv::Size size = cv::getTextSize(text, kFont, font_scale, thickness, &baseline);

  const cv::Rect bg = cv::Rect(
    cv::Point(origin.x - 4, origin.y - size.height - 4),
    cv::Point(origin.x + size.width + 4, origin.y + baseline + 4)) &
    cv::Rect(0, 0, img.cols, img.rows);
  if (!bg.empty()) {
    cv::Mat roi = img(bg);
    roi *= 0.4;  // Darkens the background (equivalent to a 60% black blend)
  }

  cv::putText(img, text, origin, kFont, font_scale, cv::Scalar::all(0), thickness + 2, cv::LINE_AA);
  cv::putText(img, text, origin, kFont, font_scale, color, thickness, cv::LINE_AA);
}

}  // namespace aero_pursuit::perception
