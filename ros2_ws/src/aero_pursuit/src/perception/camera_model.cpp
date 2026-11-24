#include "aero_pursuit/perception/camera_model.hpp"

#include <algorithm>
#include <cmath>

namespace aero_pursuit::perception
{

cv::Rect mapBoxToDepth(
  const cv::Rect & box_rgb, const cv::Size & rgb_size, const cv::Size & depth_size,
  const PinholeIntrinsics & rgb, const PinholeIntrinsics & depth)
{
  if (rgb.valid() && depth.valid()) {
    const double sx = depth.fx / rgb.fx;
    const double sy = depth.fy / rgb.fy;
    const int x1 = cvRound((box_rgb.x - rgb.cx) * sx + depth.cx);
    const int y1 = cvRound((box_rgb.y - rgb.cy) * sy + depth.cy);
    const int x2 = cvRound((box_rgb.x + box_rgb.width - rgb.cx) * sx + depth.cx);
    const int y2 = cvRound((box_rgb.y + box_rgb.height - rgb.cy) * sy + depth.cy);
    return {x1, y1, x2 - x1, y2 - y1};
  }

  const double sx = static_cast<double>(depth_size.width) / rgb_size.width;
  const double sy = static_cast<double>(depth_size.height) / rgb_size.height;
  return {
    cvRound(box_rgb.x * sx), cvRound(box_rgb.y * sy),
    cvRound(box_rgb.width * sx), cvRound(box_rgb.height * sy)};
}

Bearing bearingFromPixel(const PinholeIntrinsics & k, double u, double v)
{
  const double xn = (u - k.cx) / k.fx;
  const double yn = (v - k.cy) / k.fy;
  const double n = std::sqrt(1.0 + xn * xn + yn * yn);

  Bearing b;
  b.direction = {xn / n, yn / n, 1.0 / n};
  b.azimuth = std::atan(-xn);
  b.elevation = std::atan2(-yn, std::hypot(1.0, xn));
  return b;
}

std::optional<float> robustDepth(
  const cv::Mat & depth_m, const cv::Rect & box, double percentile_lower,
  double percentile_upper, std::vector<float> & scratch, float min_range, float max_range)
{
  const cv::Rect roi = box & cv::Rect(0, 0, depth_m.cols, depth_m.rows);
  if (roi.empty() || depth_m.type() != CV_32FC1) {
    return std::nullopt;
  }

  scratch.clear();
  scratch.reserve(static_cast<size_t>(roi.area()));
  for (int y = roi.y; y < roi.y + roi.height; ++y) {
    const float * row = depth_m.ptr<float>(y);
    for (int x = roi.x; x < roi.x + roi.width; ++x) {
      const float d = row[x];
      if (std::isfinite(d) && d >= min_range && d < max_range) {
        scratch.push_back(d);
      }
    }
  }
  if (scratch.empty()) {
    return std::nullopt;
  }

  const size_t n = scratch.size();
  const size_t lo = std::min(
    static_cast<size_t>(std::floor(percentile_lower / 100.0 * n)), n - 1);
  const size_t hi = std::clamp(
    static_cast<size_t>(std::ceil(percentile_upper / 100.0 * n)), lo + 1, n);
  const size_t mid = lo + (hi - lo) / 2;

  std::nth_element(scratch.begin(), scratch.begin() + mid, scratch.end());
  return scratch[mid];
}

DepthCoverage depthCoverage(const cv::Mat & depth_m, const cv::Rect & box, float min_range,
  float max_range)
{
  DepthCoverage c;
  const cv::Rect roi = box & cv::Rect(0, 0, depth_m.cols, depth_m.rows);
  if (roi.empty() || depth_m.type() != CV_32FC1) {
    return c;
  }
  // far may come saturated (= max_range) instead of +inf: 1 mm tolerance.
  const float far = max_range - 1e-3f;
  for (int y = roi.y; y < roi.y + roi.height; ++y) {
    const float * row = depth_m.ptr<float>(y);
    for (int x = roi.x; x < roi.x + roi.width; ++x) {
      const float d = row[x];
      ++c.total;
      if (std::isnan(d)) {
        continue;
      }
      if (d >= far) {
        ++c.beyond;
      } else if (d < min_range) {
        ++c.below;
      } else {
        ++c.valid;
      }
    }
  }
  return c;
}

}  // namespace aero_pursuit::perception
