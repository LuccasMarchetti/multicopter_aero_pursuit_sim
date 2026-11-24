#include "aero_pursuit/perception/appearance_model.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace aero_pursuit::perception
{

AppearanceModel::Features AppearanceModel::features(const cv::Mat & bgr, const cv::Rect & box) const
{
  Features f;
  const double c = std::clamp(params_.center_fraction, 0.1, 1.0);
  const int w = std::max(4, static_cast<int>(box.width * c));
  const int h = std::max(4, static_cast<int>(box.height * c));
  const cv::Rect inner = cv::Rect(box.x + (box.width - w) / 2, box.y + (box.height - h) / 2, w, h) &
    cv::Rect(0, 0, bgr.cols, bgr.rows);
  if (inner.width < 4 || inner.height < 4) {
    return f;
  }
  const cv::Mat roi = bgr(inner);
  cv::Mat hsv;
  cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
  const int channels[] = {0, 1};
  const int sizes[] = {16, 8};
  const float h_range[] = {0.0f, 180.0f}, s_range[] = {0.0f, 256.0f};
  const float * ranges[] = {h_range, s_range};
  cv::calcHist(&hsv, 1, channels, cv::Mat(), f.hist, 2, sizes, ranges);
  cv::normalize(f.hist, f.hist, 1.0, 0.0, cv::NORM_L1);

  cv::Mat gray, gx, gy, mag;
  cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
  cv::Sobel(gray, gx, CV_32F, 1, 0);
  cv::Sobel(gray, gy, CV_32F, 0, 1);
  cv::magnitude(gx, gy, mag);
  f.edges = cv::mean(mag)[0];
  return f;
}

void AppearanceModel::reset(const cv::Mat & bgr, const cv::Rect & box)
{
  const auto f = features(bgr, box);
  hist_ = f.hist;
  edges_ = f.edges;
}

double AppearanceModel::similarity(const cv::Mat & bgr, const cv::Rect & box) const
{
  const auto f = features(bgr, box);
  if (hist_.empty() || f.hist.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double colour = 1.0 - cv::compareHist(hist_, f.hist, cv::HISTCMP_BHATTACHARYYA);
  // Structure: ratio of edge energies, symmetric, 1 = same amount of edges.
  const double a = std::max(edges_, 1e-3), b = std::max(f.edges, 1e-3);
  const double structure = std::min(a, b) / std::max(a, b);
  return std::clamp(std::min(colour, structure), 0.0, 1.0);
}

void AppearanceModel::adapt(const cv::Mat & bgr, const cv::Rect & box)
{
  const auto f = features(bgr, box);
  if (hist_.empty() || f.hist.empty()) {
    return;
  }
  const double k = std::clamp(params_.update_rate, 0.0, 1.0);
  cv::addWeighted(hist_, 1.0 - k, f.hist, k, 0.0, hist_);
  edges_ += k * (f.edges - edges_);
}

}  // namespace aero_pursuit::perception
