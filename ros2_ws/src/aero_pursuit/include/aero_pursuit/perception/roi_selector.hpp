#pragma once

#include <opencv2/core.hpp>

#include <optional>

namespace aero_pursuit::perception
{

/// Two-click ROI selection (Foxglove): the first click fixes a corner,
/// hover updates the preview and the second click confirms the box.
class RoiSelector
{
public:
  bool drawing() const {return drawing_;}
  cv::Rect preview() const {return boxFromCorners(corner1_, corner2_);}

  /// Returns the box when the second click completes it.
  std::optional<cv::Rect> onClick(const cv::Point & pt)
  {
    if (!drawing_) {
      corner1_ = corner2_ = pt;
      drawing_ = true;
      return std::nullopt;
    }
    corner2_ = pt;
    drawing_ = false;
    return preview();
  }

  /// Discards a pending first click.
  void reset() {drawing_ = false;}

  void onHover(const cv::Point & pt)
  {
    if (drawing_) {
      corner2_ = pt;
    }
  }

  static cv::Rect boxFromCorners(const cv::Point & a, const cv::Point & b)
  {
    return {std::min(a.x, b.x), std::min(a.y, b.y), std::abs(a.x - b.x), std::abs(a.y - b.y)};
  }

private:
  cv::Point corner1_;
  cv::Point corner2_;
  bool drawing_ = false;
};

}  // namespace aero_pursuit::perception
