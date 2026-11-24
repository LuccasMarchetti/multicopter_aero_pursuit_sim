#pragma once

#include <opencv2/core.hpp>

namespace aero_pursuit::perception
{

/// Class-agnostic appearance of the tracked ROI, compared with the template taken when the
/// track started. Two cues of the ROI centre (where the object is, not the margin):
///  - colour: hue-saturation histogram (Bhattacharyya similarity);
///  - structure: mean gradient magnitude, relative to the template's.
/// A drift to sky or ground keeps the tracker score but loses either the colours or the
/// edges of the object. similarity() = min of both cues, in [0, 1].
class AppearanceModel
{
public:
  struct Params
  {
    double center_fraction = 0.6;  ///< Side of the central crop used (fraction of the ROI)
    double update_rate = 0.02;     ///< Template adaptation per accepted frame
  };

  AppearanceModel() : AppearanceModel(Params{}) {}
  explicit AppearanceModel(const Params & p) : params_(p) {}

  void reset(const cv::Mat & bgr, const cv::Rect & box);
  bool initialized() const {return !hist_.empty();}

  /// Similarity of the ROI with the template (NaN if not initialized or ROI empty).
  double similarity(const cv::Mat & bgr, const cv::Rect & box) const;
  /// Slow template adaptation (call only for frames judged consistent).
  void adapt(const cv::Mat & bgr, const cv::Rect & box);

private:
  struct Features
  {
    cv::Mat hist;
    double edges = 0.0;
  };
  Features features(const cv::Mat & bgr, const cv::Rect & box) const;

  Params params_;
  cv::Mat hist_;
  double edges_ = 0.0;
};

}  // namespace aero_pursuit::perception
