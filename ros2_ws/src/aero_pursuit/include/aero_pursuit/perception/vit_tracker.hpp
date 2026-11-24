#pragma once

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

#include <functional>
#include <string>

namespace aero_pursuit::perception
{

enum class DnnTarget { kCpu, kOpenCL, kOpenCLFp16 };

DnnTarget parseDnnTarget(const std::string & name);

/// Class-agnostic single-object trackers of OpenCV (template given by the initial ROI):
///  vit      ViTTrack (transformer, ~0.7 MB)
///  nano     NanoTrack v2 (Siamese, ~1.8 MB, fastest)
///  dasiamrpn DaSiamRPN (Siamese RPN with distractor-aware training, ~160 MB)
enum class TrackerModel { kVit, kNano, kDaSiamRpn };

TrackerModel parseTrackerModel(const std::string & name);
const char * toString(TrackerModel model);

/// Wraps an OpenCV DNN tracker (cv::TrackerVit, cv::TrackerNano or cv::TrackerDaSiamRPN) with:
///  - a single network load (reused for every new ROI);
///  - warm-up on the OpenCL backend and automatic fallback to CPU;
///  - confirmation by N consecutive frames above the score threshold.
class VitTracker
{
public:
  enum class Status { kInactive, kLost, kConfirming, kConfirmed };

  struct Params
  {
    TrackerModel model = TrackerModel::kVit;
    std::string model_path;      ///< vit: network; nano: backbone; dasiamrpn: model
    std::string model_path_2;    ///< nano: head; dasiamrpn: kernel_r1
    std::string model_path_3;    ///< dasiamrpn: kernel_cls1
    DnnTarget target = DnnTarget::kOpenCL;
    float score_threshold = 0.35f;
    int min_consecutive_frames = 5;
  };

  struct Result
  {
    Status status = Status::kInactive;
    cv::Rect box;
    float score = 0.0f;
  };

  explicit VitTracker(const Params & params);

  void setScoreThreshold(float t) {params_.score_threshold = t;}
  void setMinConsecutiveFrames(int n) {params_.min_consecutive_frames = n;}

  void start(const cv::Mat & bgr, const cv::Rect & box);
  void stop() {active_ = false;}
  bool active() const {return active_;}

  Result update(const cv::Mat & bgr);

  /// Description of the backend actually in use (for logging).
  std::string backendDescription() const {return std::string(toString(params_.model)) + " / " + backend_;}

private:
  cv::Ptr<cv::Tracker> create(DnnTarget target);
  void warmUp();

  Params params_;
  cv::Ptr<cv::Tracker> tracker_;
  std::function<float()> score_;  ///< getTrackingScore() of the concrete tracker
  std::string backend_;
  cv::Rect box_;
  bool active_ = false;
  int consecutive_ = 0;
};

}  // namespace aero_pursuit::perception
