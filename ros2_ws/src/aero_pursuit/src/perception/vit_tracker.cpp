#include "aero_pursuit/perception/vit_tracker.hpp"

#include <opencv2/core/ocl.hpp>
#include <opencv2/dnn.hpp>

#include <stdexcept>

namespace aero_pursuit::perception
{

DnnTarget parseDnnTarget(const std::string & name)
{
  if (name == "cpu") {
    return DnnTarget::kCpu;
  }
  if (name == "opencl") {
    return DnnTarget::kOpenCL;
  }
  if (name == "opencl_fp16") {
    return DnnTarget::kOpenCLFp16;
  }
  throw std::invalid_argument("invalid dnn_target: '" + name + "' (cpu|opencl|opencl_fp16)");
}

TrackerModel parseTrackerModel(const std::string & name)
{
  if (name == "vit") {
    return TrackerModel::kVit;
  }
  if (name == "nano") {
    return TrackerModel::kNano;
  }
  if (name == "dasiamrpn") {
    return TrackerModel::kDaSiamRpn;
  }
  throw std::invalid_argument("invalid tracker_model: '" + name + "' (vit|nano|dasiamrpn)");
}

const char * toString(TrackerModel model)
{
  switch (model) {
    case TrackerModel::kNano: return "nano";
    case TrackerModel::kDaSiamRpn: return "dasiamrpn";
    case TrackerModel::kVit: break;
  }
  return "vit";
}

namespace
{
int toCvTarget(DnnTarget t)
{
  switch (t) {
    case DnnTarget::kOpenCL: return cv::dnn::DNN_TARGET_OPENCL;
    case DnnTarget::kOpenCLFp16: return cv::dnn::DNN_TARGET_OPENCL_FP16;
    case DnnTarget::kCpu: break;
  }
  return cv::dnn::DNN_TARGET_CPU;
}
}  // namespace

VitTracker::VitTracker(const Params & params)
: params_(params)
{
  DnnTarget target = params_.target;
  if (target != DnnTarget::kCpu) {
    cv::ocl::setUseOpenCL(true);
    if (!cv::ocl::haveOpenCL()) {
      target = DnnTarget::kCpu;
    }
  }

  if (target != DnnTarget::kCpu) {
    try {
      tracker_ = create(target);
      warmUp();  // Compiles the OpenCL kernels now, not on the first click.
      const cv::ocl::Device & dev = cv::ocl::Device::getDefault();
      backend_ = std::string(target == DnnTarget::kOpenCLFp16 ? "OpenCL FP16 (" : "OpenCL (") +
        dev.name() + ")";
      return;
    } catch (const cv::Exception &) {
      tracker_.release();
    }
  }

  cv::ocl::setUseOpenCL(false);
  tracker_ = create(DnnTarget::kCpu);
  warmUp();
  backend_ = "CPU";
}

cv::Ptr<cv::Tracker> VitTracker::create(DnnTarget target)
{
  const int backend = cv::dnn::DNN_BACKEND_OPENCV;
  switch (params_.model) {
    case TrackerModel::kNano: {
        cv::TrackerNano::Params p;
        p.backbone = params_.model_path;
        p.neckhead = params_.model_path_2;
        p.backend = backend;
        p.target = toCvTarget(target);
        auto t = cv::TrackerNano::create(p);
        score_ = [t]() {return t->getTrackingScore();};
        return t;
      }
    case TrackerModel::kDaSiamRpn: {
        cv::TrackerDaSiamRPN::Params p;
        p.model = params_.model_path;
        p.kernel_r1 = params_.model_path_2;
        p.kernel_cls1 = params_.model_path_3;
        p.backend = backend;
        p.target = toCvTarget(target);
        auto t = cv::TrackerDaSiamRPN::create(p);
        score_ = [t]() {return t->getTrackingScore();};
        return t;
      }
    case TrackerModel::kVit:
      break;
  }
  cv::TrackerVit::Params p;
  p.net = params_.model_path;
  p.backend = backend;
  p.target = toCvTarget(target);
  auto t = cv::TrackerVit::create(p);
  score_ = [t]() {return t->getTrackingScore();};
  return t;
}

void VitTracker::warmUp()
{
  cv::Mat dummy(256, 256, CV_8UC3, cv::Scalar::all(127));
  cv::Rect box(96, 96, 64, 64);
  tracker_->init(dummy, box);
  tracker_->update(dummy, box);
  active_ = false;
}

void VitTracker::start(const cv::Mat & bgr, const cv::Rect & box)
{
  // A box outside the frame or without area makes TrackerVit crop an empty image, and its
  // cv::resize then aborts the whole process (the component container with every node).
  const cv::Rect clipped = box & cv::Rect(0, 0, bgr.cols, bgr.rows);
  if (bgr.empty() || clipped.width < 2 || clipped.height < 2) {
    active_ = false;
    return;
  }
  try {
    tracker_->init(bgr, clipped);
  } catch (const cv::Exception &) {
    active_ = false;
    return;
  }
  box_ = clipped;
  consecutive_ = 0;
  active_ = true;
}

VitTracker::Result VitTracker::update(const cv::Mat & bgr)
{
  Result result;
  if (!active_) {
    return result;
  }

  const cv::Rect inside = box_ & cv::Rect(0, 0, bgr.cols, bgr.rows);
  bool ok = false;
  if (!bgr.empty() && inside.width >= 2 && inside.height >= 2) {
    try {
      ok = tracker_->update(bgr, box_);
    } catch (const cv::Exception &) {
      ok = false;  // same as an empty crop: lost, and the acquisition starts over
      active_ = false;
    }
  } else {
    active_ = false;  // the box left the frame: nothing to track from (see start)
  }
  result.score = ok ? score_() : 0.0;
  result.box = box_;

  if (!ok || result.score < params_.score_threshold) {
    // Stays active: the tracker keeps trying to re-identify the target.
    consecutive_ = 0;
    result.status = Status::kLost;
  } else if (++consecutive_ < params_.min_consecutive_frames) {
    result.status = Status::kConfirming;
  } else {
    result.status = Status::kConfirmed;
  }
  return result;
}

}  // namespace aero_pursuit::perception
