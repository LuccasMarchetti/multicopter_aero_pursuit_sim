#include "aero_pursuit/perception/vit_tracker.hpp"

#include <gtest/gtest.h>

#include <filesystem>

using namespace aero_pursuit::perception;

// Ensures the official model loads and runs on the local OpenCV DNN (no ONNX Runtime).
TEST(VitTracker, OfficialModelRunsOnCpu)
{
  const std::string model = std::string(AERO_PURSUIT_MODEL_DIR) +
    "/object_tracking_vittrack_2023sep.onnx";
  if (!std::filesystem::exists(model)) {
    GTEST_SKIP() << "Modelo ausente (rode models/download_models.sh): " << model;
  }

  VitTracker::Params params;
  params.model_path = model;
  params.target = DnnTarget::kCpu;
  params.min_consecutive_frames = 1;
  VitTracker tracker(params);
  EXPECT_EQ(tracker.backendDescription(), "vit / CPU");

  cv::Mat img(480, 640, CV_8UC3, cv::Scalar::all(127));
  img(cv::Rect(300, 200, 60, 60)).setTo(cv::Scalar(0, 0, 255));
  tracker.start(img, cv::Rect(300, 200, 60, 60));
  const auto r = tracker.update(img);
  EXPECT_EQ(r.status, VitTracker::Status::kConfirmed);
  EXPECT_LT(std::abs(r.box.x - 300), 10);
}

// The other class-agnostic models follow a red square too (skipped without the files).
TEST(VitTracker, AlternativeModelsTrackOnCpu)
{
  const std::string dir = AERO_PURSUIT_MODEL_DIR;
  struct Case {TrackerModel model; std::string a, b, c;};
  const Case cases[] = {
    {TrackerModel::kNano, "nanotrackv2_nanotrack_backbone_sim.onnx", "nanotrackv2_nanotrack_head_sim.onnx", ""},
    {TrackerModel::kDaSiamRpn, "dasiamrpn_model.onnx", "dasiamrpn_kernel_r1.onnx", "dasiamrpn_kernel_cls1.onnx"},
  };
  int ran = 0;
  for (const auto & c : cases) {
    if (!std::filesystem::exists(dir + "/" + c.a)) {
      continue;
    }
    VitTracker::Params params;
    params.model = c.model;
    params.model_path = dir + "/" + c.a;
    params.model_path_2 = dir + "/" + c.b;
    params.model_path_3 = c.c.empty() ? "" : dir + "/" + c.c;
    params.target = DnnTarget::kCpu;
    params.min_consecutive_frames = 1;
    params.score_threshold = 0.1f;
    VitTracker tracker(params);
    cv::Mat img(480, 640, CV_8UC3, cv::Scalar::all(127));
    img(cv::Rect(300, 200, 60, 60)).setTo(cv::Scalar(0, 0, 255));
    tracker.start(img, cv::Rect(300, 200, 60, 60));
    cv::Mat moved(480, 640, CV_8UC3, cv::Scalar::all(127));
    moved(cv::Rect(310, 205, 60, 60)).setTo(cv::Scalar(0, 0, 255));
    const auto r = tracker.update(moved);
    EXPECT_EQ(r.status, VitTracker::Status::kConfirmed) << toString(c.model);
    EXPECT_LT(std::abs(r.box.x - 310), 12) << toString(c.model);
    ++ran;
  }
  if (!ran) {
    GTEST_SKIP() << "Modelos alternativos ausentes (models/download_models.sh [--dasiamrpn])";
  }
}
