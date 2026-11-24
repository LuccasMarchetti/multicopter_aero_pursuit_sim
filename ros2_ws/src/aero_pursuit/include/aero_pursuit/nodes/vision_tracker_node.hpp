#pragma once

#include "aero_pursuit/msg/target_bearing.hpp"
#include "aero_pursuit/msg/target_detection.hpp"
#include "aero_pursuit/perception/camera_model.hpp"
#include "aero_pursuit/perception/roi_selector.hpp"
#include "aero_pursuit/perception/appearance_model.hpp"
#include "aero_pursuit/perception/track_verifier.hpp"
#include "aero_pursuit/perception/vit_tracker.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>

#include <cmath>
#include <deque>
#include <mutex>
#include <array>
#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace aero_pursuit
{

/// Pure perception: tracks the target in the RGB image (TrackerVit), estimates the
/// depth and publishes the 3D position in the camera optical frame.
/// Without valid depth (target beyond the sensor range), publishes only the
/// line of sight (`TargetBearing`) for the direction-guided approach.
/// Transforms to the body/world are left to the estimator (tf2).
///
/// Each RGB frame is processed as soon as it arrives (no synchronizer): the last
/// depth is cached and used if it is within `sync_slop_sec`.
class VisionTrackerNode : public rclcpp::Node
{
public:
  explicit VisionTrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using PointStamped = geometry_msgs::msg::PointStamped;

  struct Localization
  {
    perception::Bearing bearing;
    std::optional<cv::Point3d> point;  ///< Empty: no valid depth
    float depth = NAN;
    bool beyond_range = false;   ///< Whole ROI beyond the far clip: target farther than depth_max_range_
    float valid_fraction = 0.0f; ///< ROI pixels with depth within range
  };

  void onRgb(const std::shared_ptr<const Image> & rgb);
  void onClick(const PointStamped & msg);
  void onHover(const PointStamped & msg);
  /// Direct selection (GCS): ROI in pixels of the original image; width/height 0 => stop.
  void onSelection(const sensor_msgs::msg::RegionOfInterest & roi);
  void startTracking(const cv::Rect & box);
  rcl_interfaces::msg::SetParametersResult onParameters(const std::vector<rclcpp::Parameter> & params);

  /// Empty path: the file in the package share/models (default: ViTTrack).
  std::string resolveModelPath(const std::string & path, const char * file = nullptr) const;
  void setModelPaths(perception::VitTracker::Params & p) const;
  /// Runs the TrackVerifier on a confirmed frame; inconsistent => no output (and a stop on failure).
  void verify(const std_msgs::msg::Header & header, perception::VitTracker::Result & result,
    std::optional<Localization> & loc);
  void loadVerifierParams();
  bool processingDue();
  std::shared_ptr<const Image> matchingDepth(const Image & rgb) const;
  std::optional<Localization> locateTarget(
    const cv::Rect & box, const Image * depth_msg, const cv::Size & rgb_size);
  void publishOutputs(
    const std_msgs::msg::Header & header, const perception::VitTracker::Result & result,
    const std::optional<Localization> & loc, float processing_ms);
  bool debugRequested();
  void publishDebug(
    const std_msgs::msg::Header & header, const perception::VitTracker::Result & result,
    const std::optional<Localization> & loc);
  double debugScale() const {return debug_resize_ ? debug_scale_ : 1.0;}
  cv::Point toImagePixel(const geometry_msgs::msg::Point & p) const;

  // Parameters (all tunable at runtime, except topics and the DNN backend)
  std::string camera_frame_;
  double depth_percentile_lower_;
  double depth_percentile_upper_;
  double depth_tolerance_;
  bool depth_fallback_ = true;
  double depth_min_range_ = 0.2;    ///< Depth sensor near clip [m]
  double depth_max_range_ = 19.1;   ///< Depth sensor far clip [m]
  double depth_beyond_fraction_ = 0.85;  ///< Fraction of the ROI beyond far to declare "farther than"

  double processing_period_ = 0.0;  ///< 0: processes every frame
  bool debug_resize_ = true;
  double debug_scale_;
  double debug_period_ = 0.0;       ///< 0: publishes every frame
  int jpeg_quality_;

  // Estado
  std::unique_ptr<perception::VitTracker> tracker_;
  perception::RoiSelector roi_;
  perception::PinholeIntrinsics rgb_intrinsics_;
  perception::PinholeIntrinsics depth_intrinsics_;
  /// Latest depth frames: RGB and depth come from different sensors,
  /// unsynchronized; the pair is the one with the closest stamp (not the last received).
  std::deque<std::shared_ptr<const Image>> depth_history_;
  std::size_t depth_history_size_ = 10;
  /// Depth has its own callback group: it keeps being stored while the
  /// RGB is processed (slow inference, e.g. OpenCL), otherwise no frame matches the stamp.
  mutable std::mutex depth_mutex_;
  perception::VitTracker::Params tracker_params_;
  std::optional<perception::DnnTarget> pending_target_;  ///< CPU/GPU switch requested at runtime
  std::optional<perception::TrackerModel> pending_model_;  ///< Model switch requested at runtime
  perception::TrackVerifier verifier_;
  perception::AppearanceModel appearance_;
  bool verifier_enabled_ = false;
  std::atomic<bool> verifier_reload_{false};  ///< Parameters changed (applied on the tracker thread)
  int verifier_failures_ = 0;
  std::string last_verifier_;          ///< Verdict of the current frame (TargetDetection)
  double last_appearance_ = NAN;
  /// vit, nano backbone/head, dasiamrpn model/kernel_r1/kernel_cls1
  std::array<std::string, 6> model_paths_;
  rclcpp::CallbackGroup::SharedPtr depth_group_;
  cv::Mat frame_bgr_;   ///< Last frame (reused buffer)
  cv::Mat depth_buf_;   ///< Buffer for converted 16UC1 depth
  cv::Mat debug_buf_;
  std::vector<float> depth_scratch_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_debug_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_process_time_{0, 0, RCL_STEADY_TIME};

  // Interfaces ROS
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_rgb_info_;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_depth_info_;
  rclcpp::Subscription<Image>::SharedPtr sub_rgb_;
  /// image_compressed: JPEG input (remote pursuer), decoded into BGR8 Image messages.
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_rgb_compressed_, sub_depth_compressed_;
  rclcpp::Subscription<Image>::SharedPtr sub_depth_;
  rclcpp::Subscription<PointStamped>::SharedPtr sub_click_;
  rclcpp::Subscription<PointStamped>::SharedPtr sub_hover_;
  rclcpp::Subscription<sensor_msgs::msg::RegionOfInterest>::SharedPtr sub_selection_;
  rclcpp::Publisher<PointStamped>::SharedPtr pub_target_;
  rclcpp::Publisher<msg::TargetBearing>::SharedPtr pub_bearing_;
  rclcpp::Publisher<msg::TargetDetection>::SharedPtr pub_detection_;
  rclcpp::Publisher<Image>::SharedPtr pub_debug_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_debug_compressed_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_handle_;
};

}  // namespace aero_pursuit
