#pragma once

#include "aero_pursuit/msg/scenario_status.hpp"
#include "aero_pursuit/msg/target_detection.hpp"
#include "aero_pursuit/msg/tracking_truth.hpp"
#include "aero_pursuit/perception/box_projection.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace aero_pursuit
{

/// Automatic target acquisition and tracking ground truth (simulation).
///
///  - Acquisition: with the tracker inactive/lost and the target visible, projects the target
///    (ground truth from the TFs) into the camera and publishes the ROI on /target_selection: the
///    pursuit becomes 100% automatic, without clicking on the video. Enabled by parameter
///    (`enabled`) or by following /scenario/status (acquisition: auto).
///  - Ground truth: every tracker frame, compares its box with the projected boxes
///    of all visible vehicles (IoU, center error, tracked
///    identity, identity switches) on /acquisition/truth (TrackingTruth).
class TargetAcquisitionNode : public rclcpp::Node
{
public:
  explicit TargetAcquisitionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct Candidate
  {
    std::string frame;
    perception::ProjectedBox projection;
    double distance = 0.0;
  };

  void onDetection(const msg::TargetDetection & d);
  void onScenario(const msg::ScenarioStatus & s);
  void tick();
  /// Projects the candidates with the TFs at instant `stamp` (0: the latest ones).
  std::vector<Candidate> project(const rclcpp::Time & stamp);
  const perception::ModelBox * boxFor(const std::string & frame) const;
  std::optional<Candidate> find(const std::vector<Candidate> & c, const std::string & frame) const;
  bool active() const;
  void resetCounters();

  // Configuration
  std::string world_frame_;
  std::string camera_frame_;
  std::string target_frame_;
  std::vector<std::string> base_candidates_;
  double target_radius_ = 0.45;
  double default_radius_ = 0.4;
  double reacquire_delay_ = 1.0;
  double min_box_px_ = 12.0;
  double margin_px_ = 8.0;
  double max_range_ = 60.0;
  double box_scale_ = 1.15;
  double height_ratio_ = 0.5;   ///< Height / width of the projected box (sphere fallback)
  bool use_image_stamp_ = true; ///< TF at the frame instant (camera_info), not the latest one
  /// Oriented boxes per name pattern (fnmatch), in the order of the model_boxes parameter.
  std::vector<std::pair<std::string, perception::ModelBox>> boxes_;
  rclcpp::Time last_image_stamp_{0, 0, RCL_ROS_TIME};

  // Estado
  perception::PinholeIntrinsics intrinsics_;
  cv::Size image_size_;
  std::optional<msg::ScenarioStatus> scenario_;
  std::map<std::string, double> radius_;      ///< Radius per frame (from the scenario)
  uint8_t tracker_status_ = msg::TargetDetection::INACTIVE;
  rclcpp::Time last_detection_;
  rclcpp::Time last_acquisition_;
  std::string last_identity_;
  std::string run_id_;
  uint32_t acquisitions_ = 0;
  uint32_t switches_ = 0;

  // Interfaces ROS
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  rclcpp::Subscription<msg::TargetDetection>::SharedPtr sub_detection_;
  rclcpp::Subscription<msg::ScenarioStatus>::SharedPtr sub_scenario_;
  rclcpp::Publisher<sensor_msgs::msg::RegionOfInterest>::SharedPtr pub_selection_;
  rclcpp::Publisher<msg::TrackingTruth>::SharedPtr pub_truth_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace aero_pursuit
