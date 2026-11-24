#include "aero_pursuit/nodes/target_acquisition_node.hpp"

#include <fnmatch.h>

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>

namespace aero_pursuit
{
namespace
{
constexpr double kDetectionTimeout = 1.0;  ///< No tracker frames => considered inactive
constexpr double kMinIdentityIou = 0.1;
}  // namespace

TargetAcquisitionNode::TargetAcquisitionNode(const rclcpp::NodeOptions & options)
: Node("target_acquisition", options),
  last_detection_(0, 0, get_clock()->get_clock_type()),
  last_acquisition_(0, 0, get_clock()->get_clock_type())
{
  world_frame_ = declare_parameter<std::string>("world_frame", "lawn");
  camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_optical");
  declare_parameter<bool>("enabled", false);
  declare_parameter<bool>("follow_scenario", true);
  declare_parameter<bool>("reacquire", true);
  target_frame_ = declare_parameter<std::string>("target_frame", "X4");
  base_candidates_ = declare_parameter<std::vector<std::string>>(
    "candidate_frames", {"X3", "X4", "Tiltrotor", "X5"});
  target_radius_ = declare_parameter<double>("target_radius", 0.45);
  default_radius_ = declare_parameter<double>("default_radius", 0.4);
  reacquire_delay_ = declare_parameter<double>("reacquire_delay", 1.0);
  min_box_px_ = declare_parameter<double>("min_box_px", 12.0);
  margin_px_ = declare_parameter<double>("margin_px", 8.0);
  max_range_ = declare_parameter<double>("max_range", 60.0);
  box_scale_ = declare_parameter<double>("box_scale", 1.15);
  height_ratio_ = declare_parameter<double>("height_ratio", 0.5);
  use_image_stamp_ = declare_parameter<bool>("use_image_stamp", true);
  // "pattern: cx cy cz sx sy sz; ..." — box center and edges (with propellers) in the model
  // frame. Exact projection of the 8 corners with the target attitude; without a box, falls back to the sphere.
  const auto spec = declare_parameter<std::string>("model_boxes",
    "x500*: 0 0 -0.03 0.45 0.45 0.39; X3*: 0 0 0.04 0.47 0.47 0.12; X4*: 0 0 0.09 0.58 0.62 0.20; "
    "Tiltrotor*: 0 0 -0.03 0.95 2.15 0.30; X5*: 0 0 -0.03 0.45 0.45 0.39");
  std::stringstream ss(spec);
  std::string item;
  while (std::getline(ss, item, ';')) {
    const auto colon = item.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    std::string pattern = item.substr(0, colon);
    pattern.erase(0, pattern.find_first_not_of(" \t"));
    pattern.erase(pattern.find_last_not_of(" \t") + 1);
    std::stringstream vs(item.substr(colon + 1));
    perception::ModelBox b;
    if (vs >> b.center.x >> b.center.y >> b.center.z >> b.size.x >> b.size.y >> b.size.z) {
      boxes_.emplace_back(pattern, b);
    } else {
      RCLCPP_WARN(get_logger(), "model_boxes: invalid entry '%s'", item.c_str());
    }
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  sub_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    declare_parameter<std::string>("camera_info_topic", "/camera/camera_info"), rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::CameraInfo & m) {
      intrinsics_ = perception::PinholeIntrinsics::fromK(m.k);
      image_size_ = cv::Size(static_cast<int>(m.width), static_cast<int>(m.height));
      last_image_stamp_ = rclcpp::Time(m.header.stamp, RCL_ROS_TIME);
    });
  sub_detection_ = create_subscription<msg::TargetDetection>(
    declare_parameter<std::string>("detection_topic", "/target_detection"), rclcpp::SensorDataQoS(),
    [this](const msg::TargetDetection & d) {onDetection(d);});
  sub_scenario_ = create_subscription<msg::ScenarioStatus>(
    "/scenario/status", rclcpp::QoS(1).reliable().transient_local(),
    [this](const msg::ScenarioStatus & s) {onScenario(s);});
  pub_selection_ = create_publisher<sensor_msgs::msg::RegionOfInterest>(
    declare_parameter<std::string>("selection_topic", "/target_selection"), 10);
  pub_truth_ = create_publisher<msg::TrackingTruth>(
    declare_parameter<std::string>("truth_topic", "/acquisition/truth"), 10);

  timer_ = create_timer(std::chrono::milliseconds(100), [this] {tick();});
}

bool TargetAcquisitionNode::active() const
{
  if (get_parameter("enabled").as_bool()) {
    return true;
  }
  return get_parameter("follow_scenario").as_bool() && scenario_ &&
         scenario_->state == msg::ScenarioStatus::RUNNING && scenario_->acquisition == "auto" &&
         scenario_->detection == "vision";
}

void TargetAcquisitionNode::resetCounters()
{
  acquisitions_ = 0;
  switches_ = 0;
  last_identity_.clear();
}

void TargetAcquisitionNode::onScenario(const msg::ScenarioStatus & s)
{
  if (s.run_id != run_id_) {
    run_id_ = s.run_id;
    resetCounters();
  }
  radius_.clear();
  for (const auto & d : s.drones) {
    radius_[d.name] = d.radius;
  }
  scenario_ = s;
}

const perception::ModelBox * TargetAcquisitionNode::boxFor(const std::string & frame) const
{
  for (const auto & [pattern, box] : boxes_) {
    if (fnmatch(pattern.c_str(), frame.c_str(), 0) == 0) {
      return &box;
    }
  }
  return nullptr;
}

std::vector<TargetAcquisitionNode::Candidate> TargetAcquisitionNode::project(const rclcpp::Time & stamp)
{
  std::vector<Candidate> out;
  if (!intrinsics_.valid()) {
    return out;
  }
  // Designated target + fixed vehicles + every scenario drone (except pursuers).
  std::set<std::string> frames(base_candidates_.begin(), base_candidates_.end());
  frames.insert(target_frame_);
  if (scenario_) {
    for (const auto & d : scenario_->drones) {
      if (d.role != "pursuer") {
        frames.insert(d.name);
      }
    }
  }
  const bool stamped = use_image_stamp_ && stamp.nanoseconds() > 0;
  for (const auto & frame : frames) {
    // Camera ← model directly, at the frame instant: TF2 interpolates both poses (pursuer
    // and target) at the same instant. Without enough history, uses the latest ones.
    Eigen::Isometry3d T_cam_obj;
    try {
      geometry_msgs::msg::TransformStamped tf;
      try {
        tf = stamped ? tf_buffer_->lookupTransform(camera_frame_, frame, stamp) :
          tf_buffer_->lookupTransform(camera_frame_, frame, tf2::TimePointZero);
      } catch (const tf2::ExtrapolationException &) {
        tf = tf_buffer_->lookupTransform(camera_frame_, frame, tf2::TimePointZero);
      }
      T_cam_obj = tf2::transformToEigen(tf);
    } catch (const tf2::TransformException &) {
      continue;
    }
    const Eigen::Vector3d p = T_cam_obj.translation();
    if (p.norm() > max_range_) {
      continue;
    }
    std::optional<perception::ProjectedBox> box;
    if (const auto * mb = boxFor(frame)) {
      const Eigen::Matrix3d R = T_cam_obj.rotation();
      cv::Matx33d Rc;
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          Rc(i, j) = R(i, j);
        }
      }
      box = perception::projectOrientedBox(intrinsics_, Rc, cv::Vec3d(p.x(), p.y(), p.z()), *mb,
          image_size_, margin_px_, 0.3);
    } else {
      const auto r = radius_.find(frame);
      const double radius = r != radius_.end() ? r->second :
        (frame == target_frame_ ? target_radius_ : default_radius_);
      box = perception::projectSphere(intrinsics_, {p.x(), p.y(), p.z()}, radius, image_size_,
          margin_px_, 0.3, height_ratio_);
    }
    if (box) {
      out.push_back({frame, *box, p.norm()});
    }
  }
  return out;
}

std::optional<TargetAcquisitionNode::Candidate> TargetAcquisitionNode::find(
  const std::vector<Candidate> & c, const std::string & frame) const
{
  for (const auto & x : c) {
    if (x.frame == frame) {
      return x;
    }
  }
  return std::nullopt;
}

void TargetAcquisitionNode::tick()
{
  if (scenario_ && !scenario_->pursuer_target.empty()) {
    target_frame_ = scenario_->pursuer_target;
  } else {
    target_frame_ = get_parameter("target_frame").as_string();
  }
  if (!active()) {
    return;
  }
  const rclcpp::Time now = get_clock()->now();
  const bool stale = (now - last_detection_).seconds() > kDetectionTimeout;
  const bool idle = stale || tracker_status_ == msg::TargetDetection::INACTIVE ||
    tracker_status_ == msg::TargetDetection::LOST;
  // Turned on by hand: node parameters; following the scenario: whatever it asks.
  const bool manual = get_parameter("enabled").as_bool();
  const bool reacquire = !manual && scenario_ ? scenario_->reacquire : get_parameter("reacquire").as_bool();
  if (!idle || (acquisitions_ > 0 && !reacquire) ||
    (now - last_acquisition_).seconds() < reacquire_delay_)
  {
    return;
  }
  const auto target = find(project(last_image_stamp_), target_frame_);
  if (!target || !target->projection.center_inside || target->projection.box.width < min_box_px_)
  {
    return;
  }
  // Margin around the projected box: TrackerVit needs some context.
  const cv::Rect2d & b = target->projection.box;
  const cv::Point2d c(b.x + b.width / 2.0, b.y + b.height / 2.0);
  const cv::Rect2d grown = cv::Rect2d(c.x - b.width * box_scale_ / 2.0, c.y - b.height * box_scale_ / 2.0,
    b.width * box_scale_, b.height * box_scale_) & cv::Rect2d(0, 0, image_size_.width, image_size_.height);
  sensor_msgs::msg::RegionOfInterest roi;
  roi.x_offset = static_cast<uint32_t>(std::max(0.0, grown.x));
  roi.y_offset = static_cast<uint32_t>(std::max(0.0, grown.y));
  roi.width = static_cast<uint32_t>(grown.width);
  roi.height = static_cast<uint32_t>(grown.height);
  pub_selection_->publish(roi);
  last_acquisition_ = now;
  ++acquisitions_;
  RCLCPP_INFO(get_logger(), "Automatic acquisition #%u of %s: ROI %ux%u at %.1f m", acquisitions_,
    target_frame_.c_str(), roi.width, roi.height, target->distance);
}

void TargetAcquisitionNode::onDetection(const msg::TargetDetection & d)
{
  tracker_status_ = d.status;
  last_detection_ = get_clock()->now();
  if (d.image_width > 0) {
    image_size_ = cv::Size(static_cast<int>(d.image_width), static_cast<int>(d.image_height));
  }
  const auto candidates = project(rclcpp::Time(d.header.stamp, RCL_ROS_TIME));
  msg::TrackingTruth t;
  t.header = d.header;
  t.target_frame = target_frame_;
  t.tracker_status = d.status;
  t.iou = 0.0f;
  t.center_error_px = NAN;
  t.target_distance = NAN;
  t.visible_count = static_cast<uint32_t>(candidates.size());
  if (const auto target = find(candidates, target_frame_)) {
    t.target_visible = target->projection.center_inside;
    t.gt_x = static_cast<int32_t>(target->projection.box.x);
    t.gt_y = static_cast<int32_t>(target->projection.box.y);
    t.gt_width = static_cast<int32_t>(target->projection.box.width);
    t.gt_height = static_cast<int32_t>(target->projection.box.height);
    t.target_distance = static_cast<float>(target->distance);
  }
  const bool tracking = d.status >= msg::TargetDetection::CONFIRMING && d.width > 0 && d.height > 0;
  if (tracking) {
    const cv::Rect2d box(d.x, d.y, d.width, d.height);
    double best = kMinIdentityIou;
    for (const auto & c : candidates) {
      const double iou = perception::intersectionOverUnion(box, c.projection.box);
      if (c.frame == target_frame_) {
        t.iou = static_cast<float>(iou);
        const cv::Point2d gc(c.projection.box.x + c.projection.box.width / 2.0,
          c.projection.box.y + c.projection.box.height / 2.0);
        t.center_error_px = static_cast<float>(std::hypot(
          d.x + d.width / 2.0 - gc.x, d.y + d.height / 2.0 - gc.y));
      }
      if (iou > best) {
        best = iou;
        t.tracked_identity = c.frame;
      }
    }
    if (!t.tracked_identity.empty()) {
      if (!last_identity_.empty() && t.tracked_identity != last_identity_) {
        ++switches_;
        RCLCPP_INFO(get_logger(), "Identity switch: %s -> %s", last_identity_.c_str(),
          t.tracked_identity.c_str());
      }
      last_identity_ = t.tracked_identity;
    }
  }
  t.on_target = !t.tracked_identity.empty() && t.tracked_identity == target_frame_;
  t.acquisitions = acquisitions_;
  t.identity_switches = switches_;
  pub_truth_->publish(t);
}

}  // namespace aero_pursuit

RCLCPP_COMPONENTS_REGISTER_NODE(aero_pursuit::TargetAcquisitionNode)
