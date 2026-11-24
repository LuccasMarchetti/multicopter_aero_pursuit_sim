#include "aero_pursuit/nodes/vision_tracker_node.hpp"

#include "aero_pursuit/perception/image_conversions.hpp"
#include "aero_pursuit/perception/overlay.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>

namespace aero_pursuit
{

using perception::VitTracker;

namespace
{
constexpr const char * kDefaultModel = "object_tracking_vittrack_2023sep.onnx";

double periodFromRate(double rate_hz)
{
  return rate_hz > 0.0 ? 1.0 / rate_hz : 0.0;
}

uint8_t toMsgStatus(VitTracker::Status s)
{
  switch (s) {
    case VitTracker::Status::kLost: return msg::TargetDetection::LOST;
    case VitTracker::Status::kConfirming: return msg::TargetDetection::CONFIRMING;
    case VitTracker::Status::kConfirmed: return msg::TargetDetection::CONFIRMED;
    case VitTracker::Status::kInactive: break;
  }
  return msg::TargetDetection::INACTIVE;
}
}  // namespace

VisionTrackerNode::VisionTrackerNode(const rclcpp::NodeOptions & options)
: Node("vision_tracker_node", options)
{
  const auto image_topic = declare_parameter<std::string>("image_topic", "/camera");
  const auto camera_info_topic = declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
  const auto depth_topic = declare_parameter<std::string>("depth_topic", "/depth_camera");
  const auto depth_info_topic =
    declare_parameter<std::string>("depth_info_topic", "/depth_camera/camera_info");
  const auto target_topic = declare_parameter<std::string>("target_topic", "/target_3d_position");
  const auto bearing_topic = declare_parameter<std::string>("bearing_topic", "/target_bearing");
  const auto detection_topic = declare_parameter<std::string>("detection_topic", "/target_detection");
  const auto debug_topic = declare_parameter<std::string>("debug_image_topic", "/camera/segmented_bb");
  const auto click_topic = declare_parameter<std::string>("click_topic", "/foxglove/cursor/click");
  const auto hover_topic = declare_parameter<std::string>("hover_topic", "/foxglove/cursor/hover");
  const auto selection_topic = declare_parameter<std::string>("selection_topic", "/target_selection");
  camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_optical");

  rcl_interfaces::msg::ParameterDescriptor read_only;
  read_only.read_only = true;

  VitTracker::Params tracker_params;
  rcl_interfaces::msg::ParameterDescriptor model_desc;
  model_desc.description = "vit (ViTTrack) | nano (NanoTrack v2) | dasiamrpn (DaSiamRPN); switchable at "
    "run time (tracking restarts). Generic trackers: the target is whatever is in the initial ROI";
  tracker_params.model = perception::parseTrackerModel(
    declare_parameter<std::string>("tracker_model", "vit", model_desc));
  model_paths_[0] = resolveModelPath(declare_parameter<std::string>("vittrack_model_path", "", read_only));
  model_paths_[1] = resolveModelPath(
    declare_parameter<std::string>("nano_backbone_path", "", read_only), "nanotrackv2_nanotrack_backbone_sim.onnx");
  model_paths_[2] = resolveModelPath(
    declare_parameter<std::string>("nano_head_path", "", read_only), "nanotrackv2_nanotrack_head_sim.onnx");
  model_paths_[3] = resolveModelPath(
    declare_parameter<std::string>("dasiamrpn_model_path", "", read_only), "dasiamrpn_model.onnx");
  model_paths_[4] = resolveModelPath(
    declare_parameter<std::string>("dasiamrpn_kernel_r1_path", "", read_only), "dasiamrpn_kernel_r1.onnx");
  model_paths_[5] = resolveModelPath(
    declare_parameter<std::string>("dasiamrpn_kernel_cls1_path", "", read_only), "dasiamrpn_kernel_cls1.onnx");
  setModelPaths(tracker_params);
  rcl_interfaces::msg::ParameterDescriptor target_desc;
  target_desc.description = "cpu | opencl | opencl_fp16 (switchable at run time: recreates the TrackerVit)";
  tracker_params.target = perception::parseDnnTarget(
    declare_parameter<std::string>("dnn_target", "cpu", target_desc));
  tracker_params.score_threshold =
    static_cast<float>(declare_parameter<double>("tracking_score_threshold", 0.35));
  tracker_params.min_consecutive_frames =
    static_cast<int>(declare_parameter<int>("min_consecutive_frames", 5));

  depth_tolerance_ = declare_parameter<double>("sync_slop_sec", 0.05);
  depth_history_size_ = static_cast<std::size_t>(
    std::max<int64_t>(1, declare_parameter<int>("depth_buffer_size", 10)));
  depth_percentile_lower_ = declare_parameter<double>("depth_percentile_lower", 10.0);
  depth_percentile_upper_ = declare_parameter<double>("depth_percentile_upper", 30.0);
  depth_fallback_ = declare_parameter<bool>("depth_fallback_enabled", true);
  // Depth camera range (OakD-Lite StereoOV7251 in the PX4 SDF: clip 0.2–19.1 m).
  // Beyond far Gazebo returns no distance: a whole ROI "beyond" becomes a lower bound.
  depth_min_range_ = declare_parameter<double>("depth_min_range", 0.2);
  depth_max_range_ = declare_parameter<double>("depth_max_range", 19.1);
  depth_beyond_fraction_ = declare_parameter<double>("depth_beyond_fraction", 0.85);
  processing_period_ = periodFromRate(declare_parameter<double>("max_processing_rate_hz", 0.0));
  debug_resize_ = declare_parameter<bool>("debug_image_resize", true);
  debug_scale_ = std::clamp(declare_parameter<double>("debug_image_scale", 0.5), 0.1, 1.0);
  debug_period_ = periodFromRate(declare_parameter<double>("debug_image_rate_hz", 0.0));
  jpeg_quality_ = std::clamp(static_cast<int>(declare_parameter<int>("debug_jpeg_quality", 75)), 1, 100);

  {
    const perception::TrackVerifier::Params vp;
    auto desc = [](const std::string & text) {
        rcl_interfaces::msg::ParameterDescriptor d;
        d.description = text;
        return d;
      };
    declare_parameter<bool>("verifier.enabled", true,
      desc("Physical consistency check of the tracking (drops drift to the background/another object)"));
    declare_parameter<bool>("verifier.depth_check", vp.depth_check,
      desc("Depth continuity: the target does not jump beyond range nor change more than v_max·Δt"));
    declare_parameter<bool>("verifier.scale_check", vp.scale_check,
      desc("ROI angular size consistent with the learned size and the distance"));
    declare_parameter<bool>("verifier.score_check", vp.score_check,
      desc("Score drop below score_drop_ratio × recent mean"));
    declare_parameter<bool>("verifier.appearance_check", vp.appearance_check,
      desc("ROI center appearance (color + edges) similar to the initial model"));
    declare_parameter<double>("verifier.appearance_min", vp.appearance_min,
      desc("appearance_check: minimum similarity to the model [0-1]"));
    declare_parameter<double>("verifier.range_margin", vp.range_margin,
      desc("depth_check: the last distance must be this margin inside the range [m]"));
    declare_parameter<double>("verifier.memory", vp.memory, desc("Maximum age of the last depth used [s]"));
    declare_parameter<double>("verifier.max_closing_speed", vp.max_closing_speed,
      desc("Limit of |dr/dt| (target + pursuer) [m/s]"));
    declare_parameter<double>("verifier.scale_ratio", vp.scale_ratio,
      desc("Maximum factor between the measured and the predicted angular size"));
    declare_parameter<double>("verifier.score_drop_ratio", vp.score_drop_ratio,
      desc("score_check: fraction of the score mean"));
    declare_parameter<int>("verifier.fail_frames", vp.fail_frames,
      desc("Inconsistent frames in a row to suspend (or drop) the tracking"));
    declare_parameter<bool>("verifier.box_check", vp.box_check,
      desc("ROI larger than max_box_fraction of the image (box grown over the background)"));
    declare_parameter<double>("verifier.max_box_fraction", vp.max_box_fraction,
      desc("box_check: maximum ROI width/height as a fraction of the image"));
    declare_parameter<double>("verifier.hold_time", vp.hold_time,
      desc("After a failure: wait up to this long for the tracker to return to the target, without "
      "publishing [s]; 0 drops at once"));
    declare_parameter<int>("verifier.recover_frames", vp.recover_frames,
      desc("Consistent frames in a row that end the suspension"));
    loadVerifierParams();
  }
  tracker_params_ = tracker_params;
  tracker_ = std::make_unique<VitTracker>(tracker_params);
  RCLCPP_INFO(get_logger(), "Tracker (OpenCV DNN) loaded from '%s'. Backend: %s",
    tracker_params.model_path.c_str(), tracker_->backendDescription().c_str());

  // Intrinsics: read only once, then the subscription is released.
  sub_rgb_info_ = create_subscription<CameraInfo>(
    camera_info_topic, rclcpp::SensorDataQoS(), [this](const CameraInfo & msg) {
      rgb_intrinsics_ = perception::PinholeIntrinsics::fromK(msg.k);
      if (rgb_intrinsics_.valid()) {sub_rgb_info_.reset();}
    });
  sub_depth_info_ = create_subscription<CameraInfo>(
    depth_info_topic, rclcpp::SensorDataQoS(), [this](const CameraInfo & msg) {
      depth_intrinsics_ = perception::PinholeIntrinsics::fromK(msg.k);
      if (depth_intrinsics_.valid()) {sub_depth_info_.reset();}
    });

  sub_click_ = create_subscription<PointStamped>(
    click_topic, 10, [this](const PointStamped & msg) {onClick(msg);});
  sub_hover_ = create_subscription<PointStamped>(
    hover_topic, rclcpp::SensorDataQoS(), [this](const PointStamped & msg) {onHover(msg);});
  sub_selection_ = create_subscription<sensor_msgs::msg::RegionOfInterest>(
    selection_topic, 10,
    [this](const sensor_msgs::msg::RegionOfInterest & msg) {onSelection(msg);});

  pub_target_ = create_publisher<PointStamped>(target_topic, 10);
  pub_bearing_ = create_publisher<msg::TargetBearing>(bearing_topic, 10);
  pub_detection_ = create_publisher<msg::TargetDetection>(detection_topic, 10);
  pub_debug_ = create_publisher<Image>(debug_topic, rclcpp::SensorDataQoS());
  pub_debug_compressed_ = create_publisher<sensor_msgs::msg::CompressedImage>(
    debug_topic + "/compressed", rclcpp::SensorDataQoS());

  // Images: best-effort and a short queue (drops old frames instead of accumulating
  // latency). const pointers => no copy in intra-process communication.
  // Local (intra-process): best effort, no copy and no retransmission. Remote pursuer (images over
  // the network, launch/pursuer.launch.xml): reliable, because a best-effort image of 1-6 MB is
  // split into dozens of UDP fragments and one lost fragment drops the whole frame.
  const bool reliable = declare_parameter<bool>("image_qos_reliable", false);
  auto image_qos = rclcpp::SensorDataQoS().keep_last(2);
  if (reliable) {
    image_qos.reliable();
  }
  depth_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions depth_options;
  depth_options.callback_group = depth_group_;
  auto push_depth = [this](std::shared_ptr<const Image> msg) {
      std::lock_guard<std::mutex> lock(depth_mutex_);
      if (!depth_history_.empty() &&
      rclcpp::Time(msg->header.stamp) < rclcpp::Time(depth_history_.back()->header.stamp))
      {
        depth_history_.clear();  // Simulation time went back (Gazebo reset)
      }
      depth_history_.push_back(std::move(msg));
      while (depth_history_.size() > depth_history_size_) {
        depth_history_.pop_front();
      }
    };
  // Remote pursuer: lossless PNG of 16-bit millimetres (image_downscaler depth_png_level).
  if (declare_parameter<bool>("depth_compressed", false)) {
    sub_depth_compressed_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      depth_topic + "/compressed", image_qos,
      [push_depth](const sensor_msgs::msg::CompressedImage & msg) {
        auto img = std::make_shared<Image>();
        if (perception::fromCompressedDepthMsg(msg, *img)) {
          push_depth(img);
        }
      }, depth_options);
  } else {
    sub_depth_ = create_subscription<Image>(depth_topic, image_qos, push_depth, depth_options);
  }
  // Remote pursuer: the simulator side sends JPEG (image_downscaler), ~15x less traffic than raw.
  if (declare_parameter<bool>("image_compressed", false)) {
    sub_rgb_compressed_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      image_topic + "/compressed", image_qos,
      [this](const sensor_msgs::msg::CompressedImage & msg) {
        cv::Mat bgr;
        if (!perception::fromCompressedMsg(msg, bgr)) {
          return;
        }
        auto img = std::make_shared<Image>();
        perception::toImageMsg(bgr, msg.header, *img);
        onRgb(img);
      });
  } else {
    sub_rgb_ = create_subscription<Image>(
      image_topic, image_qos, [this](std::shared_ptr<const Image> msg) {onRgb(msg);});
  }

  param_handle_ = add_on_set_parameters_callback(
    std::bind(&VisionTrackerNode::onParameters, this, std::placeholders::_1));
}

std::string VisionTrackerNode::resolveModelPath(const std::string & path, const char * file) const
{
  if (!path.empty()) {
    return path;
  }
  return ament_index_cpp::get_package_share_directory("aero_pursuit") + "/models/" +
         (file ? file : kDefaultModel);
}

rcl_interfaces::msg::SetParametersResult VisionTrackerNode::onParameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : params) {
    const auto & name = p.get_name();
    if (name == "tracker_model") {
      try {
        pending_model_ = perception::parseTrackerModel(p.as_string());
      } catch (const std::invalid_argument & e) {
        rcl_interfaces::msg::SetParametersResult r;
        r.successful = false;
        r.reason = e.what();
        return r;
      }
    } else if (name == "dnn_target") {
      try {
        pending_target_ = perception::parseDnnTarget(p.as_string());
      } catch (const std::invalid_argument & e) {
        rcl_interfaces::msg::SetParametersResult r;
        r.successful = false;
        r.reason = e.what();
        return r;
      }
    } else if (name == "min_consecutive_frames") {
      tracker_->setMinConsecutiveFrames(static_cast<int>(p.as_int()));
    } else if (name == "tracking_score_threshold") {
      tracker_->setScoreThreshold(static_cast<float>(p.as_double()));
    } else if (name == "depth_percentile_lower") {
      depth_percentile_lower_ = p.as_double();
    } else if (name == "depth_percentile_upper") {
      depth_percentile_upper_ = p.as_double();
    } else if (name == "sync_slop_sec") {
      depth_tolerance_ = p.as_double();
    } else if (name == "depth_fallback_enabled") {
      depth_fallback_ = p.as_bool();
    } else if (name == "depth_min_range") {
      depth_min_range_ = p.as_double();
    } else if (name == "depth_max_range") {
      depth_max_range_ = p.as_double();
    } else if (name == "depth_beyond_fraction") {
      depth_beyond_fraction_ = std::clamp(p.as_double(), 0.0, 1.0);
    } else if (name == "max_processing_rate_hz") {
      processing_period_ = periodFromRate(p.as_double());
    } else if (name == "debug_image_resize") {
      debug_resize_ = p.as_bool();
    } else if (name == "debug_image_scale") {
      debug_scale_ = std::clamp(p.as_double(), 0.1, 1.0);
    } else if (name == "debug_image_rate_hz") {
      debug_period_ = periodFromRate(p.as_double());
    } else if (name == "debug_jpeg_quality") {
      jpeg_quality_ = std::clamp(static_cast<int>(p.as_int()), 1, 100);
    }
    if (name.rfind("verifier.", 0) == 0 || name == "depth_max_range") {
      verifier_reload_ = true;
    }
  }
  return result;
}

cv::Point VisionTrackerNode::toImagePixel(const geometry_msgs::msg::Point & p) const
{
  // Clicks arrive in debug image coordinates (possibly downscaled).
  const double s = debugScale();
  return {cvRound(p.x / s), cvRound(p.y / s)};
}

void VisionTrackerNode::onClick(const PointStamped & msg)
{
  const auto box = roi_.onClick(toImagePixel(msg.point));
  if (!box) {
    tracker_->stop();  // A new selection interrupts the current tracking
    return;
  }
  startTracking(*box);
}

void VisionTrackerNode::onSelection(const sensor_msgs::msg::RegionOfInterest & roi)
{
  roi_.reset();
  if (roi.width == 0 || roi.height == 0) {
    tracker_->stop();
    RCLCPP_INFO(get_logger(), "Tracking stopped by the GCS");
    return;
  }
  startTracking(cv::Rect(
    static_cast<int>(roi.x_offset), static_cast<int>(roi.y_offset),
    static_cast<int>(roi.width), static_cast<int>(roi.height)));
}

void VisionTrackerNode::startTracking(const cv::Rect & requested)
{
  if (frame_bgr_.empty()) {
    return;
  }
  const cv::Rect box = requested & cv::Rect(0, 0, frame_bgr_.cols, frame_bgr_.rows);
  if (box.width < 4 || box.height < 4) {
    return;
  }
  try {
    tracker_->start(frame_bgr_, box);
    verifier_.reset();
    appearance_.reset(frame_bgr_, box);
    RCLCPP_INFO(get_logger(), "Tracking started: ROI %dx%d at (%d, %d)",
      box.width, box.height, box.x, box.y);
  } catch (const cv::Exception & e) {
    RCLCPP_ERROR(get_logger(), "Error starting TrackerVit: %s", e.what());
    tracker_->stop();
  }
}

void VisionTrackerNode::onHover(const PointStamped & msg)
{
  roi_.onHover(toImagePixel(msg.point));
}

bool VisionTrackerNode::processingDue()
{
  if (processing_period_ <= 0.0) {
    return true;
  }
  const rclcpp::Time now = steady_clock_.now();
  if ((now - last_process_time_).seconds() < processing_period_) {
    return false;
  }
  last_process_time_ = now;
  return true;
}

std::shared_ptr<const sensor_msgs::msg::Image> VisionTrackerNode::matchingDepth(const Image & rgb) const
{
  std::lock_guard<std::mutex> lock(depth_mutex_);
  const rclcpp::Time t_rgb(rgb.header.stamp);
  std::shared_ptr<const Image> best;
  double best_dt = depth_tolerance_;
  for (const auto & d : depth_history_) {
    const double dt = std::abs((t_rgb - rclcpp::Time(d->header.stamp)).seconds());
    if (dt <= best_dt) {
      best_dt = dt;
      best = d;
    }
  }
  return best;
}

std::optional<VisionTrackerNode::Localization> VisionTrackerNode::locateTarget(
  const cv::Rect & box, const Image * depth_msg, const cv::Size & rgb_size)
{
  if (!rgb_intrinsics_.valid()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for the RGB camera_info.");
    return std::nullopt;
  }

  const double u = box.x + 0.5 * box.width;
  const double v = box.y + 0.5 * box.height;
  Localization loc;
  loc.bearing = perception::bearingFromPixel(rgb_intrinsics_, u, v);

  if (depth_msg) {
    const cv::Mat depth_m = perception::toDepthMeters(*depth_msg, depth_buf_);
    if (!depth_m.empty()) {
      const cv::Rect box_depth = perception::mapBoxToDepth(
        box, rgb_size, depth_m.size(), rgb_intrinsics_, depth_intrinsics_);
      const auto depth = perception::robustDepth(
        depth_m, box_depth, depth_percentile_lower_, depth_percentile_upper_, depth_scratch_,
        static_cast<float>(depth_min_range_), static_cast<float>(depth_max_range_));
      const auto cov = perception::depthCoverage(depth_m, box_depth,
          static_cast<float>(depth_min_range_), static_cast<float>(depth_max_range_));
      loc.valid_fraction = static_cast<float>(cov.validFraction());
      // No valid pixel and (almost) everything beyond far: the target is farther than the range.
      loc.beyond_range = !depth && cov.total > 0 && cov.beyondFraction() >= depth_beyond_fraction_;
      if (depth) {
        loc.depth = *depth;
        loc.point = rgb_intrinsics_.backproject(u, v, *depth);
      }
    }
  }
  return loc;
}

void VisionTrackerNode::verify(
  const std_msgs::msg::Header & header, VitTracker::Result & result, std::optional<Localization> & loc)
{
  perception::TrackVerifier::Frame f;
  f.time = rclcpp::Time(header.stamp).seconds();
  if (loc->point) {
    f.depth = loc->depth;
  }
  f.beyond_range = loc->beyond_range;
  const double side = std::sqrt(static_cast<double>(result.box.width) * result.box.height);
  const double focal = std::sqrt(rgb_intrinsics_.fx * rgb_intrinsics_.fy);
  f.angular_size = side > 0.0 && focal > 0.0 ? 2.0 * std::atan(side / (2.0 * focal)) : NAN;
  if (frame_bgr_.cols > 0 && frame_bgr_.rows > 0) {
    f.box_fraction = std::max(static_cast<double>(result.box.width) / frame_bgr_.cols,
        static_cast<double>(result.box.height) / frame_bgr_.rows);
  }
  f.score = result.score;
  if (verifier_.params().appearance_check && appearance_.initialized()) {
    f.appearance = appearance_.similarity(frame_bgr_, result.box);
  }
  const bool was_holding = verifier_.holding();
  const auto v = verifier_.update(f);
  if (verifier_.holding() != was_holding) {
    if (verifier_.holding()) {
      RCLCPP_WARN(get_logger(), "Verifier: tracking suspended (%s; appearance %.2f), waiting for "
        "%d consistent frames for up to %.1f s", v.reason.c_str(), f.appearance,
        verifier_.params().recover_frames, verifier_.params().hold_time);
    } else {
      RCLCPP_INFO(get_logger(), "Verificador: rastreamento recuperado");
    }
  }
  last_appearance_ = f.appearance;
  last_verifier_ = v.verdict == perception::TrackVerifier::Verdict::kOk ? std::string("ok") :
    std::string(perception::toString(v.verdict)) + ":" + v.reason;
  if (v.verdict == perception::TrackVerifier::Verdict::kOk) {
    if (std::isfinite(f.appearance)) {
      appearance_.adapt(frame_bgr_, result.box);
    }
    return;
  }
  // Inconsistent frame: no measurement leaves the node.
  loc.reset();
  result.status = VitTracker::Status::kLost;
  if (v.verdict == perception::TrackVerifier::Verdict::kFailed) {
    ++verifier_failures_;
    RCLCPP_WARN(get_logger(),
      "Verifier: tracking dropped (%s; appearance %.2f; learned size %.2f m; failure %d). "
      "The tracker probably drifted to the background or another object.",
      v.reason.c_str(), f.appearance, verifier_.learnedSize(), verifier_failures_);
    tracker_->stop();
    verifier_.reset();
  }
}

void VisionTrackerNode::loadVerifierParams()
{
  verifier_enabled_ = get_parameter("verifier.enabled").as_bool();
  perception::TrackVerifier::Params p;
  p.depth_check = get_parameter("verifier.depth_check").as_bool();
  p.scale_check = get_parameter("verifier.scale_check").as_bool();
  p.score_check = get_parameter("verifier.score_check").as_bool();
  p.appearance_check = get_parameter("verifier.appearance_check").as_bool();
  p.appearance_min = get_parameter("verifier.appearance_min").as_double();
  p.max_range = depth_max_range_;
  p.range_margin = get_parameter("verifier.range_margin").as_double();
  p.memory = get_parameter("verifier.memory").as_double();
  p.max_closing_speed = get_parameter("verifier.max_closing_speed").as_double();
  p.scale_ratio = get_parameter("verifier.scale_ratio").as_double();
  p.score_drop_ratio = get_parameter("verifier.score_drop_ratio").as_double();
  p.fail_frames = static_cast<int>(get_parameter("verifier.fail_frames").as_int());
  p.box_check = get_parameter("verifier.box_check").as_bool();
  p.max_box_fraction = get_parameter("verifier.max_box_fraction").as_double();
  p.hold_time = get_parameter("verifier.hold_time").as_double();
  p.recover_frames = static_cast<int>(get_parameter("verifier.recover_frames").as_int());
  verifier_.setParams(p);
}

void VisionTrackerNode::setModelPaths(VitTracker::Params & p) const
{
  switch (p.model) {
    case perception::TrackerModel::kNano:
      p.model_path = model_paths_[1];
      p.model_path_2 = model_paths_[2];
      break;
    case perception::TrackerModel::kDaSiamRpn:
      p.model_path = model_paths_[3];
      p.model_path_2 = model_paths_[4];
      p.model_path_3 = model_paths_[5];
      break;
    case perception::TrackerModel::kVit:
      p.model_path = model_paths_[0];
      break;
  }
}

void VisionTrackerNode::onRgb(const std::shared_ptr<const Image> & rgb)
{
  if (verifier_reload_) {
    verifier_reload_ = false;
    loadVerifierParams();
  }
  if (pending_target_ || pending_model_) {
    // CPU/GPU or model switch on the tracker thread: recreates the network (tracking restarts).
    if (pending_target_) {tracker_params_.target = *pending_target_;}
    if (pending_model_) {
      tracker_params_.model = *pending_model_;
      setModelPaths(tracker_params_);
    }
    pending_model_.reset();
    tracker_params_.score_threshold = static_cast<float>(get_parameter("tracking_score_threshold").as_double());
    tracker_params_.min_consecutive_frames = static_cast<int>(get_parameter("min_consecutive_frames").as_int());
    pending_target_.reset();
    tracker_ = std::make_unique<VitTracker>(tracker_params_);
    RCLCPP_WARN(get_logger(), "Inference switched: %s (tracking restarted)",
      tracker_->backendDescription().c_str());
  }
  if (!processingDue()) {
    return;
  }
  if (!perception::toBgr(*rgb, frame_bgr_)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Unsupported RGB encoding: %s", rgb->encoding.c_str());
    return;
  }

  VitTracker::Result result;
  std::optional<Localization> loc;

  if (tracker_->active()) {
    const auto t0 = std::chrono::steady_clock::now();
    result = tracker_->update(frame_bgr_);
    const float ms = std::chrono::duration<float, std::milli>(
      std::chrono::steady_clock::now() - t0).count();

    if (result.status == VitTracker::Status::kConfirmed) {
      const auto depth = matchingDepth(*rgb);
      loc = locateTarget(result.box, depth.get(), frame_bgr_.size());
      if (verifier_enabled_ && loc) {
        verify(rgb->header, result, loc);
      }
    } else if (verifier_enabled_ && verifier_.holdExpired(rclcpp::Time(rgb->header.stamp).seconds())) {
      // Hold expired while the tracker itself has no target: give up as well.
      ++verifier_failures_;
      RCLCPP_WARN(get_logger(), "Verifier: tracking dropped (suspended without recovering; failure %d)",
        verifier_failures_);
      last_verifier_ = "failed:hold_timeout";
      tracker_->stop();
      verifier_.reset();
    }
    publishOutputs(rgb->header, result, loc, ms);
  }

  if (debugRequested()) {
    publishDebug(rgb->header, result, loc);
  }
}

void VisionTrackerNode::publishOutputs(
  const std_msgs::msg::Header & header, const VitTracker::Result & result,
  const std::optional<Localization> & loc, float processing_ms)
{
  std_msgs::msg::Header out_header = header;
  out_header.frame_id = camera_frame_;

  if (loc && loc->point) {
    PointStamped out;
    out.header = out_header;
    out.point.x = loc->point->x;
    out.point.y = loc->point->y;
    out.point.z = loc->point->z;
    pub_target_->publish(out);
  } else if (loc && depth_fallback_) {
    // Fallback: no depth points, only the line of sight.
    msg::TargetBearing out;
    out.header = out_header;
    out.direction.x = loc->bearing.direction.x;
    out.direction.y = loc->bearing.direction.y;
    out.direction.z = loc->bearing.direction.z;
    out.azimuth = static_cast<float>(loc->bearing.azimuth);
    out.elevation = static_cast<float>(loc->bearing.elevation);
    out.score = result.score;
    // Angular size of the ROI: its evolution gives the time to contact (fallback).
    const double side = std::sqrt(static_cast<double>(result.box.width) * result.box.height);
    const double focal = std::sqrt(rgb_intrinsics_.fx * rgb_intrinsics_.fy);
    out.angular_size = side > 0.0 && focal > 0.0 ?
      static_cast<float>(2.0 * std::atan(side / (2.0 * focal))) : NAN;
    out.range_lower_bound = loc->beyond_range ? static_cast<float>(depth_max_range_) : NAN;
    pub_bearing_->publish(out);
  }

  auto det = std::make_unique<msg::TargetDetection>();
  det->header = out_header;
  det->status = toMsgStatus(result.status);
  det->score = result.score;
  det->x = result.box.x;
  det->y = result.box.y;
  det->width = result.box.width;
  det->height = result.box.height;
  det->image_width = static_cast<uint32_t>(frame_bgr_.cols);
  det->image_height = static_cast<uint32_t>(frame_bgr_.rows);
  det->has_depth = loc && loc->point.has_value();
  det->depth = loc ? loc->depth : NAN;
  det->depth_beyond_range = loc && loc->beyond_range;
  det->depth_max_range = static_cast<float>(depth_max_range_);
  det->depth_valid_fraction = loc ? loc->valid_fraction : 0.0f;
  det->azimuth = loc ? static_cast<float>(loc->bearing.azimuth) : NAN;
  det->elevation = loc ? static_cast<float>(loc->bearing.elevation) : NAN;
  det->processing_time_ms = processing_ms;
  det->verifier = last_verifier_;
  det->appearance = static_cast<float>(last_appearance_);
  last_verifier_.clear();
  last_appearance_ = NAN;
  pub_detection_->publish(std::move(det));
}

bool VisionTrackerNode::debugRequested()
{
  if (pub_debug_->get_subscription_count() == 0 &&
    pub_debug_compressed_->get_subscription_count() == 0)
  {
    return false;
  }
  if (debug_period_ <= 0.0) {
    return true;
  }
  const rclcpp::Time now = steady_clock_.now();
  if ((now - last_debug_time_).seconds() < debug_period_) {
    return false;
  }
  last_debug_time_ = now;
  return true;
}

void VisionTrackerNode::publishDebug(
  const std_msgs::msg::Header & header, const VitTracker::Result & result,
  const std::optional<Localization> & loc)
{
  // Original resolution or a downscaled copy (less bandwidth to Foxglove and less CPU).
  const double scale = debugScale();
  if (scale < 1.0) {
    cv::resize(frame_bgr_, debug_buf_, cv::Size(), scale, scale, cv::INTER_AREA);
  } else {
    frame_bgr_.copyTo(debug_buf_);
  }

  auto scaled = [scale](const cv::Rect & r) {
      return cv::Rect(
        cvRound(r.x * scale), cvRound(r.y * scale),
        cvRound(r.width * scale), cvRound(r.height * scale));
    };
  const double font = 0.7 * std::max(scale, 0.5);
  const cv::Point banner(15, 30);

  if (roi_.drawing()) {
    cv::rectangle(debug_buf_, scaled(roi_.preview()), cv::Scalar(0, 200, 255), 2);
    perception::drawLabel(debug_buf_, "Click again to confirm box", banner, font,
      cv::Scalar(0, 200, 255));
  } else if (!tracker_->active()) {
    perception::drawLabel(debug_buf_, "Click to start ROI box...", banner, font,
      cv::Scalar(0, 255, 255));
  } else if (result.status == VitTracker::Status::kLost) {
    perception::drawLabel(debug_buf_, "Target Lost", banner, font, cv::Scalar(0, 0, 255));
  } else if (result.status == VitTracker::Status::kConfirming) {
    perception::drawLabel(debug_buf_, "Checking target", banner, font, cv::Scalar(0, 165, 255));
  } else if (result.status == VitTracker::Status::kConfirmed) {
    const bool has_depth = loc && loc->point;
    const cv::Scalar color = has_depth ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 200, 0);
    const cv::Rect box = scaled(result.box);
    cv::rectangle(debug_buf_, box, color, 2);

    char text[128];
    if (has_depth) {
      std::snprintf(text, sizeof(text), "Track %.2f | X:%.2f Y:%.2f Z:%.2f m (optico)",
        result.score, loc->point->x, loc->point->y, loc->point->z);
    } else if (loc) {
      std::snprintf(text, sizeof(text), "Track %.2f | Sem profundidade | az %.1f el %.1f deg",
        result.score, loc->bearing.azimuth * 180.0 / M_PI, loc->bearing.elevation * 180.0 / M_PI);
    } else {
      std::snprintf(text, sizeof(text), "Track %.2f", result.score);
    }
    perception::drawLabel(debug_buf_, text,
      cv::Point(std::max(0, box.x), std::max(20, box.y - 8)), 0.8 * font, color);
  }

  std_msgs::msg::Header out_header = header;
  out_header.frame_id = camera_frame_;

  if (pub_debug_compressed_->get_subscription_count() > 0) {
    auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
    if (perception::toCompressedMsg(debug_buf_, out_header, jpeg_quality_, *msg)) {
      pub_debug_compressed_->publish(std::move(msg));
    }
  }
  if (pub_debug_->get_subscription_count() > 0) {
    auto msg = std::make_unique<Image>();
    perception::toImageMsg(debug_buf_, out_header, *msg);
    pub_debug_->publish(std::move(msg));
  }
}

}  // namespace aero_pursuit

RCLCPP_COMPONENTS_REGISTER_NODE(aero_pursuit::VisionTrackerNode)
