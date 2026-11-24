#include "aero_pursuit/nodes/session_recorder_node.hpp"

#include "aero_pursuit/msg/target_detection.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace aero_pursuit
{

namespace
{
/// Double-quoted YAML scalar (free text such as end_reason may contain ": " or "#").
std::string yamlQuoted(const std::string & text)
{
  std::string out = "\"";
  for (const char c : text) {
    if (c == '"' || c == '\\') {
      out += '\\';
    }
    out += c == '\n' ? ' ' : c;
  }
  return out + "\"";
}
}  // namespace

namespace
{
constexpr size_t kMaxLatchedPerTopic = 64;
constexpr size_t kMaxPreRollMessages = 20000;  ///< Memory cap (e.g. extra_topics with images)

std::string defaultOutputDir()
{
  const char * home = std::getenv("HOME");
  return std::string(home ? home : "/tmp") + "/aero_pursuit/datasets";
}

bool isActiveMode(uint8_t mode)
{
  using S = msg::PursuitStatus;
  return mode == S::TRACKING || mode == S::COASTING || mode == S::APPROACH;
}
}  // namespace

SessionRecorderNode::SessionRecorderNode(const rclcpp::NodeOptions & options)
: Node("session_recorder", options),
  last_active_(0, 0, get_clock()->get_clock_type())
{
  output_dir_ = declare_parameter<std::string>("output_dir", "");
  if (output_dir_.empty()) {
    output_dir_ = defaultOutputDir();
  }
  storage_id_ = declare_parameter<std::string>("storage_id", "mcap");
  mode_ = declare_parameter<std::string>("record_mode", "session");  // session | continuous
  topics_ = declare_parameter<std::vector<std::string>>("topics", {
      "/pursuit/status", "/pursuit/metrics", "/pursuit/predicted_path",
      "/target_detection", "/target_3d_position", "/target_bearing", "/target_bearing_enu",
      "/target_3d_state_filtered",
      "/offboard/trajectory_setpoint", "/fmu/in/trajectory_setpoint",
      "/gimbal/joint_states", "/gimbal/joint_command",
      "/fmu/out/vehicle_local_position_v1", "/fmu/out/vehicle_attitude", "/fmu/out/vehicle_odometry",
      "/drone_joy_teleop/status",
      "/tf", "/tf_static",
      "/scenario/status", "/scenario/paths", "/acquisition/truth", "/target_selection",
      "/X3/gazebo/command/twist", "/X4/gazebo/command/twist",
      // External inputs and the scenario text: what the GCS replay re-injects / reloads.
      "/joy", "/gcs/input_log", "/scenario/source"});
  snapshot_nodes_ = declare_parameter<std::vector<std::string>>("snapshot_nodes", {
      "/pursuit_controller_node", "/vision_tracker_node", "/target_estimator_node",
      "/target_acquisition", "/offboard_control", "/scenario_runner",
      "/drone_joy_teleop"});
  // Additional topics (e.g. images) without repeating the default list.
  for (const auto & t : declare_parameter<std::vector<std::string>>("extra_topics", {""})) {
    if (!t.empty()) {
      topics_.push_back(t);
    }
  }
  pre_roll_ = declare_parameter<double>("pre_roll_sec", 2.0);
  post_roll_ = declare_parameter<double>("post_roll_sec", 3.0);
  enabled_ = declare_parameter<bool>("enabled", true);
  const auto status_topic = declare_parameter<std::string>("status_topic", "/pursuit/status");
  const auto metrics_topic = declare_parameter<std::string>("metrics_topic", "/pursuit/metrics");
  const double discovery_period = declare_parameter<double>("discovery_period_sec", 1.0);

  sub_status_ = create_subscription<msg::PursuitStatus>(
    status_topic, 10, [this](const msg::PursuitStatus & msg) {onStatus(msg);});
  sub_metrics_ = create_subscription<msg::PursuitMetrics>(
    metrics_topic, 10, [this](const msg::PursuitMetrics & msg) {onMetrics(msg);});
  sub_scenario_ = create_subscription<msg::ScenarioStatus>(
    "/scenario/status", rclcpp::QoS(1).reliable().transient_local(),
    [this](const msg::ScenarioStatus & msg) {onScenario(msg);});
  sub_truth_ = create_subscription<msg::TrackingTruth>(
    "/acquisition/truth", 10, [this](const msg::TrackingTruth & msg) {onTruth(msg);});
  sub_position_ = create_subscription<geometry_msgs::msg::PointStamped>(
    "/target_3d_position", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::PointStamped &) {
      if (writer_) {
        stats_.last_position = get_clock()->now();
      }
    });

  discovery_timer_ = create_wall_timer(
    std::chrono::duration<double>(discovery_period), [this]() {discoverTopics();});

  post_param_handle_ = add_post_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      for (const auto & p : params) {
        if (p.get_name() == "enabled") {
          enabled_ = p.as_bool();
          if (!enabled_) {
            closeSession("disabled");
          }
        } else if (p.get_name() == "pre_roll_sec") {
          pre_roll_ = p.as_double();
        } else if (p.get_name() == "post_roll_sec") {
          post_roll_ = p.as_double();
        }
      }
    });

  RCLCPP_INFO(get_logger(), "Session recorder (%s, %s) in '%s': %zu topics",
    mode_.c_str(), storage_id_.c_str(), output_dir_.c_str(), topics_.size());
}

SessionRecorderNode::~SessionRecorderNode()
{
  closeSession("shutdown");
}

rclcpp::QoS SessionRecorderNode::adaptQos(const std::string & topic, bool & latched) const
{
  // Compatible with all publishers: best-effort if any is; latched only if all are.
  const auto infos = get_publishers_info_by_topic(topic);
  bool best_effort = false;
  latched = !infos.empty();
  for (const auto & info : infos) {
    const auto & q = info.qos_profile();
    best_effort |= q.reliability() == rclcpp::ReliabilityPolicy::BestEffort;
    latched &= q.durability() == rclcpp::DurabilityPolicy::TransientLocal;
  }
  rclcpp::QoS qos(latched ? kMaxLatchedPerTopic : 100);
  if (best_effort) {
    qos.best_effort();
  }
  if (latched) {
    qos.transient_local();
  }
  return qos;
}

void SessionRecorderNode::discoverTopics()
{
  const auto graph = get_topic_names_and_types();
  for (const auto & topic : topics_) {
    if (subs_.count(topic) != 0) {
      continue;
    }
    const auto it = graph.find(topic);
    if (it == graph.end() || it->second.empty() || count_publishers(topic) == 0) {
      continue;
    }
    TopicInfo info;
    info.type = it->second.front();
    info.qos = adaptQos(topic, info.latched);
    info.sub = create_generic_subscription(
      topic, info.type, info.qos,
      [this, topic](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
        onMessage(topic, std::move(msg));
      });
    if (writer_) {
      rosbag2_storage::TopicMetadata meta;
      meta.name = topic;
      meta.type = info.type;
      meta.serialization_format = "cdr";
      meta.offered_qos_profiles = {info.qos};
      writer_->create_topic(meta);
    }
    RCLCPP_DEBUG(get_logger(), "Gravando '%s' [%s]", topic.c_str(), info.type.c_str());
    subs_.emplace(topic, std::move(info));
  }

  if (mode_ == "continuous" && enabled_ && !writer_) {
    openSession("continuous");
  }
}

void SessionRecorderNode::onMessage(
  const std::string & topic, std::shared_ptr<const rclcpp::SerializedMessage> msg)
{
  const rclcpp::Time now = get_clock()->now();
  auto & info = subs_.at(topic);
  if (info.latched && info.latched_msgs.size() < kMaxLatchedPerTopic) {
    info.latched_msgs.push_back(msg);
  }

  if (writer_) {
    write(topic, msg, now);
    return;
  }
  if (mode_ != "session" || pre_roll_ <= 0.0 || info.latched) {
    return;
  }
  // Simulation time went back (Gazebo reset): the old buffer would never expire.
  if (!pre_roll_buffer_.empty() && now < pre_roll_buffer_.back().stamp) {
    pre_roll_buffer_.clear();
  }
  pre_roll_buffer_.push_back({topic, std::move(msg), now});
  while (!pre_roll_buffer_.empty() &&
    ((now - pre_roll_buffer_.front().stamp).seconds() > pre_roll_ ||
    pre_roll_buffer_.size() > kMaxPreRollMessages))
  {
    pre_roll_buffer_.pop_front();
  }
}

void SessionRecorderNode::onStatus(const msg::PursuitStatus & msg)
{
  const rclcpp::Time now = get_clock()->now();
  const bool active = isActiveMode(msg.mode);

  if (writer_) {
    if (!stats_.last_mode.empty()) {
      stats_.time_in_mode[stats_.last_mode] += std::max(0.0, (now - stats_.last_status).seconds());
      accumulateAvailability(now);
    }
    stats_.last_mode = msg.mode_name;
    stats_.last_status = now;
    stats_.controller = msg.controller;
    stats_.yaw_mode = msg.yaw_mode;
  }

  if (mode_ != "session" || !enabled_ || scenario_active_) {
    return;
  }
  if (now < last_active_) {
    last_active_ = now;  // Time went back: the post-roll starts counting again
  }
  if (active) {
    last_active_ = now;
    if (!writer_) {
      stats_ = SessionStats{};
      stats_.controller = msg.controller;
      stats_.yaw_mode = msg.yaw_mode;
      stats_.last_mode = msg.mode_name;
      stats_.last_status = now;
      openSession(msg.controller);
    }
  } else if (writer_ && (now - last_active_).seconds() > post_roll_) {
    closeSession(msg.mode_name);
  }
}

void SessionRecorderNode::onMetrics(const msg::PursuitMetrics & msg)
{
  if (writer_) {
    stats_.last_metrics = msg;
    if (std::isfinite(msg.solve_time_us)) {
      stats_.solve_times_us.push_back(msg.solve_time_us);
    }
  }
}

void SessionRecorderNode::onScenario(const msg::ScenarioStatus & msg)
{
  using S = msg::ScenarioStatus;
  scenario_active_ = msg.state == S::STAGING || msg.state == S::RUNNING || msg.state == S::PAUSED;
  if (mode_ != "session" || !enabled_) {
    return;
  }
  const bool running = msg.state == S::RUNNING || msg.state == S::PAUSED;
  const bool ours = writer_ && !stats_.run_id.empty();
  if (ours && (!running || msg.run_id != stats_.run_id)) {
    closeSession(msg.message.empty() ? msg.state_name : msg.message);
  } else if (writer_ && !ours && scenario_active_) {
    closeSession("scenario started");  // Open manual pursuit session
  }
  if (msg.state == S::RUNNING && !writer_) {
    stats_ = SessionStats{};
    stats_.scenario = msg.scenario;
    stats_.variant = msg.variant;
    stats_.run_id = msg.run_id;
    stats_.detection = msg.detection;
    stats_.last_status = get_clock()->now();
    openSession(msg.run_id);
  }
  if (msg.state == S::RUNNING && writer_ && msg.run_id == stats_.run_id) {
    accumulateTargetMotion(msg);
  }
}

void SessionRecorderNode::accumulateTargetMotion(const msg::ScenarioStatus & msg)
{
  const auto vec = [](const auto & v) {return Eigen::Vector3d(v.x, v.y, v.z);};
  const msg::ScenarioDrone * target = nullptr;
  const msg::ScenarioDrone * pursuer = nullptr;
  for (const auto & d : msg.drones) {
    if (d.role == "target" && !target) {target = &d;}
    if (d.role == "pursuer" && !pursuer) {pursuer = &d;}
  }
  if (!target) {
    return;
  }
  const double speed = vec(target->velocity).norm();
  if (std::isfinite(speed)) {
    ++stats_.target_samples;
    stats_.target_speed_sum += speed;
    stats_.target_speed_max = std::max(stats_.target_speed_max, speed);
  }
  const Eigen::Vector3d ref = vec(target->reference);
  if (ref.allFinite() && !target->finished) {
    if (stats_.last_ref && stats_.last_elapsed >= 0.0 && msg.elapsed > stats_.last_elapsed) {
      stats_.ref_length += (ref - *stats_.last_ref).norm();
      stats_.ref_time += msg.elapsed - stats_.last_elapsed;
    }
    stats_.last_ref = ref;
    stats_.last_elapsed = msg.elapsed;
  }
  const double ref_error = (vec(target->position) - ref).norm();
  if (std::isfinite(ref_error) && !target->finished) {
    ++stats_.target_ref_samples;
    stats_.target_ref_error_sum += ref_error;
    stats_.target_ref_error_max = std::max(stats_.target_ref_error_max, ref_error);
  }
  if (pursuer) {
    const double range = (vec(target->position) - vec(pursuer->position)).norm();
    if (std::isfinite(range)) {
      ++stats_.range_samples;
      stats_.range_sum += range;
    }
  }
}

void SessionRecorderNode::accumulateAvailability(const rclcpp::Time & now)
{
  const double dt = std::max(0.0, (now - stats_.last_status).seconds());
  if (dt > 1.0) {
    return;  // a gap in the status stream: nothing known about it
  }
  constexpr double kFresh = 0.2;  // [s] camera frames come every 33 ms, the tracker ~30 Hz
  const auto fresh = [&](const rclcpp::Time & t) {
      return t.nanoseconds() > 0 && (now - t).seconds() < kFresh;
    };
  const bool box_on_target = fresh(stats_.last_box_on_target);
  const bool position = fresh(stats_.last_position);
  stats_.visible_time += fresh(stats_.last_visible) ? dt : 0.0;
  stats_.box_time += fresh(stats_.last_box) ? dt : 0.0;
  stats_.box_on_target_time += box_on_target ? dt : 0.0;
  stats_.position_time += position ? dt : 0.0;
  stats_.position_on_target_time += (position && box_on_target) ? dt : 0.0;
}

void SessionRecorderNode::onTruth(const msg::TrackingTruth & msg)
{
  if (!writer_) {
    return;
  }
  {
    const rclcpp::Time now = get_clock()->now();
    if (msg.target_visible) {
      stats_.last_visible = now;
    }
    if (msg.tracker_status == msg::TargetDetection::CONFIRMED) {
      stats_.last_box = now;
      if (msg.target_visible && msg.on_target) {
        stats_.last_box_on_target = now;
      }
    }
  }
  stats_.acquisitions = msg.acquisitions;
  stats_.identity_switches = msg.identity_switches;
  ++stats_.all_truth_frames;
  if (std::isfinite(msg.target_distance)) {
    stats_.max_target_distance = std::max(stats_.max_target_distance, static_cast<double>(msg.target_distance));
  }
  // Confirmed track that is not the target: drift to the background/decoy, also while the
  // target is out of view (the case that makes APPROACH fly away).
  if (msg.tracker_status == msg::TargetDetection::CONFIRMED && !(msg.target_visible && msg.on_target)) {
    ++stats_.false_track_frames;
  }
  if (!msg.target_visible) {
    return;
  }
  ++stats_.truth_frames;
  if (msg.tracker_status >= msg::TargetDetection::CONFIRMING) {
    ++stats_.tracked_frames;
    stats_.iou_sum += msg.iou;
    stats_.on_target_frames += msg.on_target ? 1 : 0;
  }
}

std::string SessionRecorderNode::makeSessionPath(const std::string & label) const
{
  const std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm tm{};
  localtime_r(&t, &tm);
  std::ostringstream base;
  base << output_dir_ << "/" << std::put_time(&tm, "%Y%m%d_%H%M%S") << "_" << label;

  std::string path = base.str();
  for (int i = 1; std::filesystem::exists(path); ++i) {
    path = base.str() + "_" + std::to_string(i);
  }
  return path;
}

void SessionRecorderNode::openSession(const std::string & label)
{
  try {
    std::filesystem::create_directories(output_dir_);
    rosbag2_storage::StorageOptions storage;
    storage.uri = makeSessionPath(label.empty() ? "session" : label);
    storage.storage_id = storage_id_;

    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(storage);
    for (const auto & [topic, info] : subs_) {
      rosbag2_storage::TopicMetadata meta;
      meta.name = topic;
      meta.type = info.type;
      meta.serialization_format = "cdr";
      meta.offered_qos_profiles = {info.qos};
      writer_->create_topic(meta);
    }

    const rclcpp::Time now = get_clock()->now();
    if (stats_.start.nanoseconds() == 0) {
      stats_.start = now;
    }
    stats_.path = storage.uri;
    for (const auto & [topic, info] : subs_) {
      for (const auto & m : info.latched_msgs) {
        write(topic, m, now);
      }
    }
    for (const auto & b : pre_roll_buffer_) {
      write(b.topic, b.msg, b.stamp);
    }
    pre_roll_buffer_.clear();
    ++session_count_;
    snapshotForReplay(storage.uri, now);
    RCLCPP_INFO(get_logger(), "Session %u: recording to '%s'", session_count_, storage.uri.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Could not open the rosbag: %s", e.what());
    writer_.reset();
  }
}

void SessionRecorderNode::write(
  const std::string & topic, const std::shared_ptr<const rclcpp::SerializedMessage> & msg,
  const rclcpp::Time & stamp)
{
  try {
    writer_->write(msg, topic, subs_.at(topic).type, stamp);
    ++stats_.message_count[topic];
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Could not record '%s': %s",
      topic.c_str(), e.what());
  }
}

void SessionRecorderNode::closeSession(const std::string & reason)
{
  if (!writer_) {
    return;
  }
  writer_.reset();  // Closes the bag (cache flush and metadata.yaml)
  writeSessionInfo(reason);
  RCLCPP_INFO(get_logger(), "Session %u closed (%s): '%s'", session_count_, reason.c_str(),
    stats_.path.c_str());
  stats_ = SessionStats{};
}

void SessionRecorderNode::writeSessionInfo(const std::string & reason) const
{
  std::ofstream out(stats_.path + "/session_info.yaml");
  if (!out) {
    return;
  }
  const rclcpp::Time now = get_clock()->now();
  out << std::fixed << std::setprecision(3);
  out << "session: " << session_count_ << "\n";
  out << "start_time: " << stats_.start.seconds() << "\n";
  out << "duration: " << (now - stats_.start).seconds() << "\n";
  out << "end_reason: " << yamlQuoted(reason) << "\n";
  if (!stats_.run_id.empty()) {
    out << "scenario: " << yamlQuoted(stats_.scenario) << "\n";
    out << "variant: " << yamlQuoted(stats_.variant) << "\n";
    out << "run_id: " << yamlQuoted(stats_.run_id) << "\n";
    out << "detection: " << yamlQuoted(stats_.detection) << "\n";
  }
  out << "controller: " << yamlQuoted(stats_.controller) << "\n";
  out << "yaw_mode: " << yamlQuoted(stats_.yaw_mode) << "\n";
  out << "time_in_mode:\n";
  for (const auto & [mode, t] : stats_.time_in_mode) {
    out << "  " << mode << ": " << t << "\n";
  }
  out << "availability:  # [s] over the session, like time_in_mode (tracker ~30 Hz, fresh = < 0.2 s)\n";
  out << "  visible_time: " << stats_.visible_time << "\n";                 // target in the image (truth)
  out << "  box_time: " << stats_.box_time << "\n";                         // tracker confirmed, any object
  out << "  box_on_target_time: " << stats_.box_on_target_time << "\n";     // ... the right target
  out << "  position_time: " << stats_.position_time << "\n";               // 3D position published
  out << "  position_on_target_time: " << stats_.position_on_target_time << "\n";
  if (const auto & m = stats_.last_metrics) {
    out << "metrics:  # Accumulated over the last engagement (PursuitMetrics)\n";
    out << "  rmse_position: " << m->rmse_position << "\n";
    out << "  rmse_velocity: " << m->rmse_velocity << "\n";
    out << "  rmse_los: " << m->rmse_los << "\n";
    out << "  rmse_image: " << m->rmse_image << "\n";
    out << "  max_position_error: " << m->max_position_error << "\n";
    out << "  effort_l2: " << m->effort_l2 << "\n";
    out << "  effort_l1: " << m->effort_l1 << "\n";
    out << "  engaged_time: " << m->engaged_time << "\n";
  }
  if (auto t = stats_.solve_times_us; !t.empty()) {
    // Control law (MPC solve) wall time per cycle over the whole session [µs].
    std::sort(t.begin(), t.end());
    const auto pct = [&t](double q) {return t[static_cast<size_t>(q * (t.size() - 1))];};
    double sum = 0.0;
    for (const float v : t) {
      sum += v;
    }
    out << "solve_time:  # Control law computation per cycle (PursuitMetrics.solve_time_us) [us]\n";
    out << "  samples: " << t.size() << "\n";
    out << "  mean_us: " << sum / t.size() << "\n";
    out << "  p50_us: " << pct(0.5) << "\n";
    out << "  p95_us: " << pct(0.95) << "\n";
    out << "  p99_us: " << pct(0.99) << "\n";
    out << "  max_us: " << t.back() << "\n";
  }
  if (stats_.target_samples > 0) {
    out << "target:  # Target motion while RUNNING (/scenario/status, ground truth)\n";
    out << "  mean_speed: " << stats_.target_speed_sum / stats_.target_samples << "\n";
    out << "  max_speed: " << stats_.target_speed_max << "\n";
    if (stats_.ref_time > 0.0) {
      // Speed of the reference along the path (what was asked); mean_speed is |v| of the drone,
      // larger when it weaves around the path.
      out << "  mean_reference_speed: " << stats_.ref_length / stats_.ref_time << "\n";
    }
    if (stats_.target_ref_samples > 0) {
      out << "  mean_reference_error: " << stats_.target_ref_error_sum / stats_.target_ref_samples << "\n";
      out << "  max_reference_error: " << stats_.target_ref_error_max << "\n";
    }
    if (stats_.range_samples > 0) {
      out << "  mean_range: " << stats_.range_sum / stats_.range_samples << "\n";
    }
  }
  if (stats_.truth_frames > 0 || stats_.acquisitions > 0) {
    const auto ratio = [](size_t a, size_t b) {return b ? static_cast<double>(a) / b : 0.0;};
    out << "tracking:  # Tracker vs ground truth (/acquisition/truth), frames with the target visible\n";
    out << "  visible_frames: " << stats_.truth_frames << "\n";
    out << "  tracked_ratio: " << ratio(stats_.tracked_frames, stats_.truth_frames) << "\n";
    out << "  on_target_ratio: " << ratio(stats_.on_target_frames, stats_.truth_frames) << "\n";
    out << "  mean_iou: " << (stats_.tracked_frames ? stats_.iou_sum / stats_.tracked_frames : 0.0) << "\n";
    out << "  acquisitions: " << stats_.acquisitions << "\n";
    out << "  identity_switches: " << stats_.identity_switches << "\n";
    out << "  false_track_ratio: " << ratio(stats_.false_track_frames, stats_.all_truth_frames) << "\n";
    out << "  max_target_distance: " << stats_.max_target_distance << "\n";
  }
  out << "message_count:\n";
  for (const auto & [topic, n] : stats_.message_count) {
    out << "  \"" << topic << "\": " << n << "\n";
  }
}

}  // namespace aero_pursuit

RCLCPP_COMPONENTS_REGISTER_NODE(aero_pursuit::SessionRecorderNode)
