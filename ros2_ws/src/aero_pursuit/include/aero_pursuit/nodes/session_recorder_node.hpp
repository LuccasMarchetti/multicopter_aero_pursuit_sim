#pragma once

#include "aero_pursuit/msg/pursuit_metrics.hpp"
#include "aero_pursuit/msg/pursuit_status.hpp"
#include "aero_pursuit/msg/scenario_status.hpp"
#include "aero_pursuit/msg/tracking_truth.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>

#include <deque>
#include <map>
#include <memory>
#include <optional>

#include <Eigen/Core>
#include <string>
#include <vector>

namespace aero_pursuit
{

/// Automatically records each pursuit session in a rosbag (MCAP) to
/// build the dataset: a session starts when the supervisor engages (TRACKING,
/// COASTING or APPROACH) and ends `post_roll_sec` after leaving those modes.
/// Messages from the `pre_roll_sec` before engagement are also recorded
/// (captures the target selection), and latched topics (/tf_static) are repeated at the
/// start of each bag. Each session gets a `session_info.yaml` with the summary.
///
/// With a scenario running (scenario_runner), each scenario run is a
/// session (`<date>_<run_id>`), from RUNNING to the end, with the scenario summary and the
/// tracking ground truth (IoU, time on target, identity switches) in the yaml.
///
/// `record_mode`: session (one bag per session) | continuous (one bag per execution).
/// Types are discovered from the graph (generic subscriptions, no deserialization).
class SessionRecorderNode : public rclcpp::Node
{
public:
  explicit SessionRecorderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SessionRecorderNode() override;

private:
  struct TopicInfo
  {
    std::string type;
    rclcpp::QoS qos{10};
    bool latched = false;
    rclcpp::GenericSubscription::SharedPtr sub;
    std::vector<std::shared_ptr<const rclcpp::SerializedMessage>> latched_msgs;
  };

  struct Buffered
  {
    std::string topic;
    std::shared_ptr<const rclcpp::SerializedMessage> msg;
    rclcpp::Time stamp;
  };

  struct SessionStats
  {
    std::string path;
    rclcpp::Time start{0, 0, RCL_ROS_TIME};
    std::string controller;
    std::string yaw_mode;
    std::map<std::string, double> time_in_mode;
    std::map<std::string, size_t> message_count;
    std::string last_mode;
    rclcpp::Time last_status{0, 0, RCL_ROS_TIME};
    std::optional<msg::PursuitMetrics> last_metrics;  ///< Totals at the end of the session
    std::vector<float> solve_times_us;  ///< Control law time of every metrics sample (whole session)
    // Scenario (empty in sessions triggered by the pursuit)
    std::string scenario;
    std::string variant;
    std::string run_id;
    std::string detection;
    // Tracking ground truth (/acquisition/truth)
    size_t truth_frames = 0;       ///< Frames with the target visible
    size_t tracked_frames = 0;     ///< ... with the tracker active
    size_t on_target_frames = 0;   ///< ... tracking the right target
    double iou_sum = 0.0;
    size_t all_truth_frames = 0;   ///< Every tracker frame (target visible or not)
    size_t false_track_frames = 0; ///< Tracker confident but not on the target (incl. target out of view)
    double max_target_distance = 0.0;  ///< Largest pursuer-target distance [m] (runaway)
    uint32_t acquisitions = 0;
    uint32_t identity_switches = 0;
    // Target motion from /scenario/status while RUNNING (did it fly the nominal speed?)
    size_t target_samples = 0;
    double target_speed_sum = 0.0;
    double target_speed_max = 0.0;
    size_t target_ref_samples = 0;
    double target_ref_error_sum = 0.0;  ///< |position - reference| [m]
    double target_ref_error_max = 0.0;
    double ref_length = 0.0;            ///< Path the reference moved along [m]
    double ref_time = 0.0;              ///< ... over this time [s]
    std::optional<Eigen::Vector3d> last_ref;
    double last_elapsed = -1.0;
    size_t range_samples = 0;
    double range_sum = 0.0;             ///< Pursuer-target distance (ground truth) [m]
    // Availability of the target information, integrated over the session like time_in_mode
    // [s]: each pursuit status tick adds its interval to every item that was fresh (< 0.2 s).
    double visible_time = 0.0;              ///< target inside the image (ground truth)
    double box_time = 0.0;                  ///< tracker CONFIRMED (a box, any object)
    double box_on_target_time = 0.0;        ///< ... on the right target (ground truth)
    double position_time = 0.0;             ///< 3D measurement published (/target_3d_position)
    double position_on_target_time = 0.0;   ///< ... while the box was on the right target
    rclcpp::Time last_visible{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_box{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_box_on_target{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_position{0, 0, RCL_ROS_TIME};
  };

  void discoverTopics();
  rclcpp::QoS adaptQos(const std::string & topic, bool & latched) const;
  void onMessage(const std::string & topic, std::shared_ptr<const rclcpp::SerializedMessage> msg);
  void onStatus(const msg::PursuitStatus & msg);
  void onMetrics(const msg::PursuitMetrics & msg);
  void onScenario(const msg::ScenarioStatus & msg);
  void accumulateTargetMotion(const msg::ScenarioStatus & msg);
  void onTruth(const msg::TrackingTruth & msg);
  /// Adds the interval since the previous pursuit status to the availability times.
  void accumulateAvailability(const rclcpp::Time & now);

  void openSession(const std::string & label);
  void closeSession(const std::string & reason);
  void write(const std::string & topic, const std::shared_ptr<const rclcpp::SerializedMessage> & msg,
    const rclcpp::Time & stamp);
  void writeSessionInfo(const std::string & reason) const;
  std::string makeSessionPath(const std::string & label) const;
  /// replay.yaml in the session folder: simulation environment and every parameter of
  /// `snapshot_nodes` at the start, read asynchronously (GCS replay re-simulates from it).
  void snapshotForReplay(const std::string & dir, const rclcpp::Time & start);

  // Configuration
  std::string output_dir_;
  std::string storage_id_;
  std::string mode_;
  std::vector<std::string> topics_;
  std::vector<std::string> snapshot_nodes_;
  double pre_roll_ = 2.0;
  double post_roll_ = 3.0;
  bool enabled_ = true;

  // Estado
  std::map<std::string, TopicInfo> subs_;
  std::deque<Buffered> pre_roll_buffer_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  SessionStats stats_;
  rclcpp::Time last_active_;
  bool session_active_ = false;  ///< Supervisor in an active mode
  bool scenario_active_ = false; ///< Scenario staging/running: it defines the sessions
  unsigned session_count_ = 0;

  rclcpp::Subscription<msg::PursuitStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<msg::PursuitMetrics>::SharedPtr sub_metrics_;
  rclcpp::Subscription<msg::ScenarioStatus>::SharedPtr sub_scenario_;
  rclcpp::Subscription<msg::TrackingTruth>::SharedPtr sub_truth_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_position_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_param_handle_;
};

}  // namespace aero_pursuit
