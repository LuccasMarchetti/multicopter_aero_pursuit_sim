#pragma once

#include "aero_pursuit/msg/scenario_status.hpp"
#include "aero_pursuit/msg/target_detection.hpp"
#include "aero_pursuit/msg/target_tf_cmd.hpp"
#include "aero_pursuit/scenario/agent_executor.hpp"
#include "aero_pursuit/scenario/guidance.hpp"
#include "aero_pursuit/scenario/spec.hpp"
#include "aero_pursuit/sim/world.hpp"
#include "aero_pursuit/srv/scenario_command.hpp"
#include "aero_pursuit/srv/set_drones.hpp"

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace aero_pursuit
{

/// Runs scenarios (scenarios/*.yaml): spawns the missing drones in Gazebo, takes
/// everyone to the start positions (staging), runs each drone's plans from
/// ground truth, configures the pursuer (parameters, detection, engagement) and
/// repeats the run `repeat` times. The target drones are commanded directly through
/// gz-transport (any number, no bridge entries); the main pursuer
/// through Offboard velocity setpoints.
///
/// Interface: service ~/command (aero_pursuit/srv/ScenarioCommand), /scenario/status
/// (5 Hz + transitions), /scenario/paths (latched MarkerArray, Foxglove) and TFs
/// `<world> -> <drone>` of the models inserted by the scenario.
class ScenarioRunnerNode : public rclcpp::Node
{
public:
  explicit ScenarioRunnerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Status = msg::ScenarioStatus;
  using Command = srv::ScenarioCommand;
  using SetDrones = srv::SetDrones;

  /// Base-world drone (spawned by launch_gz_and_drones.sh): name, catalog key, default spawn.
  struct BaseDrone
  {
    std::string name;
    std::string model;
    Eigen::Vector3d spawn;
  };
  using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;

  struct ModelEntry
  {
    std::string sdf;      ///< SDF text (namespace swapped on spawn)
    double radius = 0.4;  ///< Raio envolvente [m]
  };

  /// Drone of the running scenario.
  struct Agent
  {
    std::unique_ptr<scenario::AgentExecutor> executor;
    scenario::TrackerGains gains;
    bool present = false;       ///< Exists in Gazebo
    std::optional<Eigen::Vector3d> reference;
  };

  /// PX4 vehicle commanded through Offboard (the pursuer).
  struct Px4Vehicle
  {
    std::string name;           ///< Model in Gazebo (x500_depth_<i>)
    std::string ns;             ///< "" or /px4_<i>
    std::optional<Eigen::Vector3d> start;
    std::string offboard_state;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_state;
  };

  // Comandos
  void onCommand(const Command::Request & req, Command::Response & res);
  void load(const std::string & yaml_text, const std::string & variant);
  void start();
  void stop(const std::string & reason);
  void unload();

  // Loop
  void tick();
  void updateStates();
  void stepStaging(double now);
  void stepRunning(double now);
  void beginRun(double now);
  void endRun(double now, const std::string & reason);
  void holdAll();
  void transition(uint8_t state, const std::string & message = "");

  // Actuation
  /// `speed_scale`: the reference velocity is already scaled; the speed limit follows it.
  void commandDrone(
    const std::string & name, Agent & agent, const scenario::Reference & ref, double speed_scale = 1.0);
  /// Effective time scale of the target plans (parameter × scenario field).
  /// Clock scale of the target plans: target_speed / nominal speed of the pursued drone's plan
  /// (1 without a speed command; the last scale while its plan is stopped).
  double targetSpeedScale();
  void commandPx4(Px4Vehicle & v, const scenario::AgentState & self, const scenario::Reference & ref,
    double max_speed);
  bool px4Ready(const Px4Vehicle & v) const;
  void spawnMissing();
  void drainSpawnResults();
  void teleportToSpawn();
  void setPursuit(bool enabled);
  void configurePerception(bool running);
  /// Sends the run's parameters; a node not discovered yet (e.g. the pursuer on another computer
  /// right after it starts) is retried every 0.5 s for up to 15 s instead of being skipped.
  void applyParameters(const std::vector<std::string> & only = {});
  /// Reads the run's parameters back from each node and compares them with the scenario's; a
  /// node with a different value (not applied yet, or overwritten, e.g. by the GCS re-applying
  /// remembered values when the node appears) gets them again. Staging only ends after a round
  /// in which every value matched (params_verified_), so no run starts with other parameters.
  void verifyParameters(double now_s);
  void enableTargets();

  // Outputs
  void publishStatus();
  void publishPaths();
  void publishTransforms();

  std::string resolvePath(const std::string & path) const;
  std::string runId() const;
  const scenario::AgentState * state(const std::string & name) const;

  // Configuration
  std::string world_frame_;
  bool publish_model_tf_ = true;  ///< TFs of the spawned models (off when the simulator sends them)
  std::string scenario_dir_;
  std::map<std::string, ModelEntry> catalog_;
  scenario::ExecutorConfig executor_config_;
  scenario::TrackerGains default_gains_;
  double pursuer_max_speed_ = 4.0;
  double velocity_filter_ = 0.4;
  double period_ = 0.05;
  bool publish_tf_ = true;
  std::string autostart_file_;

  // Mundo
  std::unique_ptr<sim::World> gz_;  ///< Gazebo (gz-transport) or ROS topics (Isaac Sim)
  scenario::WorldView world_;
  std::map<std::string, double> state_stamp_;  ///< Time of the last pose used per model
  double pose_stamp_ = 0.0;
  std::set<std::string> spawned_;         ///< Spawned by this node (removed on unload)
  std::set<std::string> preexisting_;     ///< Already in the world when loading (kept on unload)
  std::map<std::string, double> spawn_pending_;  ///< Name -> request instant
  std::mutex spawn_mutex_;                       ///< Responses arrive on the gz-transport thread
  std::vector<std::pair<std::string, bool>> spawn_results_;

  // Scenario
  std::optional<scenario::Scenario> scenario_;
  std::string variant_;
  std::map<std::string, Agent> agents_;
  std::unique_ptr<scenario::AgentExecutor> pursuer_plan_;
  Px4Vehicle pursuer_;
  uint8_t state_ = Status::IDLE;
  std::string message_;
  int run_index_ = 0;
  double run_start_ = 0.0;
  double paused_at_ = 0.0;
  double staging_start_ = 0.0;
  double staged_since_ = -1.0;
  std::optional<Eigen::Vector3d> pursuer_hold_;  ///< Position held before engagement
  /// PX4 controls the ESTIMATED heading (EKF), which differs from the true one by a bias (world
  /// field declination × WMM): slow integrator on the true error, added to the command.
  double yaw_correction_ = 0.0;
  double yaw_correction_t_ = -1.0;
  double facingYaw(double desired, double actual, double now_s);
  /// Pursuer ENU heading to the target (target start during staging, current position afterwards).
  std::optional<double> yawToTarget(const Eigen::Vector3d & from, bool use_start) const;
  double elapsed_ = 0.0;
  double target_clock_ = 0.0;  ///< Time of the target plans (advances dt × speed scale)
  double target_nominal_speed_ = 0.0;  ///< Plan speed of the pursued drone at the last step [m/s]
  double target_scale_ = 1.0;          ///< Last clock scale (kept while the plan is stopped)
  std::set<std::string> recovered_;   ///< Targets moved back to their start in this staging
  double start_accel_ = 3.0;           ///< Soft start of the target plans [m/s²] (0 = step)
  bool live_speed_override_ = false;   ///< target_speed was set during the current scenario's runs
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr speed_param_cb_;
  bool engaged_ = false;
  bool target_confirmed_ = false;
  double last_tick_ = -1.0;
  double last_enable_ = -1e9;
  double last_status_ = -1e9;

  // Interfaces ROS
  rclcpp::Service<Command>::SharedPtr srv_command_;
  rclcpp::Service<SetDrones>::SharedPtr srv_drones_;
  std::vector<BaseDrone> base_drones_;
  std::map<std::string, double> base_enable_until_;  ///< Respawned base drones: enable resend
  /// Base drones to lift to their spawn height (they fall before the Gazebo velocity
  /// controller is enabled): name -> deadline [s]. Only while no scenario flies them.
  std::map<std::string, double> base_liftoff_until_;
  bool base_liftoff_started_ = false;
  void liftBaseDrones(double now_s);
  /// Base drones re-inserted by ~/set_drones: no PosePublisher (gz_pose_tf.sh runs at launch),
  /// so their TF comes from here.
  std::set<std::string> base_respawned_;
  void onSetDrones(const SetDrones::Request & req, SetDrones::Response & res);
  rclcpp::Publisher<Status>::SharedPtr pub_status_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_paths_;
  /// Text of the loaded scenario (latched): the session recorder keeps it in the bag so a
  /// session can be simulated again (GCS replay).
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_source_;
  rclcpp::Publisher<msg::TargetTfCmd>::SharedPtr pub_tf_cmd_;
  rclcpp::Publisher<sensor_msgs::msg::RegionOfInterest>::SharedPtr pub_selection_;
  rclcpp::Subscription<msg::TargetDetection>::SharedPtr sub_detection_;
  /// Pitch gimbal, commanded and measured: staging waits for the camera at its command (a joint
  /// stuck at its limit in Gazebo kept the camera 31.5 deg up for 24 runs, the target never in view).
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_gimbal_cmd_, sub_gimbal_state_;
  std::optional<std::pair<double, rclcpp::Time>> gimbal_cmd_, gimbal_meas_;  ///< [rad], received
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr pursuit_enable_;
  std::map<std::string, std::shared_ptr<rclcpp::AsyncParametersClient>> param_clients_;
  rclcpp::TimerBase::SharedPtr param_retry_timer_;
  int param_retry_count_ = 0;
  bool params_verified_ = true;    ///< last verification round: every value as in the scenario
  int params_checks_pending_ = 0;  ///< nodes of the current round still to answer
  bool params_round_ok_ = true;
  std::string params_round_text_;  ///< first difference found in the current round
  std::string params_mismatch_;    ///< first difference of the last round (shown while staging)
  double params_check_t_ = -1e9;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;
};

}  // namespace aero_pursuit
