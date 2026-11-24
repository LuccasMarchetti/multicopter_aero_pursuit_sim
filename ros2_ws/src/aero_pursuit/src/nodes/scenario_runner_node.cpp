#include "aero_pursuit/nodes/scenario_runner_node.hpp"

#include "aero_pursuit/common/frames.hpp"
#include "aero_pursuit/scenario/scenario_io.hpp"
#include "aero_pursuit/sim/ros_world.hpp"
#ifdef AERO_HAVE_GZ
#include "aero_pursuit/sim/gz_world.hpp"
#endif

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace aero_pursuit
{
namespace
{
namespace fs = std::filesystem;
using scenario::AgentState;
using scenario::Reference;

constexpr double kStatusPeriod = 0.2;   ///< /scenario/status a 5 Hz
constexpr double kEnablePeriod = 1.0;   ///< Resend of the velocity controllers' enable
constexpr double kMinStaging = 1.5;     ///< Minimum staging time (post-teleport poses)
constexpr double kAutostartWait = 120.0;

const char * stateName(uint8_t s)
{
  switch (s) {
    case msg::ScenarioStatus::IDLE: return "IDLE";
    case msg::ScenarioStatus::LOADED: return "LOADED";
    case msg::ScenarioStatus::STAGING: return "STAGING";
    case msg::ScenarioStatus::RUNNING: return "RUNNING";
    case msg::ScenarioStatus::PAUSED: return "PAUSED";
    case msg::ScenarioStatus::FINISHED: return "FINISHED";
    case msg::ScenarioStatus::ERROR: return "ERROR";
  }
  return "?";
}

std::string readFile(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("could not open " + path);
  }
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d & v)
{
  geometry_msgs::msg::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}

sim::ModelPose poseAt(const Eigen::Vector3d & p, double yaw)
{
  sim::ModelPose pose;
  pose.position = p;
  pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  return pose;
}

/// Scenario value -> parameter of the type declared by the node (5 in a double becomes 5.0).
std::optional<rclcpp::Parameter> toParameter(
  const std::string & name, rclcpp::ParameterType declared, const scenario::ParamValue & value)
{
  using T = rclcpp::ParameterType;
  const auto * d = std::get_if<double>(&value);
  const auto * i = std::get_if<std::int64_t>(&value);
  switch (declared) {
    case T::PARAMETER_DOUBLE:
      if (d) {return rclcpp::Parameter(name, *d);}
      if (i) {return rclcpp::Parameter(name, static_cast<double>(*i));}
      return std::nullopt;
    case T::PARAMETER_INTEGER:
      if (i) {return rclcpp::Parameter(name, *i);}
      if (d && std::floor(*d) == *d) {return rclcpp::Parameter(name, static_cast<std::int64_t>(*d));}
      return std::nullopt;
    case T::PARAMETER_DOUBLE_ARRAY:
      if (const auto * v = std::get_if<std::vector<double>>(&value)) {return rclcpp::Parameter(name, *v);}
      return std::nullopt;
    case T::PARAMETER_INTEGER_ARRAY:
      if (const auto * v = std::get_if<std::vector<double>>(&value)) {
        return rclcpp::Parameter(name, std::vector<std::int64_t>(v->begin(), v->end()));
      }
      return std::nullopt;
    case T::PARAMETER_STRING_ARRAY:
      if (const auto * v = std::get_if<std::vector<std::string>>(&value)) {return rclcpp::Parameter(name, *v);}
      return std::nullopt;
    default:
      break;
  }
  // Bool, string or not declared: the natural type of the value.
  return std::visit([&](const auto & v) {return std::optional<rclcpp::Parameter>(rclcpp::Parameter(name, v));}, value);
}

}  // namespace

ScenarioRunnerNode::ScenarioRunnerNode(const rclcpp::NodeOptions & options)
: Node("scenario_runner", options)
{
  const auto world = declare_parameter<std::string>("world_name", "lawn");
  const auto world_backend = declare_parameter<std::string>("world_backend", "gz");
  world_frame_ = declare_parameter<std::string>("world_frame", world);
  // false when the simulator publishes /tf of every model itself (Isaac Sim, isaac_world.launch.xml).
  publish_model_tf_ = declare_parameter<bool>("publish_model_tf", true);
  scenario_dir_ = declare_parameter<std::string>("scenario_dir", "");
  auto models_dir = declare_parameter<std::string>("models_dir", "");
  const auto catalog = declare_parameter<std::vector<std::string>>(
    "model_catalog", {"x3:x3:0.35", "x4:x4:0.45", "tiltrotor:tiltrotor:0.6", "x5:x5:0.4"});
  const double rate = declare_parameter<double>("rate_hz", 20.0);
  autostart_file_ = declare_parameter<std::string>("autostart_file", "");
  variant_ = declare_parameter<std::string>("variant", "");
  executor_config_.transit_speed = declare_parameter<double>("transit_speed", 3.0);
  executor_config_.transit_threshold = declare_parameter<double>("transit_threshold", 1.5);
  default_gains_.position_gain = declare_parameter<double>("position_gain", 1.2);
  default_gains_.yaw_gain = declare_parameter<double>("yaw_gain", 1.5);
  default_gains_.max_yaw_rate = declare_parameter<double>("max_yaw_rate", 1.2);
  pursuer_max_speed_ = declare_parameter<double>("pursuer_max_speed", 4.0);
  {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = "Speed of the pursued drone along its plan [m/s]; 0 = the plan's (or the scenario "
      "target_speed). Same paths; the other targets keep pace on the same clock. Live.";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 15.0;
    range.step = 0.0;
    d.floating_point_range.push_back(range);
    declare_parameter<double>("target_speed", 0.0, d);
    // Only a change while a run is going counts as overriding a scenario that fixes the speed:
    // values set before (the GCS re-applies the remembered slider when the node appears, maybe
    // after the scenario loaded) would silently replace the speed of every run.
    speed_param_cb_ = add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & p : params) {
          if (p.get_name() == "target_speed") {
            live_speed_override_ = state_ == Status::RUNNING || state_ == Status::PAUSED;
          }
        }
      });
  }
  start_accel_ = std::max(0.0, declare_parameter<double>("target_start_accel", 3.0));  // [m/s²], 0 = step
  velocity_filter_ = std::clamp(declare_parameter<double>("velocity_filter", 0.4), 0.05, 1.0);
  publish_tf_ = declare_parameter<bool>("publish_tf", true);

  if (models_dir.empty()) {
    models_dir = ament_index_cpp::get_package_share_directory("aero_pursuit") + "/gz_models";
  }
  for (const auto & entry : catalog) {
    // "<chave>:<pasta>:<raio>"
    std::stringstream ss(entry);
    std::string key, dir, radius;
    std::getline(ss, key, ':');
    std::getline(ss, dir, ':');
    std::getline(ss, radius, ':');
    try {
      // The ROS world (Isaac Sim) builds its own geometry from the key: the SDF is optional.
      std::string sdf;
      try {
        sdf = readFile(models_dir + "/" + dir + "/model.sdf");
      } catch (const std::exception &) {
        if (world_backend != "ros") {
          throw;
        }
      }
      catalog_[key] = {sdf, radius.empty() ? 0.4 : std::stod(radius)};
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Catalog: model '%s' ignored (%s)", key.c_str(), e.what());
    }
  }

  if (world_backend == "ros") {
    gz_ = std::make_unique<sim::RosWorld>(*this, world);
  } else {
#ifdef AERO_HAVE_GZ
    gz_ = std::make_unique<sim::GzWorld>(world);
#else
    throw std::runtime_error("world_backend '" + world_backend + "' unavailable (built without Gazebo): use ros");
#endif
  }
  if (!gz_->subscribePoses()) {
    throw std::runtime_error("could not subscribe to the world poses " + world);
  }
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  pub_status_ = create_publisher<Status>("/scenario/status", latched);
  pub_paths_ = create_publisher<visualization_msgs::msg::MarkerArray>("/scenario/paths", latched);
  pub_source_ = create_publisher<std_msgs::msg::String>("/scenario/source", latched);
  pub_tf_cmd_ = create_publisher<msg::TargetTfCmd>("/target_tf_cmd", 10);
  pub_selection_ = create_publisher<sensor_msgs::msg::RegionOfInterest>("/target_selection", 10);
  sub_detection_ = create_subscription<msg::TargetDetection>(
    "/target_detection", rclcpp::SensorDataQoS(), [this](const msg::TargetDetection & d) {
      target_confirmed_ = target_confirmed_ || d.status == msg::TargetDetection::CONFIRMED;
    });
  sub_gimbal_cmd_ = create_subscription<sensor_msgs::msg::JointState>(
    "/gimbal/joint_command", 10, [this](const sensor_msgs::msg::JointState & m) {
      if (!m.position.empty()) {
        gimbal_cmd_ = {m.position[0], now()};
      }
    });
  sub_gimbal_state_ = create_subscription<sensor_msgs::msg::JointState>(
    "/gimbal/joint_states", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::JointState & m) {
      for (std::size_t i = 0; i < m.name.size() && i < m.position.size(); ++i) {
        if (m.name[i] == "gimbal_pitch_joint") {
          gimbal_meas_ = {m.position[i], now()};
        }
      }
    });
  pursuit_enable_ = create_client<std_srvs::srv::SetBool>("/pursuit_controller_node/enable");

  pursuer_.pub = create_publisher<TrajectorySetpoint>("/offboard/trajectory_setpoint", 10);
  pursuer_.sub_state = create_subscription<std_msgs::msg::String>(
    "/offboard_control/state", latched,
    [this](const std_msgs::msg::String & m) {pursuer_.offboard_state = m.data;});

  for (const auto & entry : declare_parameter<std::vector<std::string>>("base_drones",
    {"X3:x3:5:2:1", "X4:x4:5:-2:1", "Tiltrotor:tiltrotor:5:-4:1", "X5:x5:5:4:1"}))
  {
    // "<name>:<catalog key>:<x>:<y>:<z>" (same as launch_gz_and_drones.sh)
    std::stringstream ss(entry);
    std::vector<std::string> f;
    for (std::string item; std::getline(ss, item, ':');) {
      f.push_back(item);
    }
    if (f.size() != 5 || !catalog_.count(f[1])) {
      RCLCPP_WARN(get_logger(), "base_drones: invalid entry '%s'", entry.c_str());
      continue;
    }
    base_drones_.push_back({f[0], f[1], Eigen::Vector3d(std::stod(f[2]), std::stod(f[3]), std::stod(f[4]))});
  }
  srv_drones_ = create_service<SetDrones>(
    "~/set_drones", [this](const std::shared_ptr<SetDrones::Request> req, std::shared_ptr<SetDrones::Response> res) {
      onSetDrones(*req, *res);
    });

  srv_command_ = create_service<Command>(
    "~/command", [this](const std::shared_ptr<Command::Request> req, std::shared_ptr<Command::Response> res) {
      onCommand(*req, *res);
    });

  period_ = 1.0 / std::max(rate, 1.0);
  timer_ = create_timer(std::chrono::duration<double>(period_), [this] {tick();});
  publishStatus();

  if (!autostart_file_.empty()) {
    const double t0 = now().seconds();
    autostart_timer_ = create_wall_timer(std::chrono::seconds(1), [this, t0]() {
        try {
          if (!scenario_) {
            load(readFile(resolvePath(autostart_file_)), variant_);
          }
          if (px4Ready(pursuer_) || now().seconds() - t0 > kAutostartWait) {
            start();
            autostart_timer_->cancel();
          }
        } catch (const std::exception & e) {
          transition(Status::ERROR, e.what());
          autostart_timer_->cancel();
        }
      });
  }
}

// -- comandos ------------------------------------------------------------------------

void ScenarioRunnerNode::onCommand(const Command::Request & req, Command::Response & res)
{
  try {
    const auto text = !req.yaml.empty() ? req.yaml :
      (!req.path.empty() ? readFile(resolvePath(req.path)) : std::string());
    if (req.command == "load") {
      if (text.empty()) {throw std::runtime_error("load needs 'yaml' or 'path'");}
      load(text, req.variant);
    } else if (req.command == "start") {
      if (!text.empty()) {load(text, req.variant);}
      start();
    } else if (req.command == "stop") {
      stop("interrompido");
    } else if (req.command == "pause") {
      if (state_ != Status::RUNNING) {throw std::runtime_error("nothing running");}
      paused_at_ = now().seconds();
      setPursuit(false);
      holdAll();
      transition(Status::PAUSED, "pausado");
    } else if (req.command == "resume") {
      if (state_ != Status::PAUSED) {throw std::runtime_error("not paused");}
      run_start_ += now().seconds() - paused_at_;
      if (engaged_) {setPursuit(true);}
      transition(Status::RUNNING, "retomado");
    } else if (req.command == "unload") {
      unload();
    } else {
      throw std::runtime_error("unknown command: " + req.command);
    }
    res.success = true;
    res.message = std::string(stateName(state_)) + (message_.empty() ? "" : ": " + message_);
  } catch (const std::exception & e) {
    res.success = false;
    res.message = e.what();
  }
}

std::string ScenarioRunnerNode::resolvePath(const std::string & path) const
{
  if (fs::path(path).is_absolute() || scenario_dir_.empty()) {
    return path;
  }
  return (fs::path(scenario_dir_) / path).string();
}

void ScenarioRunnerNode::load(const std::string & yaml_text, const std::string & variant)
{
  scenario::Scenario s = scenario::parseScenario(yaml_text);  // Throws before touching the state
  for (const auto & d : s.drones) {
    if (!catalog_.count(d.model)) {
      throw std::runtime_error("drone " + d.name + ": model '" + d.model + "' not in the catalog");
    }
  }
  if (state_ == Status::RUNNING || state_ == Status::STAGING || state_ == Status::PAUSED) {
    stop("replaced by " + s.name);
  }
  // Removes what this node spawned and the new scenario does not use.
  drainSpawnResults();
  std::set<std::string> wanted;
  for (const auto & d : s.drones) {
    wanted.insert(d.name);
  }
  for (auto it = spawned_.begin(); it != spawned_.end();) {
    if (!wanted.count(*it)) {
      gz_->remove(*it);
      it = spawned_.erase(it);
    } else {
      ++it;
    }
  }

  executor_config_.pursuer_frame = s.pursuer.frame;
  agents_.clear();
  for (const auto & d : s.drones) {
    Agent a;
    a.gains = default_gains_;
    a.gains.max_speed = d.max_speed;
    a.executor = std::make_unique<scenario::AgentExecutor>(d, executor_config_);
    agents_.emplace(d.name, std::move(a));
  }
  pursuer_plan_.reset();
  if (!s.pursuer.plan.empty()) {
    scenario::DroneSpec p;
    p.name = s.pursuer.frame;
    p.plan = s.pursuer.plan;
    pursuer_plan_ = std::make_unique<scenario::AgentExecutor>(p, executor_config_);
  }
  pursuer_.name = s.pursuer.frame;
  pursuer_.start = s.pursuer.start;
  live_speed_override_ = false;  // a live target_speed set before this scenario does not count

  variant_ = variant;
  scenario_ = std::move(s);
  std_msgs::msg::String source;
  source.data = yaml_text;
  pub_source_->publish(source);
  run_index_ = 0;
  spawnMissing();
  publishPaths();
  transition(Status::LOADED, "loaded: " + scenario_->name +
    " (" + std::to_string(agents_.size()) + " drones)");
}

void ScenarioRunnerNode::start()
{
  if (!scenario_) {
    throw std::runtime_error("no scenario loaded");
  }
  if (state_ == Status::RUNNING || state_ == Status::STAGING) {
    throw std::runtime_error("already running");
  }
  run_index_ = 1;
  const double t = now().seconds();
  staging_start_ = t;
  recovered_.clear();
  staged_since_ = -1.0;
  setPursuit(false);
  configurePerception(false);
  teleportToSpawn();
  applyParameters();
  transition(Status::STAGING, "run 1/" + std::to_string(scenario_->repeat) + ": staging");
}

void ScenarioRunnerNode::stop(const std::string & reason)
{
  if (!scenario_) {
    return;
  }
  setPursuit(false);
  configurePerception(false);
  holdAll();
  transition(Status::LOADED, reason);
}

void ScenarioRunnerNode::unload()
{
  stop("unloaded");
  drainSpawnResults();
  // Everything the scenario spawned: confirmed, in transit or present in the world without
  // belonging to the base world (e.g. spawned before a world reset).
  std::set<std::string> doomed = spawned_;
  for (const auto & [name, t] : spawn_pending_) {
    doomed.insert(name);
  }
  for (const auto & [name, agent] : agents_) {
    if (!preexisting_.count(name)) {
      doomed.insert(name);
    }
  }
  for (const auto & name : doomed) {
    gz_->remove(name);
  }
  RCLCPP_INFO(get_logger(), "Unloaded: %zu models removed", doomed.size());
  spawned_.clear();
  spawn_pending_.clear();
  agents_.clear();
  pursuer_plan_.reset();
  scenario_.reset();
  publishPaths();
  transition(Status::IDLE, "no scenario");
}

// -- loop ----------------------------------------------------------------------------

void ScenarioRunnerNode::updateStates()
{
  double stamp = 0.0;
  const auto poses = gz_->poses(&stamp);
  if (stamp + 0.5 < pose_stamp_) {
    world_.agents.clear();  // Mundo reiniciado
    state_stamp_.clear();
  }
  pose_stamp_ = stamp;
  for (const auto & [name, pose] : poses) {
    AgentState & s = world_.agents[name];
    double & last = state_stamp_[name];
    if (last > 0.0 && stamp > last) {
      const Eigen::Vector3d v = (pose.position - s.position) / (stamp - last);
      s.velocity = velocity_filter_ * v + (1.0 - velocity_filter_) * s.velocity;
    }
    if (stamp > last) {
      last = stamp;
    }
    s.position = pose.position;
    s.yaw = sim::yawOf(pose.orientation);
    s.attitude = pose.orientation.normalized();
  }
  for (auto & [name, agent] : agents_) {
    agent.present = poses.count(name) > 0;
    if (agent.present) {
      // Present without having been spawned by this node: base-world model (X3, X4,
      // Tiltrotor...) reused, kept on unload.
      if (!spawned_.count(name) && !spawn_pending_.count(name)) {
        preexisting_.insert(name);
      }
      spawn_pending_.erase(name);
    }
  }
}

const AgentState * ScenarioRunnerNode::state(const std::string & name) const
{
  return world_.find(name);
}

void ScenarioRunnerNode::tick()
{
  const double t = now().seconds();
  if (last_tick_ >= 0.0 && t < last_tick_ - 0.5) {
    // Simulation time went back (world reset): the current run loses its meaning.
    if (state_ == Status::RUNNING || state_ == Status::STAGING || state_ == Status::PAUSED) {
      stop("mundo reiniciado");
    }
    // spawned_ is kept: if the models vanished, spawnMissing spawns them again;
    // if they stayed, unload still removes them.
  }
  last_tick_ = t;
  drainSpawnResults();
  updateStates();
  const bool resend = t - last_enable_ >= kEnablePeriod;
  if (resend) {
    last_enable_ = t;
    for (auto it = base_enable_until_.begin(); it != base_enable_until_.end();) {
      gz_->enable(it->first, true);  // first messages may precede gz-transport discovery
      it = t > it->second ? base_enable_until_.erase(it) : std::next(it);
    }
  }
  if (scenario_ && resend) {
    enableTargets();
    if (state_ == Status::LOADED || state_ == Status::STAGING) {
      spawnMissing();
    }
  }
  liftBaseDrones(t);
  switch (state_) {
    case Status::STAGING: stepStaging(t); break;
    case Status::RUNNING: stepRunning(t); break;
    default:
      // Stopped: the zero command goes once on the transition (holdAll); repeating it here
      // would fight the joystick/training after the scenario ends.
      break;
  }
  if (publish_tf_) {
    publishTransforms();
  }
  if (t - last_status_ >= kStatusPeriod || t < last_status_) {
    publishStatus();
  }
}

void ScenarioRunnerNode::stepStaging(double now_s)
{
  const auto & st = scenario_->staging;
  bool ready = now_s - staging_start_ >= kMinStaging;
  std::vector<std::string> waiting;  // what staging still waits for (shown in the GCS)
  char buf[160];
  for (auto & [name, agent] : agents_) {
    const AgentState * self = state(name);
    if (!agent.present || !self) {
      ready = false;
      waiting.push_back(name + " to appear in the simulator");
      continue;
    }
    const auto target = agent.executor->startPoint();
    Reference ref;
    ref.has_position = true;
    ref.position = target.value_or(self->position);
    if (!target) {
      // Relative/reactive plan: only guarantees height for take-off.
      ref.position.z() = std::max(ref.position.z(), 2.0);
    }
    // A target lying on the ground while its start is in the air crashed in the previous run (a
    // flipped multicopter does not take off again): put it back at its start, upright.
    constexpr double kGrounded = 0.6, kRecoverAfter = 8.0;  // [m], [s] of staging
    if (target && self->position.z() < kGrounded && ref.position.z() > 2.0 * kGrounded &&
      now_s - staging_start_ > kRecoverAfter && !recovered_.count(name))
    {
      recovered_.insert(name);
      RCLCPP_WARN(get_logger(), "%s is on the ground (crashed?): moved back to its start", name.c_str());
      gz_->setPose(name, poseAt(ref.position, self->yaw));
      gz_->enable(name, true);
    }
    commandDrone(name, agent, ref);
    const double d = (ref.position - self->position).norm();
    if (d >= st.tolerance || self->velocity.norm() >= st.max_speed) {
      ready = false;
      std::snprintf(buf, sizeof(buf), "%s at its start (%.1f m away)", name.c_str(), d);
      waiting.push_back(buf);
    }
  }
  auto stage_px4 = [&](Px4Vehicle & v, const std::optional<Eigen::Vector3d> & start, double tol) {
      const AgentState * self = state(v.name);
      if (!start) {
        return;
      }
      // A pursuer lying on the ground (crashed, maybe upside down) cannot take off: put it back
      // upright where it is, once per staging; the offboard node then re-arms and takes off.
      constexpr double kGroundedPx4 = 0.6, kRecoverPx4After = 5.0;  // [m], [s] of staging
      if (self && self->position.z() < kGroundedPx4 && start->z() > 2.0 * kGroundedPx4 &&
        now_s - staging_start_ > kRecoverPx4After && !recovered_.count(v.name))
      {
        recovered_.insert(v.name);
        RCLCPP_WARN(get_logger(), "%s is on the ground (crashed?): put back upright", v.name.c_str());
        Eigen::Vector3d p = self->position;
        p.z() = 0.3;
        gz_->setPose(v.name, poseAt(p, self->yaw));
      }
      if (!self || !px4Ready(v)) {
        ready = false;
        waiting.push_back("the pursuer (PX4) to take off");
        return;
      }
      Reference ref;
      ref.has_position = true;
      ref.position = *start;
      ref.has_yaw = true;
      ref.yaw = scenario_->pursuer.start_yaw;
      const auto & ps = scenario_->pursuer;
      if (&v == &pursuer_ && ps.face_target) {
        // Camera facing the target: heading from the pursuer start to the target start; the
        // staging only ends aligned (automatic acquisition starts with the target in the frame).
        if (const auto yaw = yawToTarget(*start, true)) {
          ref.yaw = facingYaw(*yaw, self->yaw, now_s);
          const double err = std::abs(scenario::wrapAngle(self->yaw - *yaw));
          if (err >= ps.yaw_tolerance) {
            ready = false;
            std::snprintf(buf, sizeof(buf), "the pursuer heading (%.1f deg off, tolerance %.1f)",
              err * 180.0 / M_PI, ps.yaw_tolerance * 180.0 / M_PI);
            waiting.push_back(buf);
          }
        }
      }
      commandPx4(v, *self, ref, pursuer_max_speed_);
      const double d = (*start - self->position).norm();
      if (d >= tol || self->velocity.norm() >= st.max_speed) {
        ready = false;
        std::snprintf(buf, sizeof(buf), "the pursuer at its start (%.1f m away)", d);
        waiting.push_back(buf);
      }
    };
  stage_px4(pursuer_, pursuer_.start, st.pursuer_tolerance);
  // Gimbal (when there is one, both topics fresh): the measured pitch at the command.
  constexpr double kGimbalFresh = 1.0, kGimbalTolerance = 3.0 * M_PI / 180.0;  // [s], [rad]
  const auto t_ros = now();
  if (gimbal_cmd_ && gimbal_meas_ && (t_ros - gimbal_cmd_->second).seconds() < kGimbalFresh &&
    (t_ros - gimbal_meas_->second).seconds() < kGimbalFresh)
  {
    const double err = std::abs(gimbal_meas_->first - gimbal_cmd_->first);
    if (err >= kGimbalTolerance) {
      ready = false;
      std::snprintf(buf, sizeof(buf), "the gimbal at its command (pitch %.1f deg, command %.1f deg)",
        gimbal_meas_->first * 180.0 / M_PI, gimbal_cmd_->first * 180.0 / M_PI);
      waiting.push_back(buf);
    }
  }
  if (!scenario_->parameters.empty()) {
    verifyParameters(now_s);
    if (!params_verified_) {
      ready = false;
      waiting.push_back("the run's parameters (" + params_mismatch_ + ")");
    }
  }
  if (!waiting.empty()) {
    std::string text = "staging: waiting for ";
    for (std::size_t i = 0; i < waiting.size(); ++i) {
      text += (i ? ", " : "") + waiting[i];
    }
    std::snprintf(buf, sizeof(buf), " (%.0f/%.0f s)", now_s - staging_start_, st.timeout);
    message_ = text + buf;
  }

  if (ready && staged_since_ < 0.0) {
    staged_since_ = now_s;
  } else if (!ready) {
    staged_since_ = -1.0;
  }
  if (staged_since_ >= 0.0 && now_s - staged_since_ >= st.settle) {
    beginRun(now_s);
  } else if (now_s - staging_start_ > st.timeout && params_verified_) {
    // Positions may be off after the timeout, but never the parameters: without them the run
    // would not be the variant it is named after (it keeps staging until they match).
    RCLCPP_WARN(get_logger(), "Staging incomplete after %.0f s: starting anyway", st.timeout);
    beginRun(now_s);
    message_ = "staging incomplete (timeout)";
  }
}

double ScenarioRunnerNode::facingYaw(double desired, double actual, double now_s)
{
  const double dt = yaw_correction_t_ < 0.0 ? 0.0 : std::clamp(now_s - yaw_correction_t_, 0.0, 0.2);
  yaw_correction_t_ = now_s;
  const double err = scenario::wrapAngle(desired - actual);
  // Only integrates near the heading target (PX4 is already turning when the error is large).
  if (std::abs(err) < 0.35) {
    yaw_correction_ = std::clamp(yaw_correction_ + 0.6 * err * dt, -0.5, 0.5);
  }
  return scenario::wrapAngle(desired + yaw_correction_);
}

std::optional<double> ScenarioRunnerNode::yawToTarget(const Eigen::Vector3d & from, bool use_start) const
{
  const auto & target = scenario_->pursuer.target;
  std::optional<Eigen::Vector3d> p;
  if (use_start) {
    if (const auto it = agents_.find(target); it != agents_.end()) {
      p = it->second.executor->startPoint();
    }
  }
  if (!p) {
    if (const AgentState * s = state(target)) {
      p = s->position;
    }
  }
  if (!p) {
    return std::nullopt;
  }
  const Eigen::Vector3d d = *p - from;
  if (d.head<2>().norm() < 0.5) {
    return std::nullopt;  // target vertically above the pursuer: undefined heading
  }
  return std::atan2(d.y(), d.x());
}

void ScenarioRunnerNode::beginRun(double now_s)
{
  pursuer_hold_.reset();
  run_start_ = now_s;
  elapsed_ = 0.0;
  target_clock_ = 0.0;
  target_nominal_speed_ = 0.0;
  target_scale_ = 1.0;
  engaged_ = false;
  target_confirmed_ = false;
  for (auto & [name, agent] : agents_) {
    agent.executor->reset();
  }
  if (pursuer_plan_) {
    pursuer_plan_->reset();
  }
  configurePerception(true);
  transition(Status::RUNNING, "run " + std::to_string(run_index_) + "/" +
    std::to_string(scenario_->repeat) + " (" + runId() + ")");
}

void ScenarioRunnerNode::stepRunning(double now_s)
{
  // Measured step (simulation time): the timer does not fire at exactly rate_hz (~21 vs 20 Hz
  // seen with Isaac), and a fixed period made the target plans run that much faster.
  const double prev = elapsed_;
  elapsed_ = now_s - run_start_;
  const double dt = std::clamp(elapsed_ - prev, 0.0, 4.0 * period_);
  // Target plans on their own clock: speeding up changes how fast, not where (no jumps when the
  // scale changes in flight).
  double scale = targetSpeedScale();
  // Soft start: from rest the target speed rises at most at target_start_accel. A reference that
  // starts at full speed makes the target pitch to its limit and sink metres (slow vertical loop).
  if (start_accel_ > 0.0) {
    const double v_full = scale * target_nominal_speed_;
    if (v_full > 0.2) {  // a hover/hold (or the first tick, speed not known yet) keeps its clock
      scale *= std::min(1.0, elapsed_ * start_accel_ / v_full);
    }
  }
  target_clock_ += dt * scale;
  bool all_done = true;
  for (auto & [name, agent] : agents_) {
    const AgentState * self = state(name);
    if (!agent.present || !self) {
      continue;
    }
    auto cmd = agent.executor->update(target_clock_, dt * scale, *self, world_);
    if (name == scenario_->pursuer.target) {
      target_nominal_speed_ = cmd.reference.velocity.norm();  // plan speed (per unit of plan time)
    }
    cmd.reference.velocity *= scale;  // d/dt = scale · d/dτ
    all_done = all_done && agent.executor->finished();
    commandDrone(name, agent, cmd.reference, scale);
  }

  const auto & ps = scenario_->pursuer;
  if (!engaged_) {
    if (pursuer_plan_) {
      if (const AgentState * self = state(pursuer_.name); self && px4Ready(pursuer_)) {
        const auto cmd = pursuer_plan_->update(elapsed_, dt, *self, world_);
        commandPx4(pursuer_, *self, cmd.reference, pursuer_max_speed_);
      }
    } else if (ps.face_target) {
      // No patrol: holds position and follows the target in yaw until engaging (ROI in the frame).
      if (const AgentState * self = state(pursuer_.name); self && px4Ready(pursuer_)) {
        if (!pursuer_hold_) {
          pursuer_hold_ = self->position;
        }
        Reference ref;
        ref.has_position = true;
        ref.position = *pursuer_hold_;
        if (const auto yaw = yawToTarget(self->position, false)) {
          ref.has_yaw = true;
          ref.yaw = facingYaw(*yaw, self->yaw, now_s);
        }
        commandPx4(pursuer_, *self, ref, pursuer_max_speed_);
      }
    }
    if (elapsed_ >= ps.engage_after && (!ps.engage_on_detection || target_confirmed_ ||
      ps.detection == "omniscient"))
    {
      engaged_ = true;
      setPursuit(true);
      message_ = "pursuit enabled";
    }
  }

  // End: time, plans completed or capture.
  if (scenario_->duration > 0.0 && elapsed_ >= scenario_->duration) {
    endRun(now_s, "time up");
  } else if (scenario_->duration <= 0.0 && all_done && !agents_.empty()) {
    endRun(now_s, "plans finished");
  } else if (scenario_->capture_radius > 0.0) {
    const AgentState * p = state(pursuer_.name);
    const AgentState * tg = state(ps.target);
    if (p && tg && (p->position - tg->position).norm() < scenario_->capture_radius) {
      endRun(now_s, "capture");
    }
  }
}

void ScenarioRunnerNode::endRun(double now_s, const std::string & reason)
{
  setPursuit(false);
  configurePerception(false);
  RCLCPP_INFO(get_logger(), "Run %s ended: %s (%.1f s)", runId().c_str(), reason.c_str(), elapsed_);
  if (run_index_ < scenario_->repeat) {
    // Publishes the end of this run before switching the run_id (the recorder closes the bag).
    transition(Status::STAGING, reason);
    ++run_index_;
    staging_start_ = now_s;
    recovered_.clear();
    staged_since_ = -1.0;
    teleportToSpawn();
    applyParameters();
    message_ = reason + "; run " + std::to_string(run_index_) + "/" +
      std::to_string(scenario_->repeat) + ": staging";
    publishStatus();
  } else {
    holdAll();
    transition(Status::FINISHED, reason);
  }
}

void ScenarioRunnerNode::transition(uint8_t state, const std::string & message)
{
  if (state != state_) {
    RCLCPP_INFO(get_logger(), "Scenario: %s -> %s %s", stateName(state_), stateName(state), message.c_str());
  }
  state_ = state;
  message_ = message;
  publishStatus();
}

// -- actuation -------------------------------------------------------------------------

double ScenarioRunnerNode::targetSpeedScale()
{
  // Requested speed: the scenario field when it fixes one (unless the live parameter was moved
  // during the run), else the live parameter; 0 = the plan's own speeds.
  const double live = get_parameter("target_speed").as_double();
  const double fixed = scenario_ ? scenario_->target_speed : 0.0;
  double wanted = fixed > 0.0 && !(live_speed_override_ && live > 0.0) ? fixed : live;
  if (wanted <= 0.0) {
    target_scale_ = 1.0;
    return 1.0;
  }
  // The plan speed of the pursued drone at the last step (unscaled): the clock runs wanted/nominal
  // times faster so it flies at `wanted` m/s. Stopped (hover, a knot hold): keep the last scale.
  if (target_nominal_speed_ > 0.2) {
    target_scale_ = std::clamp(wanted / target_nominal_speed_, 0.05, 20.0);
  }
  return target_scale_;
}

void ScenarioRunnerNode::commandDrone(
  const std::string & name, Agent & agent, const Reference & ref, double speed_scale)
{
  const AgentState * self = state(name);
  if (!self) {
    return;
  }
  agent.reference = ref.has_position ? std::optional<Eigen::Vector3d>(ref.position) : std::nullopt;
  scenario::TrackerGains gains = agent.gains;
  gains.max_speed *= std::max(1.0, speed_scale);  // the scenario's cap scales with the plan
  gains.max_yaw_rate *= std::max(1.0, speed_scale);
  const auto cmd = scenario::trackReference(ref, *self, gains);
  // The multicopter velocity controller (gz-sim LeeVelocityController, and the Isaac copy) turns
  // the command into the world frame with the full body rotation R, not only the heading: sending
  // heading-frame velocities, a target pitched by θ to accelerate was commanded to sink at
  // v·sin θ (metres lost in every acceleration). R⁻¹·v makes it fly the world velocity asked.
  gz_->publishTwist(name, self->attitude.conjugate() * cmd.velocity, cmd.yaw_rate);
}

void ScenarioRunnerNode::commandPx4(
  Px4Vehicle & v, const AgentState & self, const Reference & ref, double max_speed)
{
  scenario::TrackerGains g = default_gains_;
  g.max_speed = max_speed;
  const auto cmd = scenario::trackReference(ref, self, g);
  TrajectorySetpoint sp;
  sp.position = {NAN, NAN, NAN};
  const Eigen::Vector3d v_ned = frames::enuToNed(cmd.velocity);
  sp.velocity = {static_cast<float>(v_ned.x()), static_cast<float>(v_ned.y()), static_cast<float>(v_ned.z())};
  sp.acceleration = {NAN, NAN, NAN};
  sp.jerk = {NAN, NAN, NAN};
  // Rumo ENU -> NED: psi_ned = pi/2 - psi_enu.
  sp.yaw = ref.has_yaw ? static_cast<float>(scenario::wrapAngle(M_PI_2 - ref.yaw)) : NAN;
  sp.yawspeed = NAN;
  sp.timestamp = static_cast<uint64_t>(now().nanoseconds() / 1000);
  v.pub->publish(sp);
}

bool ScenarioRunnerNode::px4Ready(const Px4Vehicle & v) const
{
  return v.offboard_state == "HOLD" || v.offboard_state == "EXTERNAL";
}

void ScenarioRunnerNode::holdAll()
{
  for (auto & [name, agent] : agents_) {
    if (agent.present) {
      gz_->publishTwist(name, Eigen::Vector3d::Zero(), 0.0);
    }
  }
}

void ScenarioRunnerNode::onSetDrones(const SetDrones::Request & req, SetDrones::Response & res)
{
  const auto poses = gz_->poses();
  const double t = now().seconds();
  std::vector<std::string> kept_by_scenario;
  for (const auto & d : base_drones_) {
    const bool wanted = std::find(req.present.begin(), req.present.end(), d.name) != req.present.end();
    const bool present = poses.count(d.name) > 0;
    if (wanted) {
      res.present.push_back(d.name);
      if (!present) {
        gz_->spawn(d.name, d.model, sim::withNamespace(catalog_.at(d.model).sdf, d.name), poseAt(d.spawn, 0.0));
        base_enable_until_[d.name] = t + 5.0;
        base_liftoff_until_[d.name] = t + 20.0;
        base_respawned_.insert(d.name);
        RCLCPP_INFO(get_logger(), "Base drone %s spawned", d.name.c_str());
      }
    } else if (present) {
      if (agents_.count(d.name)) {
        kept_by_scenario.push_back(d.name);  // the loaded scenario flies it
        res.present.push_back(d.name);
        continue;
      }
      gz_->remove(d.name);
      preexisting_.erase(d.name);
      base_respawned_.erase(d.name);
      RCLCPP_INFO(get_logger(), "Drone-base %s removido", d.name.c_str());
    }
  }
  res.success = true;
  res.message = kept_by_scenario.empty() ? "ok" : "kept (used by the scenario):";
  for (const auto & n : kept_by_scenario) {
    res.message += " " + n;
  }
}

void ScenarioRunnerNode::liftBaseDrones(double now_s)
{
  if (!base_liftoff_started_) {  // first tick: every base drone (spawned by the world launcher)
    base_liftoff_started_ = true;
    for (const auto & d : base_drones_) {
      base_liftoff_until_[d.name] = now_s + 30.0;
      base_enable_until_.emplace(d.name, now_s + 10.0);
    }
  }
  for (auto it = base_liftoff_until_.begin(); it != base_liftoff_until_.end();) {
    const auto & name = it->first;
    const auto base = std::find_if(base_drones_.begin(), base_drones_.end(),
        [&name](const BaseDrone & d) {return d.name == name;});
    const AgentState * self = state(name);
    if (now_s > it->second || agents_.count(name) || base == base_drones_.end()) {
      it = base_liftoff_until_.erase(it);  // timed out, or a scenario flies it now
      continue;
    }
    if (!self) {
      ++it;  // not spawned yet
      continue;
    }
    if (self->position.z() < base->spawn.z() - 0.1) {
      gz_->publishTwist(name, Eigen::Vector3d(0.0, 0.0, 0.8), 0.0);  // climb to the spawn height
      ++it;
    } else {
      gz_->publishTwist(name, Eigen::Vector3d::Zero(), 0.0);  // hover there
      it = base_liftoff_until_.erase(it);
    }
  }
}

void ScenarioRunnerNode::enableTargets()
{
  for (const auto & [name, agent] : agents_) {
    if (agent.present) {
      gz_->enable(name, true);
    }
  }
}

void ScenarioRunnerNode::drainSpawnResults()
{
  std::vector<std::pair<std::string, bool>> results;
  {
    std::lock_guard<std::mutex> lock(spawn_mutex_);
    results.swap(spawn_results_);
  }
  for (const auto & [name, ok] : results) {
    if (!ok) {
      RCLCPP_WARN(get_logger(), "Failed to spawn %s (retrying)", name.c_str());
      spawn_pending_.erase(name);
    } else if (agents_.count(name)) {
      spawned_.insert(name);
    } else {
      // Reply arrived after the unload/scenario switch: do not leave an orphan.
      spawn_pending_.erase(name);
      gz_->remove(name);
    }
  }
}

void ScenarioRunnerNode::spawnMissing()
{
  if (pose_stamp_ <= 0.0) {
    return;  // No poses yet: cannot know what already exists in the world
  }
  constexpr double kSpawnRetry = 15.0;
  const double t = now().seconds();
  int k = 0;
  for (auto & [name, agent] : agents_) {
    ++k;
    const auto pending = spawn_pending_.find(name);
    if (agent.present || (pending != spawn_pending_.end() && t - pending->second < kSpawnRetry)) {
      continue;
    }
    const auto & spec = agent.executor->spec();
    const auto & entry = catalog_.at(spec.model);
    const Eigen::Vector3d at = spec.spawn.value_or(
      agent.executor->startPoint().value_or(Eigen::Vector3d(-5.0, 2.5 * k, 1.0)));
    spawn_pending_[name] = t;
    gz_->spawn(name, spec.model, sim::withNamespace(entry.sdf, name), poseAt(at, 0.0),
      [this, name](bool ok) {
        std::lock_guard<std::mutex> lock(spawn_mutex_);
        spawn_results_.emplace_back(name, ok);
      });
  }
}

void ScenarioRunnerNode::teleportToSpawn()
{
  for (auto & [name, agent] : agents_) {
    if (const auto & spawn = agent.executor->spec().spawn; spawn && agent.present) {
      gz_->setPose(name, poseAt(*spawn, 0.0));
      gz_->publishTwist(name, Eigen::Vector3d::Zero(), 0.0);
    }
  }
}

void ScenarioRunnerNode::setPursuit(bool enabled)
{
  if (!pursuit_enable_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "pursuit_controller_node/enable unavailable");
    return;
  }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = enabled;
  pursuit_enable_->prune_pending_requests();
  pursuit_enable_->async_send_request(req);
}

void ScenarioRunnerNode::configurePerception(bool running)
{
  const auto & ps = scenario_->pursuer;
  msg::TargetTfCmd tf_cmd;
  tf_cmd.target_frame = ps.target;
  tf_cmd.enable_tf_tracking = running && ps.detection == "omniscient";
  pub_tf_cmd_->publish(tf_cmd);
  // Tracker cleared on every run; with automatic acquisition target_acquisition
  // reinitializes it on seeing RUNNING on /scenario/status.
  pub_selection_->publish(sensor_msgs::msg::RegionOfInterest());
}

void ScenarioRunnerNode::verifyParameters(double now_s)
{
  constexpr double kPeriod = 1.0;  // one round per second while staging
  if (params_checks_pending_ > 0 || now_s - params_check_t_ < kPeriod) {
    return;
  }
  params_check_t_ = now_s;
  params_round_ok_ = true;
  params_round_text_.clear();
  auto finish_node = [this]() {
      if (--params_checks_pending_ == 0) {
        params_verified_ = params_round_ok_;
        params_mismatch_ = params_round_text_;
      }
    };
  for (const auto & [node_name, values] : scenario_->parameters) {
    auto & client = param_clients_[node_name];
    if (!client) {
      client = std::make_shared<rclcpp::AsyncParametersClient>(this, node_name);
    }
    if (!client->service_is_ready()) {
      // This round failed, also for the nodes that answer (finish_node copies params_round_ok_).
      params_round_ok_ = false;
      params_round_text_ = node_name + " not available yet";
      params_verified_ = false;
      params_mismatch_ = params_round_text_;
      continue;  // the next round asks again
    }
    std::vector<std::string> names;
    for (const auto & kv : values) {
      names.push_back(kv.first);
    }
    ++params_checks_pending_;
    client->get_parameters(names,
      [this, node_name, values, client, finish_node](std::shared_future<std::vector<rclcpp::Parameter>> f) {
        std::vector<rclcpp::Parameter> fix;
        try {
          for (const auto & current : f.get()) {
            if (current.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
              continue;  // not declared by the node: nothing to compare (warned when applied)
            }
            const auto wanted = toParameter(current.get_name(), current.get_type(),
              values.at(current.get_name()));
            if (wanted && !(wanted->get_parameter_value() == current.get_parameter_value())) {
              if (params_round_ok_) {
                params_round_text_ = node_name + " " + current.get_name() + " = " +
                  current.value_to_string() + " (scenario: " + wanted->value_to_string() + ")";
              }
              params_round_ok_ = false;
              fix.push_back(*wanted);
            }
          }
        } catch (const std::exception & e) {
          params_round_ok_ = false;
          params_round_text_ = node_name + ": " + e.what();
        }
        if (!fix.empty()) {
          RCLCPP_WARN(get_logger(), "%s: %zu parameter(s) differ from the scenario (%s); set again",
            node_name.c_str(), fix.size(), params_round_text_.c_str());
          client->set_parameters(fix);
        }
        finish_node();
      });
  }
}

void ScenarioRunnerNode::applyParameters(const std::vector<std::string> & only)
{
  if (only.empty()) {
    param_retry_count_ = 0;
    param_retry_timer_.reset();
    // A new run: nothing verified yet (staging waits for a matching round).
    params_verified_ = scenario_->parameters.empty();
    params_mismatch_ = "not checked yet";
    params_check_t_ = -1e9;
  }
  std::vector<std::string> pending;
  for (const auto & [node_name, values] : scenario_->parameters) {
    if (!only.empty() && std::find(only.begin(), only.end(), node_name) == only.end()) {
      continue;
    }
    auto & client = param_clients_[node_name];
    if (!client) {
      client = std::make_shared<rclcpp::AsyncParametersClient>(this, node_name);
    }
    if (!client->service_is_ready()) {
      pending.push_back(node_name);
      continue;
    }
    std::vector<std::string> names;
    for (const auto & kv : values) {
      names.push_back(kv.first);
    }
    // Queries the declared types and sends each value converted to them.
    client->get_parameters(names,
      [this, node_name, values, client](std::shared_future<std::vector<rclcpp::Parameter>> f) {
        std::vector<rclcpp::Parameter> out;
        for (const auto & declared : f.get()) {
          const auto converted = toParameter(declared.get_name(), declared.get_type(),
            values.at(declared.get_name()));
          if (converted) {
            out.push_back(*converted);
          } else {
            RCLCPP_WARN(get_logger(), "%s: value of '%s' does not match the declared type",
              node_name.c_str(), declared.get_name().c_str());
          }
        }
        client->set_parameters(out,
          [this, node_name](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> r) {
            for (const auto & res : r.get()) {
              if (!res.successful) {
                RCLCPP_WARN(get_logger(), "%s refused a parameter: %s", node_name.c_str(), res.reason.c_str());
              }
            }
          });
      });
  }
  if (pending.empty()) {
    param_retry_timer_.reset();
    return;
  }
  constexpr int kMaxRetries = 30;  // 15 s
  if (++param_retry_count_ > kMaxRetries) {
    param_retry_timer_.reset();
    for (const auto & n : pending) {
      RCLCPP_WARN(get_logger(), "Parameters of %s not applied: node unavailable", n.c_str());
      message_ = n + " unavailable (parameters not applied)";
    }
    return;
  }
  param_retry_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this, pending]() {
      param_retry_timer_->cancel();
      if (scenario_) {
        applyParameters(pending);
      }
    });
}

// -- outputs --------------------------------------------------------------------------

std::string ScenarioRunnerNode::runId() const
{
  if (!scenario_) {
    return "";
  }
  return scenario_->name + (variant_.empty() ? "" : "_" + variant_) + "_r" + std::to_string(run_index_);
}

void ScenarioRunnerNode::publishStatus()
{
  last_status_ = now().seconds();
  Status s;
  s.header.stamp = now();
  s.header.frame_id = world_frame_;
  s.state = state_;
  s.state_name = stateName(state_);
  s.message = message_;
  s.variant = variant_;
  if (scenario_) {
    s.scenario = scenario_->name;
    s.run_id = runId();
    s.run_index = static_cast<uint32_t>(run_index_);
    s.repeat = static_cast<uint32_t>(scenario_->repeat);
    s.elapsed = state_ == Status::RUNNING || state_ == Status::PAUSED ?
      (state_ == Status::PAUSED ? paused_at_ - run_start_ : elapsed_) : 0.0;
    s.duration = scenario_->duration;
    s.engaged = engaged_;
    s.pursuer_target = scenario_->pursuer.target;
    s.detection = scenario_->pursuer.detection;
    s.acquisition = scenario_->pursuer.acquisition;
    s.reacquire = scenario_->pursuer.reacquire;
    auto add = [&](const std::string & name, const std::string & model, const std::string & role,
      double radius) -> msg::ScenarioDrone & {
        msg::ScenarioDrone d;
        d.name = name;
        d.model = model;
        d.role = role;
        d.radius = static_cast<float>(radius);
        d.reference.x = d.reference.y = d.reference.z = NAN;
        if (const AgentState * st = state(name)) {
          d.position = toPoint(st->position);
          d.velocity.x = st->velocity.x();
          d.velocity.y = st->velocity.y();
          d.velocity.z = st->velocity.z();
        }
        s.drones.push_back(d);
        return s.drones.back();
      };
    add(scenario_->pursuer.frame, "x500_depth", "pursuer", 0.5);
    for (const auto & [name, agent] : agents_) {
      const auto & spec = agent.executor->spec();
      auto & d = add(name, spec.model, "target", catalog_.at(spec.model).radius);
      d.leg = agent.executor->legLabel();
      d.legs_started = static_cast<uint32_t>(agent.executor->legsStarted());
      d.leg_iteration = static_cast<uint32_t>(agent.executor->legIteration());
      d.in_transit = agent.executor->inTransit();
      d.finished = agent.executor->finished();
      if (agent.reference) {
        d.reference = toPoint(*agent.reference);
      }
    }
  }
  pub_status_->publish(s);
}

void ScenarioRunnerNode::publishPaths()
{
  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(clear);
  if (scenario_) {
    const double horizon = scenario::previewHorizon(*scenario_);
    int id = 0;
    for (const auto & [name, agent] : agents_) {
      const auto & spec = agent.executor->spec();
      const Eigen::Vector3d origin = agent.executor->startPoint().value_or(
        spec.spawn.value_or(Eigen::Vector3d(0.0, 0.0, 5.0)));
      const auto preview = scenario::previewPlan(spec, executor_config_, origin, horizon, 0.2);
      for (const auto & path : preview.paths) {
        if (path.size() < 2) {
          continue;
        }
        visualization_msgs::msg::Marker m;
        m.header.frame_id = world_frame_;
        m.ns = "scenario/" + name;
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.scale.x = 0.12;
        m.color.a = 0.8f;
        m.color.r = 1.0f;
        m.color.g = 0.3f;
        m.color.b = 0.3f;
        m.pose.orientation.w = 1.0;
        for (const auto & p : path) {
          m.points.push_back(toPoint(p));
        }
        arr.markers.push_back(m);
      }
    }
  }
  pub_paths_->publish(arr);
}

void ScenarioRunnerNode::publishTransforms()
{
  if (!publish_model_tf_) {
    return;
  }
  std::vector<geometry_msgs::msg::TransformStamped> out;
  auto add = [&](const std::string & name) {
      const auto pose = gz_->pose(name);
      if (!pose) {
        return;
      }
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = rclcpp::Time(static_cast<int64_t>(pose_stamp_ * 1e9), RCL_ROS_TIME);
      tf.header.frame_id = world_frame_;
      tf.child_frame_id = name;
      tf.transform.translation.x = pose->position.x();
      tf.transform.translation.y = pose->position.y();
      tf.transform.translation.z = pose->position.z();
      tf.transform.rotation.w = pose->orientation.w();
      tf.transform.rotation.x = pose->orientation.x();
      tf.transform.rotation.y = pose->orientation.y();
      tf.transform.rotation.z = pose->orientation.z();
      out.push_back(tf);
    };
  // Only what has no PosePublisher TF: spawned models.
  for (const auto & name : spawned_) {
    add(name);
  }
  for (const auto & name : base_respawned_) {
    if (!spawned_.count(name)) {
      add(name);
    }
  }
  if (!out.empty()) {
    tf_broadcaster_->sendTransform(out);
  }
}

}  // namespace aero_pursuit

RCLCPP_COMPONENTS_REGISTER_NODE(aero_pursuit::ScenarioRunnerNode)
