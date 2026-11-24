#include "aero_pursuit/nodes/pursuit_controller_node.hpp"

#include "aero_pursuit/common/frames.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>

namespace aero_pursuit
{

using control::PursuitMode;

namespace
{
double wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

}  // namespace

PursuitControllerNode::PursuitControllerNode(const rclcpp::NodeOptions & options)
// `mpc.params.<name>` are free-form numbers created from the GCS (not declared in advance).
: Node("pursuit_controller_node", rclcpp::NodeOptions(options).allow_undeclared_parameters(true)),
  v_measured_stamp_(0, 0, get_clock()->get_clock_type()),
  odometry_stamp_(0, 0, get_clock()->get_clock_type())
{
  const auto input_topic = declare_parameter<std::string>("input_topic", "/target_3d_state_filtered");
  const auto bearing_topic = declare_parameter<std::string>("bearing_topic", "/target_bearing_enu");
  const auto accel_topic =
    declare_parameter<std::string>("target_accel_topic", "/target_3d_accel_filtered");
  const auto output_topic = declare_parameter<std::string>("output_topic", "/offboard/trajectory_setpoint");
  const auto local_position_topic =
    declare_parameter<std::string>("local_position_topic", "/fmu/out/vehicle_local_position_v1");
  const auto odometry_topic =
    declare_parameter<std::string>("odometry_topic", "/fmu/out/vehicle_odometry");
  const auto path_topic = declare_parameter<std::string>("predicted_path_topic", "/pursuit/predicted_path");
  const auto status_topic = declare_parameter<std::string>("status_topic", "/pursuit/status");
  const auto metrics_topic = declare_parameter<std::string>("metrics_topic", "/pursuit/metrics");

  rcl_interfaces::msg::ParameterDescriptor read_only;
  read_only.read_only = true;
  dt_ = declare_parameter<double>("dt", 0.05, read_only);
  enabled_ = declare_parameter<bool>("start_enabled", true);
  declareControllerParameters();
  loadRuntimeParameters();
  controller_ = std::make_unique<control::Mpc>();
  {
    // `mpc.params.*` of the YAML / launch overrides: declared (so they are listed and the GCS
    // shows them; dynamic typing so the GCS can also remove them) and given to the MPC.
    rcl_interfaces::msg::ParameterDescriptor free_form;
    free_form.dynamic_typing = true;
    std::vector<rclcpp::Parameter> initial;
    for (const auto & p : get_node_options().parameter_overrides()) {
      if (p.get_name().rfind("mpc.params.", 0) == 0) {
        if (!has_parameter(p.get_name())) {
          declare_parameter(p.get_name(), p.get_parameter_value(), free_form);
        }
        initial.push_back(get_parameter(p.get_name()));
      }
    }
    applyMpcParameters(initial);
  }

  sub_target_ = create_subscription<nav_msgs::msg::Odometry>(
    input_topic, 10, [this](const nav_msgs::msg::Odometry & msg) {onTarget(msg);});
  sub_target_accel_ = create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
    accel_topic, 10, [this](const geometry_msgs::msg::AccelWithCovarianceStamped & msg) {
      const auto & a = msg.accel.accel.linear;
      target_accel_ = std::make_pair(Eigen::Vector3d(a.x, a.y, a.z),
      rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type()));
    });
  sub_bearing_ = create_subscription<msg::TargetBearing>(
    bearing_topic, 10, [this](const msg::TargetBearing & msg) {onBearing(msg);});
  sub_local_position_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    local_position_topic, rclcpp::SensorDataQoS(),
    [this](const px4_msgs::msg::VehicleLocalPosition & msg) {onLocalPosition(msg);});
  sub_odometry_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
    odometry_topic, rclcpp::SensorDataQoS(),
    [this](const px4_msgs::msg::VehicleOdometry & msg) {onOdometry(msg);});
  sub_gimbal_ = create_subscription<sensor_msgs::msg::JointState>(
    "/gimbal/joint_states", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::JointState & m) {
      for (size_t i = 0; i < m.name.size() && i < m.position.size(); ++i) {
        if (m.name[i].find("gimbal_pitch_joint") != std::string::npos) {
          gimbal_pitch_ = m.position[i];
        }
      }
    });
  sub_tf_static_ = create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(100).reliable().transient_local(),
    [this](const tf2_msgs::msg::TFMessage & msg) {onStaticTf(msg);});

  pub_setpoint_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(output_topic, 10);
  pub_path_ = create_publisher<nav_msgs::msg::Path>(path_topic, 10);
  pub_status_ = create_publisher<msg::PursuitStatus>(status_topic, 10);
  pub_metrics_ = create_publisher<msg::PursuitMetrics>(metrics_topic, 10);

  srv_enable_ = create_service<std_srvs::srv::SetBool>(
    "~/enable",
    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
      enabled_ = req->data;
      res->success = true;
      res->message = enabled_ ? "Pursuit enabled" : "Pursuit disabled";
      RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
    });

  validate_param_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {return validateParameters(params);});
  post_param_handle_ = add_post_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {onParametersSet(params);});

  // Timer on the node clock (simulation time when use_sim_time=true).
  timer_ = create_timer(std::chrono::duration<double>(dt_), [this]() {controlLoop();});

  RCLCPP_INFO(get_logger(), "Pursuit controller: '%s' (dt=%.3f s, d=%.1f m)",
    controller_->name().c_str(), dt_, standoff_);
}


void PursuitControllerNode::onTarget(const nav_msgs::msg::Odometry & msg)
{
  const auto & p = msg.pose.pose.position;
  const auto & v = msg.twist.twist.linear;
  target_ = TargetEstimate{
    Eigen::Vector3d(p.x, p.y, p.z), Eigen::Vector3d(v.x, v.y, v.z),
    rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type())};
  frame_id_ = msg.header.frame_id;
}

void PursuitControllerNode::onBearing(const msg::TargetBearing & msg)
{
  bearing_ = BearingEstimate{
    Eigen::Vector3d(msg.direction.x, msg.direction.y, msg.direction.z),
    control::offAxisAngle(msg.azimuth, msg.elevation),
    rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type()), msg.angular_size,
    msg.range_lower_bound > 0.0f ? static_cast<double>(msg.range_lower_bound) : NAN};
  if (frame_id_.empty()) {
    frame_id_ = msg.header.frame_id;
  }
}

void PursuitControllerNode::onStaticTf(const tf2_msgs::msg::TFMessage & msg)
{
  for (const auto & tf : msg.transforms) {
    const auto & q = tf.transform.rotation;
    const auto & t = tf.transform.translation;
    static_tf_.set(tf.header.frame_id, tf.child_frame_id, Eigen::Quaterniond(q.w, q.x, q.y, q.z),
      Eigen::Vector3d(t.x, t.y, t.z));
  }
}

void PursuitControllerNode::onLocalPosition(const px4_msgs::msg::VehicleLocalPosition & msg)
{
  if (!msg.v_xy_valid || !msg.v_z_valid) {
    return;
  }
  v_measured_ = frames::nedToEnu(Eigen::Vector3d(msg.vx, msg.vy, msg.vz));
  v_measured_stamp_ = get_clock()->now();
  altitude_ = msg.z_valid ? -static_cast<double>(msg.z) : NAN;
  heading_ = std::isfinite(msg.heading) ? static_cast<double>(msg.heading) : NAN;
}

void PursuitControllerNode::onOdometry(const px4_msgs::msg::VehicleOdometry & msg)
{
  using Odom = px4_msgs::msg::VehicleOdometry;
  if (msg.pose_frame != Odom::POSE_FRAME_NED || msg.velocity_frame != Odom::VELOCITY_FRAME_NED ||
    !std::isfinite(msg.q[0]))
  {
    return;
  }
  const Eigen::Vector3d v(msg.velocity[0], msg.velocity[1], msg.velocity[2]);
  const Eigen::Vector3d w(msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]);
  if (!v.allFinite() || !w.allFinite()) {
    return;
  }
  odometry_.attitude = frames::px4AttitudeToEnuFlu(msg.q);
  odometry_.body_rates = frames::frdToFlu(w);
  odometry_.velocity = frames::nedToEnu(v);
  odometry_stamp_ = get_clock()->now();
}

std::optional<control::VehicleState> PursuitControllerNode::measuredVehicleState() const
{
  if (odometry_stamp_.nanoseconds() == 0 ||
    (get_clock()->now() - odometry_stamp_).seconds() > velocity_timeout_)
  {
    return std::nullopt;
  }
  return odometry_;
}

std::optional<Eigen::Vector3d> PursuitControllerNode::measuredChaserVelocity() const
{
  if (v_measured_stamp_.nanoseconds() == 0 ||
    (get_clock()->now() - v_measured_stamp_).seconds() > velocity_timeout_)
  {
    return std::nullopt;
  }
  return v_measured_;
}

void PursuitControllerNode::controlLoop()
{
  if (runtime_pending_) {
    loadRuntimeParameters();
    RCLCPP_INFO(get_logger(), "Parameters applied: v_max=%.2f m/s", limits_.max_velocity);
  }

  const rclcpp::Time now = get_clock()->now();
  // Simulation time went back (Gazebo reset): stamps "from the future" would have
  // age 0 forever and the old estimate would be treated as recent.
  const rclcpp::Time zero(0, 0, now.get_clock_type());
  auto fromFuture = [&now](const rclcpp::Time & t) {return (t - now).seconds() > 0.5;};
  if (target_ && fromFuture(target_->stamp)) {
    RCLCPP_WARN(get_logger(), "Clock jumped back: dropping old estimates.");
    target_.reset();
  }
  if (bearing_ && fromFuture(bearing_->stamp)) {
    bearing_.reset();
  }
  if (fromFuture(v_measured_stamp_)) {
    v_measured_stamp_ = zero;
  }
  if (fromFuture(odometry_stamp_)) {
    odometry_stamp_ = zero;
  }

  std::optional<double> age;
  if (target_) {
    age = std::max(0.0, (now - target_->stamp).seconds());
  }
  std::optional<double> bearing_age;
  if (bearing_) {
    bearing_age = std::max(0.0, (now - bearing_->stamp).seconds());
  }

  supervisor_.update(enabled_, age, bearing_age);
  const auto mode = supervisor_.mode();
  if (supervisor_.changed()) {
    if (mode == PursuitMode::kLost) {
      RCLCPP_WARN(get_logger(), "Target lost for %.2f s: pursuit off.", age.value_or(0.0));
    } else if (mode == PursuitMode::kApproach) {
      RCLCPP_INFO(get_logger(), "No depth: only the bounding box (APPROACH); the MPC decides the action.");
    } else {
      RCLCPP_INFO(get_logger(), "Pursuit mode: %s", control::toString(mode));
    }
  }

  if (!supervisor_.active()) {
    publishStatus(now, age, bearing_age);
    return;
  }

  const auto v_meas = measuredChaserVelocity();
  if (supervisor_.engaged()) {
    v_ref_ = v_meas.value_or(Eigen::Vector3d::Zero());  // Avoids a jump in the reference
    // Heading kept while the law does not command yaw (ControlCommand::yaw empty).
    hold_yaw_ = std::isfinite(heading_) ? wrapAngle(M_PI_2 - heading_) : NAN;
  }

  // (Re)initializes the law's memories on (re)entering TRACKING from outside the 3D loop.
  if (supervisor_.changed() && mode == PursuitMode::kTracking &&
    supervisor_.previous() != PursuitMode::kCoasting)
  {
    controller_->reset();
    metrics_.reset();
  }
  runController(mode, age, v_meas);
  for (const auto & m : controller_->takeMessages()) {
    RCLCPP_INFO(get_logger(), "%s", m.c_str());  // e.g. MPC solver compiled / switched
  }
  publishStatus(now, age, bearing_age);
}

void PursuitControllerNode::runController(
  PursuitMode mode, std::optional<double> age, const std::optional<Eigen::Vector3d> & v_meas)
{
  // >>> Input of the pursuit law: everything the MPC sees is built here (PursuitState). <<<
  control::PursuitState state;
  const bool has_position = mode != PursuitMode::kApproach && target_.has_value();
  state.has_position = has_position;
  state.standoff = standoff_;
  state.limits = limits_;
  state.v_chaser = v_ref_;
  state.vehicle = measuredVehicleState();
  if (std::isfinite(heading_)) {
    state.heading = wrapAngle(M_PI_2 - heading_);  // NED heading -> ENU yaw
  }
  const rclcpp::Time t_now = get_clock()->now();
  if (bearing_ && (t_now - bearing_->stamp).seconds() <= bearing_timeout_) {
    state.box = control::TargetBox{bearing_->direction, bearing_->off_axis, bearing_->angular_size,
      bearing_->range_lower_bound, std::max(0.0, (t_now - bearing_->stamp).seconds())};
  }
  if (has_position) {
    // Extrapolates the estimate up to now (essential during COASTING).
    state.p_rel = target_->p_rel + target_->v_rel * age.value_or(0.0);
    state.v_rel = target_->v_rel;
    state.v_target = v_meas.value_or(v_ref_) + target_->v_rel;
    if (accel_cfg_.enabled && target_accel_ &&
      std::abs((t_now - target_accel_->second).seconds()) <= accel_cfg_.timeout)
    {
      Eigen::Vector3d a = target_accel_->first;
      if (a.norm() > accel_cfg_.max) {
        a *= accel_cfg_.max / a.norm();
      }
      state.a_target = a;
      state.accel_tau = accel_cfg_.tau;
      // Extrapolation over the estimate age: the acceleration part of the target motion.
      const double t_age = age.value_or(0.0);
      const Eigen::Vector3d dp = state.targetDisplacement(t_age) - state.v_target * t_age;
      const Eigen::Vector3d dv = state.targetVelocityAt(t_age) - state.v_target;
      state.p_rel += dp;
      state.v_rel += dv;
      state.v_target += dv;
    }
  }

  const auto t0 = std::chrono::steady_clock::now();
  auto raw = controller_->compute(state, dt_);  // >>> Mpc::compute (control/mpc.cpp) <<<
  const double solve_us =
    std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
  // Safety only: |velocity| <= max_vel (the law is responsible for everything else).
  auto cmd = control::limitVelocity(raw, limits_);
  if (max_acc_xy_ > 0.0) {
    const double a_xy = cmd.acceleration.head<2>().norm();
    if (a_xy > max_acc_xy_) {
      cmd.acceleration.head<2>() *= max_acc_xy_ / a_xy;
    }
  }
  if (min_altitude_ > 0.0 && std::isfinite(altitude_)) {
    // Safety floor: descent rate limited to 1 /s x the height above the floor (climbs back below it).
    constexpr double kFloorGain = 1.0;  // [1/s]
    const double vz_min = -kFloorGain * (altitude_ - min_altitude_);
    if (cmd.velocity.z() < vz_min) {
      cmd.velocity.z() = vz_min;
      cmd.acceleration.z() = std::max(cmd.acceleration.z(), 0.0);
      cmd.jerk.z() = std::numeric_limits<double>::quiet_NaN();
    }
  }
  v_ref_ = cmd.velocity;
  if (cmd.yaw) {
    hold_yaw_ = *cmd.yaw;  // a later cycle without yaw (e.g. yaw_mode 0) holds where the law left it
  }

  const rclcpp::Time now = get_clock()->now();
  publishSetpoint(cmd);  // TrajectorySetpoint: velocity + acceleration feedforward + heading
  publishPath(cmd, now);
  publishMetrics(now, state, cmd, solve_us);
}

double PursuitControllerNode::losError(const Eigen::Vector3d & p_rel) const
{
  if (!std::isfinite(heading_) || p_rel.head<2>().norm() < 1e-3) {
    return NAN;
  }
  return std::abs(wrapAngle(heading_ - frames::enuDirectionToNedYaw(p_rel)));
}

std::pair<double, double> PursuitControllerNode::imageFeatures(const control::PursuitState & state) const
{
  // Measured attitude x gimbal mount (static TF base -> gimbal_mount) x measured gimbal pitch:
  // where the estimate falls in the image. NaN without odometry or TF.
  if (!state.vehicle || !state.has_position || state.p_rel.norm() < 1e-3) {
    return {NAN, NAN};
  }
  const auto base = get_parameter("camera_base_frame").as_string();
  const auto camera = get_parameter("camera_frame").as_string();
  const auto mount = static_tf_.rotation(base, camera);
  if (!mount) {
    return {NAN, NAN};
  }
  const Eigen::Matrix3d R_body = state.vehicle->attitude.toRotationMatrix();
  const auto T = static_tf_.transform(base, camera);
  const Eigen::Vector3d rel = T ? Eigen::Vector3d(state.p_rel - R_body * T->translation()) : state.p_rel;
  const Eigen::Matrix3d R_gimbal =
    Eigen::AngleAxisd(-gimbal_pitch_, Eigen::Vector3d::UnitY()).toRotationMatrix();  // + = up
  const Eigen::Vector3d c = (R_body * mount->toRotationMatrix() * R_gimbal).transpose() * rel;
  return {std::atan2(c.y(), c.x()), std::atan2(c.z(), std::hypot(c.x(), c.y()))};
}

void PursuitControllerNode::publishMetrics(
  const rclcpp::Time & now, const control::PursuitState & state,
  const control::ControlCommand & cmd, double solve_time_us)
{
  if (!state.has_position) {
    return;  // no depth: nothing to measure against the 3D estimate
  }
  const Eigen::Vector3d e_p = control::standoffPoint(state.p_rel, standoff_);
  const Eigen::Vector3d e_v = state.v_chaser - state.v_target;
  const double los = losError(state.p_rel);
  metrics_.update(e_p, e_v, los, cmd.acceleration, dt_);
  const auto image = imageFeatures(state);
  metrics_.updateImage(image.first, image.second, dt_);

  if (pub_metrics_->get_subscription_count() == 0) {
    return;
  }
  // Tilt the commanded acceleration needs: thrust along a + g e_z (ENU).
  const Eigen::Vector3d f = cmd.acceleration + Eigen::Vector3d(0.0, 0.0, 9.81);
  const double tilt = std::acos(std::clamp(f.z() / std::max(f.norm(), 1e-6), -1.0, 1.0));
  const bool bearing_fresh = bearing_ && (now - bearing_->stamp).seconds() <= 0.5;

  msg::PursuitMetrics m;
  m.header.stamp = now;
  m.header.frame_id = frame_id_;
  m.controller = controller_->name();
  m.mode = static_cast<uint8_t>(supervisor_.mode());
  m.position_error = static_cast<float>(e_p.norm());
  m.velocity_error = static_cast<float>(e_v.norm());
  m.range_error = static_cast<float>(state.p_rel.norm() - standoff_);
  m.los_error = static_cast<float>(los);
  m.camera_off_axis = bearing_fresh ? static_cast<float>(bearing_->off_axis) : NAN;
  m.image_azimuth = static_cast<float>(image.first);
  m.image_elevation = static_cast<float>(image.second);
  m.acceleration = static_cast<float>(cmd.acceleration.norm());
  m.deadband_factor = NAN;
  m.thrust_ratio = NAN;  // needs the vehicle model (not used: the PX4 closes the inner loops)
  m.tilt = static_cast<float>(tilt);
  m.solve_time_us = static_cast<float>(solve_time_us);
  m.rmse_position = static_cast<float>(metrics_.rmsePosition());
  m.rmse_velocity = static_cast<float>(metrics_.rmseVelocity());
  m.rmse_los = static_cast<float>(metrics_.rmseLos());
  m.rmse_image = static_cast<float>(metrics_.rmseImage());
  m.max_position_error = static_cast<float>(metrics_.maxPositionError());
  m.effort_l2 = static_cast<float>(metrics_.effortL2());
  m.effort_l1 = static_cast<float>(metrics_.effortL1());
  m.engaged_time = static_cast<float>(metrics_.duration());
  pub_metrics_->publish(m);
}

double PursuitControllerNode::yawEnu(const control::ControlCommand & cmd) const
{
  if (cmd.yaw) {
    return *cmd.yaw;
  }
  if (std::isfinite(hold_yaw_)) {
    return hold_yaw_;
  }
  return std::isfinite(heading_) ? wrapAngle(M_PI_2 - heading_) : 0.0;
}

void PursuitControllerNode::publishSetpoint(const control::ControlCommand & cmd)
{
  px4_msgs::msg::TrajectorySetpoint sp;
  sp.timestamp = static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  sp.position = {NAN, NAN, NAN};

  const Eigen::Vector3d v = frames::enuToNed(cmd.velocity);
  const Eigen::Vector3d a = frames::enuToNed(cmd.acceleration);
  sp.velocity = {static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z())};
  sp.acceleration = {static_cast<float>(a.x()), static_cast<float>(a.y()), static_cast<float>(a.z())};
  const Eigen::Vector3d j = frames::enuToNed(cmd.jerk);  // NaN => not used by PX4
  sp.jerk = {static_cast<float>(j.x()), static_cast<float>(j.y()), static_cast<float>(j.z())};

  // Heading: the law's (ENU -> NED, planned rate as the PX4 feedforward) or the one held since
  // the engagement.
  sp.yaw = static_cast<float>(wrapAngle(M_PI_2 - yawEnu(cmd)));
  sp.yawspeed = cmd.yaw ? static_cast<float>(-cmd.yaw_rate) : NAN;
  pub_setpoint_->publish(sp);
}

void PursuitControllerNode::publishPath(const control::ControlCommand & cmd, const rclcpp::Time & stamp)
{
  if (cmd.predicted_path.empty() || pub_path_->get_subscription_count() == 0) {
    return;
  }
  nav_msgs::msg::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = frame_id_;
  path.poses.resize(cmd.predicted_path.size());
  for (size_t i = 0; i < cmd.predicted_path.size(); ++i) {
    auto & pose = path.poses[i];
    pose.header = path.header;
    pose.pose.position.x = cmd.predicted_path[i].x();
    pose.pose.position.y = cmd.predicted_path[i].y();
    pose.pose.position.z = cmd.predicted_path[i].z();
    pose.pose.orientation.w = 1.0;
  }
  pub_path_->publish(path);
}

void PursuitControllerNode::publishStatus(
  const rclcpp::Time & now, std::optional<double> age, std::optional<double> bearing_age)
{
  msg::PursuitStatus status;
  status.header.stamp = now;
  status.header.frame_id = frame_id_;
  status.mode = static_cast<uint8_t>(supervisor_.mode());
  status.mode_name = control::toString(supervisor_.mode());
  controller_->selectVariant();               // a solver compiled while idle is loaded now
  status.controller = controller_->label();  // e.g. "MPC N=20, dt=0.100 s, heading: ..."
  // No yaw/vertical policies any more: both channels are the law's (yaw held unless it commands it).
  status.yaw_mode = controller_->name();
  status.vertical_mode = controller_->name();
  status.target_age = age ? static_cast<float>(*age) : -1.0f;
  status.bearing_age = bearing_age ? static_cast<float>(*bearing_age) : -1.0f;
  // Distance only with a valid estimate: in LOST the last value would stay frozen
  // on screen. In APPROACH, the estimated distance (last recent measurement or ROI).
  double range = NAN;
  if (supervisor_.mode() != PursuitMode::kApproach && target_ && age && *age <= lost_timeout_) {
    range = (target_->p_rel + target_->v_rel * *age).norm();
  }
  status.range = static_cast<float>(range);
  status.range_error = static_cast<float>(range - standoff_);
  status.command_speed = supervisor_.active() ? static_cast<float>(v_ref_.norm()) : 0.0f;

  // Own motion (PX4) and the target's, estimated from it: v_target = v_own + v_rel.
  const auto v_own = measuredChaserVelocity();
  const auto vehicle = measuredVehicleState();
  const auto put = [](std::array<float, 3> & dst, const Eigen::Vector3d & v) {
      for (int i = 0; i < 3; ++i) {dst[static_cast<std::size_t>(i)] = static_cast<float>(v(i));}
    };
  put(status.chaser_velocity, v_own.value_or(Eigen::Vector3d::Constant(NAN)));
  status.chaser_yaw_rate = vehicle ? static_cast<float>(vehicle->body_rates.z()) : NAN;
  Eigen::Vector3d v_target = Eigen::Vector3d::Constant(NAN);
  if (v_own && target_ && age && *age <= lost_timeout_) {
    v_target = *v_own + target_->v_rel;
    target_course_.update(v_target, dt_);
  } else {
    target_course_.reset();
  }
  put(status.target_velocity, v_target);
  status.target_turn_rate = static_cast<float>(target_course_.rate());
  // Active: what was really sent this cycle idle: the configured one.
  status.output = "trajectory";  // the only output: TrajectorySetpoint to the PX4 velocity loop
  pub_status_->publish(status);
}

}  // namespace aero_pursuit

RCLCPP_COMPONENTS_REGISTER_NODE(aero_pursuit::PursuitControllerNode)
