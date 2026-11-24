#include "aero_pursuit/nodes/target_estimator_node.hpp"

#include "aero_pursuit/common/frames.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer_interface.h>

#include <algorithm>
#include <cmath>
#include <iterator>

namespace aero_pursuit
{

TargetEstimatorNode::TargetEstimatorNode(const rclcpp::NodeOptions & options)
: Node("target_estimator_node", options),
  last_stamp_(0, 0, get_clock()->get_clock_type()),
  last_enu_tf_stamp_(0, 0, get_clock()->get_clock_type())
{
  const auto input_topic = declare_parameter<std::string>("input_topic", "/target_3d_position");
  const auto output_topic = declare_parameter<std::string>("output_topic", "/target_3d_state_filtered");
  const auto bearing_topic = declare_parameter<std::string>("bearing_topic", "/target_bearing");
  const auto bearing_enu_topic =
    declare_parameter<std::string>("bearing_output_topic", "/target_bearing_enu");
  const auto attitude_topic = declare_parameter<std::string>("attitude_topic", "/fmu/out/vehicle_attitude");
  base_frame_ = declare_parameter<std::string>("base_frame", "x500_depth_0");
  enu_frame_ = declare_parameter<std::string>("enu_frame", "x500_depth_0_enu");
  estimate_frame_ = declare_parameter<std::string>("estimate_frame", "target_estimate");
  tf_target_frame_ = declare_parameter<std::string>("default_target_frame", "X4");
  world_frame_ = declare_parameter<std::string>("world_frame", "lawn");
  use_tf_tracking_ = declare_parameter<bool>("use_tf_tracking", false);
  reset_timeout_ = declare_parameter<double>("reset_timeout", 1.0);
  gate_chi2_ = declare_parameter<double>("gating_chi2", 16.27);  // 99,9% p/ 3 GDL
  max_gate_rejections_ = static_cast<int>(declare_parameter<int>("max_gate_rejections", 5));
  const double tf_rate = declare_parameter<double>("tf_lookup_rate", 30.0);
  const double enu_tf_rate = declare_parameter<double>("enu_tf_rate", 50.0);
  attitude_history_sec_ = declare_parameter<double>("attitude_history_sec", 1.0);
  const auto odometry_topic =
    declare_parameter<std::string>("odometry_topic", "/fmu/out/vehicle_odometry");
  compensate_chaser_motion_ = declare_parameter<bool>("compensate_chaser_motion", true);
  enu_tf_period_ = enu_tf_rate > 0.0 ? 1.0 / enu_tf_rate : 0.0;

  rcl_interfaces::msg::ParameterDescriptor model_desc;
  model_desc.description =
    "cv (constant velocity) | singer (target acceleration as a Gauss-Markov process)";
  kf_model_ = declare_parameter<std::string>("kf_model", "cv", model_desc);
  cv_params_.process_noise_std = declare_parameter<double>("process_noise_std", 0.5);
  cv_params_.measurement_noise_std = declare_parameter<double>("measurement_noise_std", 0.05);
  cv_params_.initial_velocity_std = declare_parameter<double>("initial_velocity_std", 1.0);
  singer_params_.accel_std = declare_parameter<double>("singer.accel_std", 1.0);
  singer_params_.accel_tau = declare_parameter<double>("singer.accel_tau", 1.0);
  rcl_interfaces::msg::ParameterDescriptor win_desc;
  win_desc.description = "Omniscient: 0 = acceleration from the 2nd difference of two poses (exact, but with the "
    "noise of the interval between stamps); > 0 = quadratic fit of the poses in this window [s]";
  omni_accel_window_ = declare_parameter<double>("omniscient_accel_window", 0.0, win_desc);
  buildFilter();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, false);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(output_topic, 10);
  pub_accel_ = create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
    declare_parameter<std::string>("accel_output_topic", "/target_3d_accel_filtered"), 10);
  pub_marker_ = create_publisher<visualization_msgs::msg::Marker>("/target_marker", 10);

  sub_measurement_ = create_subscription<PointStamped>(
    input_topic, 10, [this](const PointStamped & msg) {onMeasurement(msg);});
  sub_attitude_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
    attitude_topic, rclcpp::SensorDataQoS(),
    [this](const px4_msgs::msg::VehicleAttitude & msg) {onAttitude(msg);});
  sub_odometry_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
    odometry_topic, rclcpp::SensorDataQoS(),
    [this](const px4_msgs::msg::VehicleOdometry & msg) {onOdometry(msg);});
  sub_tf_cmd_ = create_subscription<msg::TargetTfCmd>(
    "/target_tf_cmd", 10, [this](const msg::TargetTfCmd & msg) {onTfCommand(msg);});
  sub_bearing_ = create_subscription<msg::TargetBearing>(
    bearing_topic, 10, [this](const msg::TargetBearing & msg) {onBearing(msg);});
  pub_bearing_ = create_publisher<msg::TargetBearing>(bearing_enu_topic, 10);

  param_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {return onParameters(params);});

  omniscient_timer_ = create_timer(
    std::chrono::duration<double>(1.0 / tf_rate), [this]() {onOmniscientTimer();});

  RCLCPP_INFO(get_logger(), "Estimador: base='%s' enu='%s' | modo inicial: %s",
    base_frame_.c_str(), enu_frame_.c_str(),
    use_tf_tracking_ ? ("TF -> " + tf_target_frame_).c_str() : "VISION");
}

void TargetEstimatorNode::onAttitude(const px4_msgs::msg::VehicleAttitude & msg)
{
  if (!std::isfinite(msg.q[0])) {
    return;
  }
  q_enu_flu_ = frames::px4AttitudeToEnuFlu(msg.q);

  const rclcpp::Time now = get_clock()->now();
  if (!attitude_history_.empty() && now < attitude_history_.back().first) {
    attitude_history_.clear();  // Simulation time went back (Gazebo reset)
    last_enu_tf_stamp_ = rclcpp::Time(0, 0, now.get_clock_type());
  }
  attitude_history_.emplace_back(now, *q_enu_flu_);
  while (attitude_history_.size() > 2 &&
    (now - attitude_history_.front().first).seconds() > attitude_history_sec_)
  {
    attitude_history_.pop_front();
  }

  // TF base -> enu (rotation only), rate limited.
  if ((now - last_enu_tf_stamp_).seconds() < enu_tf_period_) {
    return;
  }
  last_enu_tf_stamp_ = now;

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = now;
  tf.header.frame_id = base_frame_;
  tf.child_frame_id = enu_frame_;
  tf.transform.rotation = tf2::toMsg(q_enu_flu_->inverse());
  tf_broadcaster_->sendTransform(tf);
}

std::optional<Eigen::Vector3d> TargetEstimatorNode::windowedAcceleration(double t, const Eigen::Vector3d & p)
{
  if (!omni_target_hist_.empty() && t <= omni_target_hist_.back().first) {
    omni_target_hist_.clear();  // Clock went back
  }
  omni_target_hist_.emplace_back(t, p);
  while (!omni_target_hist_.empty() && t - omni_target_hist_.front().first > omni_accel_window_) {
    omni_target_hist_.pop_front();
  }
  if (omni_target_hist_.size() < 4) {
    return std::nullopt;
  }
  // Least squares p(τ) = c0 + c1 τ + c2 τ², τ = t_i - t (exact stamps, any spacing); a = 2 c2.
  Eigen::MatrixXd A(omni_target_hist_.size(), 3);
  Eigen::MatrixXd B(omni_target_hist_.size(), 3);
  for (std::size_t i = 0; i < omni_target_hist_.size(); ++i) {
    const double tau = omni_target_hist_[i].first - t;
    A.row(i) << 1.0, tau, tau * tau;
    B.row(i) = omni_target_hist_[i].second.transpose();
  }
  const Eigen::MatrixXd C = A.colPivHouseholderQr().solve(B);
  return Eigen::Vector3d(2.0 * C.row(2).transpose());
}

void TargetEstimatorNode::buildFilter()
{
  if (kf_model_ == "singer") {
    singer_params_.measurement_noise_std = cv_params_.measurement_noise_std;
    singer_params_.initial_velocity_std = cv_params_.initial_velocity_std;
    kf_ = std::make_unique<estimation::SingerKF>(singer_params_);
  } else {
    kf_ = std::make_unique<estimation::ConstantVelocityKF>(cv_params_);
  }
  RCLCPP_INFO(get_logger(), "Target filter: %s", kf_model_.c_str());
}

void TargetEstimatorNode::onTfCommand(const msg::TargetTfCmd & msg)
{
  // Keeps the parameters consistent with the command (visible via `ros2 param get`).
  std::vector<rclcpp::Parameter> params{rclcpp::Parameter("use_tf_tracking", msg.enable_tf_tracking)};
  if (!msg.target_frame.empty()) {
    params.emplace_back("default_target_frame", msg.target_frame);
  }
  set_parameters_atomically(params);
}

void TargetEstimatorNode::setTrackingSource(bool use_tf, const std::string & target_frame)
{
  const bool changed = use_tf != use_tf_tracking_ || (use_tf && target_frame != tf_target_frame_);
  tf_target_frame_ = target_frame;
  use_tf_tracking_ = use_tf;
  if (changed) {
    omni_prev_.reset();
    omni_target_prev_.reset();
    omni_target_vel_.reset();
    kf_->invalidate();  // Resets the filter for a clean transition
    RCLCPP_INFO(get_logger(), ">>> Measurement source: %s <<<",
      use_tf_tracking_ ? ("target TF '" + tf_target_frame_ + "'").c_str() : "VISION");
  }
}

rcl_interfaces::msg::SetParametersResult TargetEstimatorNode::onParameters(
  const std::vector<rclcpp::Parameter> & params)
{
  bool use_tf = use_tf_tracking_;
  std::string target_frame = tf_target_frame_;
  bool kf_changed = false;

  for (const auto & p : params) {
    const auto & name = p.get_name();
    if (name == "use_tf_tracking") {
      use_tf = p.as_bool();
    } else if (name == "default_target_frame") {
      target_frame = p.as_string();
    } else if (name == "reset_timeout") {
      reset_timeout_ = p.as_double();
    } else if (name == "compensate_chaser_motion") {
      compensate_chaser_motion_ = p.as_bool();
    } else if (name == "gating_chi2") {
      gate_chi2_ = p.as_double();
    } else if (name == "max_gate_rejections") {
      max_gate_rejections_ = static_cast<int>(p.as_int());
    } else if (name == "kf_model") {
      if (p.as_string() != "cv" && p.as_string() != "singer") {
        rcl_interfaces::msg::SetParametersResult bad;
        bad.successful = false;
        bad.reason = "invalid kf_model: '" + p.as_string() + "' (cv|singer)";
        return bad;
      }
      kf_model_ = p.as_string();
      kf_changed = true;
    } else if (name == "process_noise_std") {
      cv_params_.process_noise_std = p.as_double();
      kf_changed = true;
    } else if (name == "measurement_noise_std") {
      cv_params_.measurement_noise_std = p.as_double();
      kf_changed = true;
    } else if (name == "initial_velocity_std") {
      cv_params_.initial_velocity_std = p.as_double();
      kf_changed = true;
    } else if (name == "singer.accel_std") {
      singer_params_.accel_std = p.as_double();
      kf_changed = true;
    } else if (name == "omniscient_accel_window") {
      omni_accel_window_ = std::max(0.0, p.as_double());
      omni_target_hist_.clear();
    } else if (name == "singer.accel_tau") {
      singer_params_.accel_tau = p.as_double();
      kf_changed = true;
    }
  }
  if (kf_changed) {
    buildFilter();
  }
  setTrackingSource(use_tf, target_frame);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

std::optional<Eigen::Isometry3d> TargetEstimatorNode::cameraToBase(
  const std::string & frame, const rclcpp::Time & stamp)
{
  if (frame == base_frame_) {
    return Eigen::Isometry3d::Identity();
  }
  // Looked up at the measurement instant: with the gimbal (camera_mount:=gimbal) the camera
  // TF is dynamic. A static chain (fixed camera) is returned for any instant. The image can be
  // newer than the last joint TF: then the most recent one is used.
  try {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(base_frame_, frame, tf2_ros::fromRclcpp(stamp));
    } catch (const tf2::ExtrapolationException &) {
      tf = tf_buffer_->lookupTransform(base_frame_, frame, tf2::TimePointZero);
    }
    return tf2::transformToEigen(tf);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Aguardando TF '%s' -> '%s': %s", base_frame_.c_str(), frame.c_str(), ex.what());
    return std::nullopt;
  }
}

void TargetEstimatorNode::onOdometry(const px4_msgs::msg::VehicleOdometry & msg)
{
  using Odom = px4_msgs::msg::VehicleOdometry;
  const Eigen::Vector3d v(msg.velocity[0], msg.velocity[1], msg.velocity[2]);
  if (msg.velocity_frame != Odom::VELOCITY_FRAME_NED || !v.allFinite()) {
    return;
  }
  // Same clock (reception instant) as the attitude history.
  const rclcpp::Time now = get_clock()->now();
  if (!velocity_history_.empty() && now < velocity_history_.back().first) {
    velocity_history_.clear();
  }
  velocity_history_.emplace_back(now, frames::nedToEnu(v));
  while (velocity_history_.size() > 2 &&
    (now - velocity_history_.front().first).seconds() > attitude_history_sec_)
  {
    velocity_history_.pop_front();
  }
}

std::optional<Eigen::Vector3d> TargetEstimatorNode::chaserVelocityAt(const rclcpp::Time & stamp) const
{
  if (velocity_history_.empty()) {
    return std::nullopt;
  }
  if (stamp >= velocity_history_.back().first) {
    return velocity_history_.back().second;
  }
  if (stamp <= velocity_history_.front().first) {
    return velocity_history_.front().second;
  }
  const auto after = std::lower_bound(
    velocity_history_.begin(), velocity_history_.end(), stamp,
    [](const auto & entry, const rclcpp::Time & t) {return entry.first < t;});
  const auto before = std::prev(after);
  const double span = (after->first - before->first).seconds();
  const double s = span > 0.0 ? (stamp - before->first).seconds() / span : 1.0;
  return before->second + s * (after->second - before->second);
}

std::optional<Eigen::Quaterniond> TargetEstimatorNode::attitudeAt(const rclcpp::Time & stamp) const
{
  if (attitude_history_.empty()) {
    return std::nullopt;
  }
  if (stamp >= attitude_history_.back().first) {
    return attitude_history_.back().second;
  }
  if (stamp <= attitude_history_.front().first) {
    return attitude_history_.front().second;
  }
  const auto after = std::lower_bound(
    attitude_history_.begin(), attitude_history_.end(), stamp,
    [](const auto & entry, const rclcpp::Time & t) {return entry.first < t;});
  const auto before = std::prev(after);
  const double span = (after->first - before->first).seconds();
  const double s = span > 0.0 ? (stamp - before->first).seconds() / span : 1.0;
  return before->second.slerp(s, after->second).normalized();
}

rclcpp::Time TargetEstimatorNode::measurementTime(const builtin_interfaces::msg::Time & stamp)
{
  // Protection against inconsistent clocks (e.g. images in simulation time
  // with use_sim_time off): uses the reception instant.
  const rclcpp::Time now = get_clock()->now();
  const rclcpp::Time t(stamp, now.get_clock_type());
  if (std::abs((now - t).seconds()) > 5.0) {
    if (!warned_clock_) {
      RCLCPP_WARN(get_logger(),
        "Measurement stamp differs from the node clock by %.1f s. Check use_sim_time. "
        "Using the reception time.", (now - t).seconds());
      warned_clock_ = true;
    }
    return now;
  }
  return t;
}

void TargetEstimatorNode::onMeasurement(const PointStamped & msg)
{
  if (use_tf_tracking_) {
    return;
  }
  const rclcpp::Time stamp = measurementTime(msg.header.stamp);
  const auto T = cameraToBase(msg.header.frame_id, stamp);
  if (!T) {
    return;
  }
  const Eigen::Vector3d p(msg.point.x, msg.point.y, msg.point.z);
  processMeasurement(*T * p, stamp);
}

void TargetEstimatorNode::onBearing(const msg::TargetBearing & msg)
{
  if (use_tf_tracking_) {
    return;
  }
  const rclcpp::Time stamp = measurementTime(msg.header.stamp);
  const auto q = attitudeAt(stamp);
  const auto T = q ? cameraToBase(msg.header.frame_id, stamp) : std::nullopt;
  if (!T) {
    return;
  }
  const Eigen::Vector3d d(msg.direction.x, msg.direction.y, msg.direction.z);
  const Eigen::Vector3d d_enu = (*q * (T->linear() * d)).normalized();

  msg::TargetBearing out = msg;
  out.header.stamp = stamp;
  out.header.frame_id = enu_frame_;
  out.direction.x = d_enu.x();
  out.direction.y = d_enu.y();
  out.direction.z = d_enu.z();
  pub_bearing_->publish(out);
}

void TargetEstimatorNode::onOmniscientTimer()
{
  if (!use_tf_tracking_) {
    return;
  }
  try {
    // Most recent common instant of the pursuer -> target chain.
    const auto rel = tf_buffer_->lookupTransform(base_frame_, tf_target_frame_, tf2::TimePointZero);
    const rclcpp::Time stamp = measurementTime(rel.header.stamp);
    if (omni_prev_ && stamp <= omni_prev_->first) {
      return;  // No new pose since the last read
    }
    // Relative position directly in the world (ENU), both poses at the same instant: does not
    // mix Gazebo's real attitude with the attitude estimated by the PX4 EKF, which
    // made the point "sway" with the drone tilt (error ~ distance x attitude error).
    const tf2::TimePoint t = tf2_ros::fromMsg(rel.header.stamp);
    const auto target = tf_buffer_->lookupTransform(world_frame_, tf_target_frame_, t);
    const auto chaser = tf_buffer_->lookupTransform(world_frame_, base_frame_, t);
    const auto & a = target.transform.translation;
    const auto & b = chaser.transform.translation;
    // Omniscient = simulator ground truth: no filter. Relative velocity from the exact
    // difference between two consecutive poses (Gazebo does not publish velocity in the TFs).
    const Eigen::Vector3d z(a.x - b.x, a.y - b.y, a.z - b.z);
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    if (omni_prev_) {
      const double dt = (stamp - omni_prev_->first).seconds();
      if (dt > 0.0 && dt <= reset_timeout_) {
        v = (z - omni_prev_->second) / dt;
      }
    }
    omni_prev_ = std::make_pair(stamp, z);
    // Absolute target acceleration: exact second difference of the world position.
    const Eigen::Vector3d p_t(a.x, a.y, a.z);
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    if (omni_target_prev_) {
      const double dt = (stamp - omni_target_prev_->first).seconds();
      if (dt > 0.0 && dt <= reset_timeout_) {
        const Eigen::Vector3d v_t = (p_t - omni_target_prev_->second) / dt;
        if (omni_target_vel_) {
          acc = (v_t - *omni_target_vel_) / dt;
        }
        omni_target_vel_ = v_t;
      } else {
        omni_target_vel_.reset();
      }
    }
    omni_target_prev_ = std::make_pair(stamp, p_t);
    if (omni_accel_window_ > 0.0) {
      acc = windowedAcceleration(stamp.seconds(), p_t).value_or(acc);
    }
    last_stamp_ = stamp;
    publishEstimate(stamp, z, v, estimation::TargetFilter::CovariancePV::Zero(), acc);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Waiting for TF '%s' -> '%s' (world '%s'): %s", base_frame_.c_str(),
      tf_target_frame_.c_str(), world_frame_.c_str(), ex.what());
  }
}

void TargetEstimatorNode::processMeasurement(const Eigen::Vector3d & p_base, const rclcpp::Time & stamp)
{
  const auto q = attitudeAt(stamp);
  if (!q) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "No PX4 attitude: measurement dropped.");
    return;
  }
  processMeasurementEnu(*q * p_base, stamp);
}

void TargetEstimatorNode::processMeasurementEnu(const Eigen::Vector3d & z, const rclcpp::Time & stamp)
{
  const double dt = (stamp - last_stamp_).seconds();
  if (!kf_->initialized() || dt > reset_timeout_ || dt < 0.0) {
    kf_->reset(z);
    gate_rejections_ = 0;
    last_stamp_ = stamp;
    publishEstimate(stamp);
    return;
  }
  if (dt == 0.0) {
    return;
  }

  // Change of the pursuer velocity over the interval: known KF input.
  const auto v_prev = compensate_chaser_motion_ ? chaserVelocityAt(last_stamp_) : std::nullopt;
  const auto v_now = v_prev ? chaserVelocityAt(stamp) : std::nullopt;
  if (v_now) {
    kf_->predict(dt, *v_now - *v_prev);
  } else {
    kf_->predict(dt, Eigen::Vector3d::Zero());
  }
  last_stamp_ = stamp;

  if (!kf_->update(z, gate_chi2_)) {
    if (++gate_rejections_ >= max_gate_rejections_) {
      RCLCPP_WARN(get_logger(), "%d measurements rejected in a row: resetting the filter.",
        gate_rejections_);
      kf_->reset(z);
      gate_rejections_ = 0;
    } else {
      return;  // Keeps only the prediction; does not publish a spurious measurement
    }
  } else {
    gate_rejections_ = 0;
  }
  publishEstimate(stamp);
}

void TargetEstimatorNode::publishEstimate(const rclcpp::Time & stamp)
{
  publishEstimate(stamp, kf_->position(), kf_->velocity(), kf_->covariancePV(), kf_->acceleration(),
    kf_->covarianceA());
}

void TargetEstimatorNode::publishEstimate(
  const rclcpp::Time & stamp, const Eigen::Vector3d & p, const Eigen::Vector3d & v,
  const estimation::TargetFilter::CovariancePV & P, const Eigen::Vector3d & a,
  const Eigen::Matrix3d & P_a)
{
  auto odom = std::make_unique<nav_msgs::msg::Odometry>();
  odom->header.stamp = stamp;
  odom->header.frame_id = enu_frame_;
  odom->child_frame_id = estimate_frame_;
  odom->pose.pose.position.x = p.x();
  odom->pose.pose.position.y = p.y();
  odom->pose.pose.position.z = p.z();
  odom->pose.pose.orientation.w = 1.0;
  odom->twist.twist.linear.x = v.x();
  odom->twist.twist.linear.y = v.y();
  odom->twist.twist.linear.z = v.z();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      odom->pose.covariance[i * 6 + j] = P(i, j);
      odom->twist.covariance[i * 6 + j] = P(3 + i, 3 + j);
    }
  }
  pub_odom_->publish(std::move(odom));

  geometry_msgs::msg::AccelWithCovarianceStamped acc;
  acc.header.stamp = stamp;
  acc.header.frame_id = enu_frame_;  // ENU orientation; absolute target acceleration
  acc.accel.accel.linear.x = a.x();
  acc.accel.accel.linear.y = a.y();
  acc.accel.accel.linear.z = a.z();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      acc.accel.covariance[i * 6 + j] = P_a(i, j);
    }
  }
  pub_accel_->publish(acc);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = enu_frame_;
  tf.child_frame_id = estimate_frame_;
  tf.transform.translation.x = p.x();
  tf.transform.translation.y = p.y();
  tf.transform.translation.z = p.z();
  tf.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(tf);

  if (pub_marker_->get_subscription_count() > 0) {
    visualization_msgs::msg::Marker marker;
    marker.header = tf.header;
    marker.ns = "target";
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = p.x();
    marker.pose.position.y = p.y();
    marker.pose.position.z = p.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.3;
    marker.color.r = 1.0f;
    marker.color.a = 0.8f;
    marker.lifetime = rclcpp::Duration::from_seconds(reset_timeout_);
    pub_marker_->publish(marker);
  }
}

}  // namespace aero_pursuit

RCLCPP_COMPONENTS_REGISTER_NODE(aero_pursuit::TargetEstimatorNode)
