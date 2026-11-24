#include "aero_pursuit/nodes/gimbal_node.hpp"

#include "aero_pursuit/common/frames.hpp"
#include "aero_pursuit/control/gimbal_pitch.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <cmath>
#include <vector>

namespace aero_pursuit
{

namespace
{
constexpr double kDeg = M_PI / 180.0;
}

GimbalNode::GimbalNode(const rclcpp::NodeOptions & options)
: Node("gimbal", options)
{
  mount_frame_ = declare_parameter<std::string>("mount_frame", "gimbal_mount");
  camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_link");
  joint_name_ = declare_parameter<std::string>("joint_name", "gimbal_pitch_joint");
  const auto mount = declare_parameter<std::vector<double>>("mount_xyz", {0.15, 0.03, 0.165});
  t_mount_ = Eigen::Vector3d(mount.at(0), mount.at(1), mount.at(2));
  const double rate = declare_parameter<double>("rate_hz", 50.0);
  dt_ = 1.0 / rate;
  // Run-time (GCS): the check box and the manual angle.
  declare_parameter<bool>("auto_pitch", false);   // false: fixed camera at pitch_deg
  declare_parameter<double>("pitch_deg", 0.0);       // manual angle (+ = up) [deg]
  declare_parameter<double>("max_rate_deg", 90.0);   // [deg/s]
  declare_parameter<double>("min_deg", -90.0);       // command range; x500_gimbal_depth joint stops at -109..46 (velocity-driven, it stuck at a stop)
  declare_parameter<double>("max_deg", 30.0);
  declare_parameter<double>("target_timeout", 0.5);  // [s]

  tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  pub_joint_command_ = create_publisher<sensor_msgs::msg::JointState>("/gimbal/joint_command", 10);
  pub_gz_pitch_ = create_publisher<std_msgs::msg::Float64>("/gimbal/cmd/pitch", 10);
  sub_joints_ = create_subscription<sensor_msgs::msg::JointState>(
    "/gimbal/joint_states", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState & m) {onJointStates(m);});
  sub_target_ = create_subscription<nav_msgs::msg::Odometry>(
    "/target_3d_state_filtered", 10, [this](const nav_msgs::msg::Odometry & m) {
      const auto & p = m.pose.pose.position;
      target_ = std::make_pair(Eigen::Vector3d(p.x, p.y, p.z), now());
    });
  sub_bearing_ = create_subscription<msg::TargetBearing>(
    "/target_bearing_enu", 10, [this](const msg::TargetBearing & m) {
      bearing_ = std::make_pair(Eigen::Vector3d(m.direction.x, m.direction.y, m.direction.z), now());
    });
  sub_status_ = create_subscription<msg::PursuitStatus>(
    "/pursuit/status", 10, [this](const msg::PursuitStatus & m) {pursuit_mode_ = m.mode;});
  sub_odometry_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
    [this](const px4_msgs::msg::VehicleOdometry & m) {onOdometry(m);});
  timer_ = create_timer(std::chrono::duration<double>(dt_), [this]() {step();});
  RCLCPP_INFO(get_logger(), "Gimbal (pitch): %s -> %s, junta %s", mount_frame_.c_str(),
    camera_frame_.c_str(), joint_name_.c_str());
}

void GimbalNode::onOdometry(const px4_msgs::msg::VehicleOdometry & m)
{
  if (m.pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED || !std::isfinite(m.q[0])) {
    return;
  }
  attitude_ = std::make_pair(frames::px4AttitudeToEnuFlu(m.q), now());
}

void GimbalNode::onJointStates(const sensor_msgs::msg::JointState & m)
{
  for (size_t i = 0; i < m.name.size() && i < m.position.size(); ++i) {
    if (m.name[i].find(joint_name_) == std::string::npos) {  // Gazebo may scope it: model::joint
      continue;
    }
    measured_ = m.position[i];
    const rclcpp::Time stamp(m.header.stamp, get_clock()->get_clock_type());
    // The simulators publish every physics step: 100 Hz is enough for the TF.
    if (last_tf_stamp_.nanoseconds() != 0 && stamp > last_tf_stamp_ &&
      (stamp - last_tf_stamp_).seconds() < 0.0099)
    {
      return;
    }
    last_tf_stamp_ = stamp;
    // camera_link = gimbal_mount turned by -pitch about y (the joint axis is -y: + = up).
    const Eigen::Quaterniond q(Eigen::AngleAxisd(-m.position[i], Eigen::Vector3d::UnitY()));
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = m.header.stamp;
    tf.header.frame_id = mount_frame_;
    tf.child_frame_id = camera_frame_;
    tf.transform.rotation.w = q.w();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf_->sendTransform(tf);
    return;
  }
}

std::optional<double> GimbalNode::lawPitch()
{
  const double timeout = get_parameter("target_timeout").as_double();
  const auto t = now();
  if (!attitude_ || (t - attitude_->second).seconds() > timeout) {
    return std::nullopt;
  }
  if (target_ && (t - target_->second).seconds() <= timeout) {
    return control::gimbalPitchToward(target_->first, attitude_->first, t_mount_);
  }
  if (bearing_ && (t - bearing_->second).seconds() <= timeout) {
    // No depth: only the direction (the pivot offset needs a distance; negligible far away).
    return control::gimbalPitchToward(bearing_->first, attitude_->first);
  }
  return std::nullopt;
}

void GimbalNode::step()
{
  const double lo = get_parameter("min_deg").as_double() * kDeg;
  const double hi = get_parameter("max_deg").as_double() * kDeg;
  std::optional<double> desired;
  using S = msg::PursuitStatus;
  const bool target_lost = pursuit_mode_ == S::LOST || pursuit_mode_ == S::IDLE ||
    pursuit_mode_ == S::DISABLED;
  if (get_parameter("auto_pitch").as_bool() && !target_lost) {
    desired = lawPitch();          // empty: target briefly missing, hold the last command
  } else {
    desired = get_parameter("pitch_deg").as_double() * kDeg;  // fixed camera, or target lost
  }
  if (!have_command_) {
    command_ = measured_.value_or(0.0);
    have_command_ = true;
  }
  if (desired) {
    command_ = control::stepToward(command_, *desired, get_parameter("max_rate_deg").as_double() * kDeg,
      dt_, lo, hi);
  }
  sensor_msgs::msg::JointState js;
  js.header.stamp = now();
  js.name = {joint_name_};
  js.position = {command_};
  pub_joint_command_->publish(js);
  std_msgs::msg::Float64 f;
  f.data = command_;
  pub_gz_pitch_->publish(f);
}

}  // namespace aero_pursuit

RCLCPP_COMPONENTS_REGISTER_NODE(aero_pursuit::GimbalNode)
