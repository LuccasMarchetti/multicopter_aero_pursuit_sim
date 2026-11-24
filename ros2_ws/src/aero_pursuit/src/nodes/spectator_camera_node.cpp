#include "aero_pursuit/nodes/spectator_camera_node.hpp"
#include "aero_pursuit/sim/ros_world.hpp"
#ifdef AERO_HAVE_GZ
#include "aero_pursuit/sim/gz_world.hpp"
#endif

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <cmath>
#include <fstream>
#include <sstream>

namespace aero_pursuit
{
namespace
{
/// The Gazebo camera looks along the link's +x; pitch > 0 tilts down.
Eigen::Quaterniond yawPitch(double yaw, double pitch)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
}

Eigen::Quaterniond lookAt(const Eigen::Vector3d & from, const Eigen::Vector3d & to)
{
  const Eigen::Vector3d d = to - from;
  return yawPitch(std::atan2(d.y(), d.x()), std::atan2(-d.z(), d.head<2>().norm()));
}
}  // namespace

SpectatorCameraNode::SpectatorCameraNode(const rclcpp::NodeOptions & options)
: Node("spectator_camera", options),
  last_tick_(0, 0, RCL_STEADY_TIME), last_sent_(0, 0, RCL_STEADY_TIME), last_spawn_(0, 0, RCL_STEADY_TIME)
{
  const auto backend = declare_parameter<std::string>("world_backend", "gz");
  const auto world = declare_parameter<std::string>("world_name", "lawn");
  world_frame_ = declare_parameter<std::string>("world_frame", world);
  model_name_ = declare_parameter<std::string>("model_name", "spectator_camera");
  auto sdf_path = declare_parameter<std::string>("model_sdf", "");
  const double rate = declare_parameter<double>("rate_hz", 30.0);
  smoothing_ = declare_parameter<double>("smoothing", 0.25);
  const auto initial = declare_parameter<std::vector<double>>("initial_pose", {-18.0, -14.0, 12.0, 0.6, 0.35});

  if (sdf_path.empty()) {
    sdf_path = ament_index_cpp::get_package_share_directory("aero_pursuit") +
      "/gz_models/spectator_camera/model.sdf";
  }
  std::ifstream in(sdf_path);
  std::stringstream ss;
  ss << in.rdbuf();
  model_sdf_ = ss.str();
  if (model_sdf_.empty() && backend == "ros") {
    model_sdf_ = "<isaac>";   // the ROS world only needs the model name (aero_world.py renders it)
  }
  if (model_sdf_.empty()) {
    RCLCPP_ERROR(get_logger(), "Free camera SDF not found: %s", sdf_path.c_str());
  }

  command_.mode = msg::SpectatorCommand::FREE;
  if (initial.size() >= 5) {
    command_.position.x = initial[0];
    command_.position.y = initial[1];
    command_.position.z = initial[2];
    command_.yaw = initial[3];
    command_.pitch = initial[4];
  }
  command_.distance = 10.0;
  command_.elevation = 0.35;
  command_.orbit_rate = 0.15;
  position_ = {command_.position.x, command_.position.y, command_.position.z};

  if (backend == "ros") {
    gz_ = std::make_unique<sim::RosWorld>(*this, world);
  } else {
#ifdef AERO_HAVE_GZ
    gz_ = std::make_unique<sim::GzWorld>(world);
#else
    throw std::runtime_error("world_backend '" + backend + "' unavailable (built without Gazebo): use ros");
#endif
  }
  gz_->subscribePoses();
  tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  sub_ = create_subscription<msg::SpectatorCommand>(
    "/spectator/command", 10, [this](const msg::SpectatorCommand & m) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (m.mode != command_.mode || m.target != command_.target) {
        orbit_azimuth_ = m.azimuth;
      }
      command_ = m;
      command_changed_ = true;
    });
  // Wall clock: the camera responds to the operator even with the simulation paused.
  timer_ = create_wall_timer(
    std::chrono::nanoseconds(static_cast<int64_t>(1e9 / std::clamp(rate, 1.0, 60.0))), [this] {tick();});
}

void SpectatorCameraNode::ensureModel()
{
  double stamp = 0.0;
  const auto poses = gz_->poses(&stamp);
  rclcpp::Clock steady(RCL_STEADY_TIME);
  if (stamp <= 0.0 || poses.count(model_name_) || model_sdf_.empty() ||
    (steady.now() - last_spawn_).seconds() < 10.0)
  {
    return;
  }
  last_spawn_ = steady.now();
  sim::ModelPose p;
  p.position = position_;
  gz_->spawn(model_name_, model_name_, model_sdf_, p, [this](bool ok) {
      if (!ok) {
        RCLCPP_WARN(get_logger(), "Could not insert the free camera (retrying in 10 s)");
      }
    });
}

void SpectatorCameraNode::tick()
{
  ensureModel();
  rclcpp::Clock steady(RCL_STEADY_TIME);
  const rclcpp::Time now = steady.now();
  const double dt = last_tick_.nanoseconds() > 0 ? std::clamp((now - last_tick_).seconds(), 0.0, 0.2) : 0.0;
  last_tick_ = now;

  msg::SpectatorCommand cmd;
  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd = command_;
    changed = command_changed_;
    command_changed_ = false;
  }

  const bool free = cmd.mode == msg::SpectatorCommand::FREE || cmd.target.empty();
  const auto target = free ? std::nullopt : gz_->pose(cmd.target);
  if (free || !target) {
    if (!changed && (now - last_sent_).seconds() < 1.0) {
      return;  // Nothing changed: do not keep Gazebo busy with set_pose
    }
    position_ = {cmd.position.x, cmd.position.y, cmd.position.z};
    orientation_ = yawPitch(cmd.yaw, cmd.pitch);
  } else {
    const Eigen::Vector3d p = target->position;
    double azimuth = 0.0;
    Eigen::Vector3d desired;
    if (cmd.mode == msg::SpectatorCommand::LOOK) {
      desired = {cmd.position.x, cmd.position.y, cmd.position.z};
    } else {
      if (cmd.mode == msg::SpectatorCommand::ORBIT) {
        orbit_azimuth_ += cmd.orbit_rate * dt;
        azimuth = orbit_azimuth_;
      } else {  // FOLLOW: behind the target, in its heading
        azimuth = sim::yawOf(target->orientation) + M_PI + cmd.azimuth;
      }
      const double d = std::max(1.0, cmd.distance);
      desired = p + d * Eigen::Vector3d(std::cos(cmd.elevation) * std::cos(azimuth),
        std::cos(cmd.elevation) * std::sin(azimuth), std::sin(cmd.elevation));
    }
    const double alpha = smoothing_ > 0.0 ? 1.0 - std::exp(-dt / smoothing_) : 1.0;
    position_ += alpha * (desired - position_);
    orientation_ = lookAt(position_, p);
  }
  sim::ModelPose pose;
  pose.position = position_;
  pose.orientation = orientation_;
  gz_->setPose(model_name_, pose, true);
  last_sent_ = now;

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = get_clock()->now();
  tf.header.frame_id = world_frame_;
  tf.child_frame_id = model_name_;
  tf.transform.translation.x = position_.x();
  tf.transform.translation.y = position_.y();
  tf.transform.translation.z = position_.z();
  tf.transform.rotation.w = orientation_.w();
  tf.transform.rotation.x = orientation_.x();
  tf.transform.rotation.y = orientation_.y();
  tf.transform.rotation.z = orientation_.z();
  tf_->sendTransform(tf);
}

}  // namespace aero_pursuit

RCLCPP_COMPONENTS_REGISTER_NODE(aero_pursuit::SpectatorCameraNode)
