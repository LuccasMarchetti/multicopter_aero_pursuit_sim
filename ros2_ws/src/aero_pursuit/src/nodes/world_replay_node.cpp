#include "aero_pursuit/nodes/world_replay_node.hpp"

#include "aero_pursuit/sim/ros_world.hpp"
#ifdef AERO_HAVE_GZ
#include "aero_pursuit/sim/gz_world.hpp"
#endif

#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <stdexcept>

namespace aero_pursuit
{

namespace
{
constexpr std::array<const char *, 3> kAxes{"yaw", "roll", "pitch"};
}  // namespace

WorldReplayNode::WorldReplayNode(const rclcpp::NodeOptions & options)
: Node("world_replay", options)
{
  const auto world = declare_parameter<std::string>("world_name", "lawn");
  const auto backend = declare_parameter<std::string>("world_backend", "gz");
  world_frame_ = declare_parameter<std::string>("world_frame", world);
  exclude_ = declare_parameter<std::vector<std::string>>("exclude_prefixes", {"spectator"});
  const auto tf_topic = declare_parameter<std::string>("tf_topic", "/replay/tf");
  const auto gimbal_topic = declare_parameter<std::string>("gimbal_state_topic", "/replay/gimbal/state");
  const auto gimbal_prefix = declare_parameter<std::string>("gimbal_command_prefix", "/gimbal/cmd");
  const double rate = declare_parameter<double>("rate_hz", 60.0);

  if (backend == "ros") {
    world_ = std::make_unique<sim::RosWorld>(*this, world);
  } else {
#ifdef AERO_HAVE_GZ
    world_ = std::make_unique<sim::GzWorld>(world);
#else
    throw std::runtime_error("world_backend gz unavailable (built without Gazebo): use ros");
#endif
  }
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  for (size_t i = 0; i < kAxes.size(); ++i) {
    gimbal_cmd_[i] = create_publisher<std_msgs::msg::Float64>(gimbal_prefix + "/" + kAxes[i], 10);
  }
  sub_tf_ = create_subscription<tf2_msgs::msg::TFMessage>(
    tf_topic, rclcpp::QoS(200), [this](const tf2_msgs::msg::TFMessage & m) {onTf(m);});
  sub_gimbal_ = create_subscription<sensor_msgs::msg::JointState>(
    gimbal_topic, 10, [this](const sensor_msgs::msg::JointState & m) {onGimbal(m);});
  // Wall timer: the replay follows the bag player, not the (idle) simulation clock.
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(rate, 1.0)), [this] {flush();});
  RCLCPP_INFO(get_logger(), "World replay '%s' (%s): poses from %s", world.c_str(), backend.c_str(),
    tf_topic.c_str());
}

void WorldReplayNode::onTf(const tf2_msgs::msg::TFMessage & msg)
{
  tf2_msgs::msg::TFMessage below;  // camera/gimbal frames: re-published with the current time
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & t : msg.transforms) {
      if (t.header.frame_id != world_frame_) {
        below.transforms.push_back(t);
        continue;
      }
      bool skip = false;
      for (const auto & prefix : exclude_) {
        skip = skip || t.child_frame_id.rfind(prefix, 0) == 0;
      }
      if (skip) {
        continue;
      }
      sim::ModelPose p;
      p.position = {t.transform.translation.x, t.transform.translation.y, t.transform.translation.z};
      const auto & q = t.transform.rotation;
      p.orientation = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
      pending_[t.child_frame_id] = p;
    }
  }
  if (!below.transforms.empty()) {
    const auto stamp = now();
    for (auto & t : below.transforms) {
      t.header.stamp = stamp;
    }
    tf_broadcaster_->sendTransform(below.transforms);
  }
}

void WorldReplayNode::onGimbal(const sensor_msgs::msg::JointState & msg)
{
  for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
    for (size_t a = 0; a < kAxes.size(); ++a) {
      if (msg.name[i].find(kAxes[a]) != std::string::npos) {
        std_msgs::msg::Float64 cmd;
        cmd.data = msg.position[i];
        gimbal_cmd_[a]->publish(cmd);
      }
    }
  }
}

void WorldReplayNode::flush()
{
  std::vector<std::pair<std::string, sim::ModelPose>> batch;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    batch.assign(pending_.begin(), pending_.end());
    pending_.clear();
  }
  world_->setPoses(batch);
}

}  // namespace aero_pursuit

RCLCPP_COMPONENTS_REGISTER_NODE(aero_pursuit::WorldReplayNode)
