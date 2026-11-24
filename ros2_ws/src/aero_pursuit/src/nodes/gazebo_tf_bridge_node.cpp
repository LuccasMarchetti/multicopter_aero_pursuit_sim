#include "aero_pursuit/nodes/gazebo_tf_bridge_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <stdexcept>
#include <vector>

namespace aero_pursuit
{

GazeboTfBridgeNode::GazeboTfBridgeNode(const rclcpp::NodeOptions & options)
: Node("gazebo_tf_bridge", options)
{
  const auto world_name = declare_parameter<std::string>("world_name", "lawn");
  world_frame_ = declare_parameter<std::string>("world_frame", world_name);
  const auto models = declare_parameter<std::vector<std::string>>(
    "models", {"x500_depth_0", "X3", "X4", "Tiltrotor", "X5"});
  use_sim_stamp_ = declare_parameter<bool>("use_sim_stamp", true);
  const double max_rate = declare_parameter<double>("max_rate_hz", 60.0);
  min_period_ns_ = max_rate > 0.0 ? static_cast<int64_t>(1e9 / max_rate) : 0;

  for (const auto & m : models) {
    if (m != world_frame_) {  // Evita a TF degenerada world -> world
      models_.insert(m);
    }
  }

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  const std::string topic = "/world/" + world_name + "/dynamic_pose/info";
  if (!gz_node_.Subscribe(topic, &GazeboTfBridgeNode::onPoses, this)) {
    throw std::runtime_error("Could not subscribe to the Gazebo topic: " + topic);
  }
  RCLCPP_INFO(get_logger(), "Gazebo TF: '%s' -> %zu models (%.0f Hz max.)",
    topic.c_str(), models_.size(), max_rate);
}

void GazeboTfBridgeNode::onPoses(const gz::msgs::Pose_V & msg)
{
  const bool has_stamp = use_sim_stamp_ && msg.has_header() && msg.header().has_stamp();
  const rclcpp::Time stamp = has_stamp ?
    rclcpp::Time(
      static_cast<int32_t>(msg.header().stamp().sec()),
      static_cast<uint32_t>(msg.header().stamp().nsec()), RCL_ROS_TIME) :
    get_clock()->now();

  std::lock_guard<std::mutex> lock(mutex_);
  const int64_t t_ns = stamp.nanoseconds();
  if (min_period_ns_ > 0 && t_ns >= last_publish_ns_ && t_ns - last_publish_ns_ < min_period_ns_) {
    return;
  }
  last_publish_ns_ = t_ns;

  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.reserve(models_.size());
  for (const auto & pose : msg.pose()) {
    if (models_.count(pose.name()) == 0) {
      continue;
    }
    auto & tf = transforms.emplace_back();
    tf.header.stamp = stamp;
    tf.header.frame_id = world_frame_;
    tf.child_frame_id = pose.name();
    tf.transform.translation.x = pose.position().x();
    tf.transform.translation.y = pose.position().y();
    tf.transform.translation.z = pose.position().z();
    tf.transform.rotation.x = pose.orientation().x();
    tf.transform.rotation.y = pose.orientation().y();
    tf.transform.rotation.z = pose.orientation().z();
    tf.transform.rotation.w = pose.orientation().w();
  }

  if (!transforms.empty()) {
    tf_broadcaster_->sendTransform(transforms);
  }
}

}  // namespace aero_pursuit

RCLCPP_COMPONENTS_REGISTER_NODE(aero_pursuit::GazeboTfBridgeNode)
