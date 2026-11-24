#include "aero_pursuit/sim/ros_world.hpp"

#include <utility>

namespace aero_pursuit::sim
{

RosWorld::RosWorld(rclcpp::Node & node, std::string world, const std::string & prefix)
: node_(node), world_(std::move(world)), prefix_(prefix)
{
  const auto cmd_qos = rclcpp::QoS(50).reliable();
  pub_spawn_ = node_.create_publisher<geometry_msgs::msg::TransformStamped>(prefix_ + "/spawn", cmd_qos);
  pub_set_pose_ = node_.create_publisher<geometry_msgs::msg::TransformStamped>(prefix_ + "/set_pose", cmd_qos);
  pub_remove_ = node_.create_publisher<std_msgs::msg::String>(prefix_ + "/remove", cmd_qos);
  pub_enable_ = node_.create_publisher<std_msgs::msg::String>(prefix_ + "/enable", cmd_qos);
  pub_twist_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>(prefix_ + "/twist", cmd_qos);
}

bool RosWorld::subscribePoses()
{
  sub_poses_ = node_.create_subscription<tf2_msgs::msg::TFMessage>(
    prefix_ + "/poses", rclcpp::QoS(10).reliable(),
    [this](const tf2_msgs::msg::TFMessage & msg) {onPoses(msg);});
  return static_cast<bool>(sub_poses_);
}

void RosWorld::onPoses(const tf2_msgs::msg::TFMessage & msg)
{
  if (msg.transforms.empty()) {
    return;
  }
  std::vector<Done> succeeded, failed;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Each message carries the whole world: a model missing from it no longer exists.
    poses_.clear();
    stamp_ = rclcpp::Time(msg.transforms.front().header.stamp).seconds();
    for (const auto & tf : msg.transforms) {
      ModelPose p;
      p.position = {tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z};
      p.orientation = Eigen::Quaterniond(
        tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z);
      p.stamp = rclcpp::Time(tf.header.stamp).seconds();
      poses_[tf.child_frame_id] = p;
    }
    const auto now = Clock::now();
    for (auto it = pending_spawn_.begin(); it != pending_spawn_.end();) {
      if (poses_.count(it->first)) {
        succeeded.push_back(std::move(it->second.done));
      } else if (now > it->second.deadline) {
        failed.push_back(std::move(it->second.done));
      } else {
        ++it;
        continue;
      }
      it = pending_spawn_.erase(it);
    }
  }
  for (auto & d : succeeded) {
    if (d) {d(true);}
  }
  for (auto & d : failed) {
    if (d) {d(false);}
  }
}

std::unordered_map<std::string, ModelPose> RosWorld::poses(double * stamp) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (stamp) {
    *stamp = stamp_;
  }
  return poses_;
}

std::optional<ModelPose> RosWorld::pose(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = poses_.find(name);
  if (it == poses_.end()) {
    return std::nullopt;
  }
  return it->second;
}

geometry_msgs::msg::TransformStamped RosWorld::toTransform(
  const std::string & frame, const std::string & child, const ModelPose & pose) const
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = node_.now();
  tf.header.frame_id = frame;
  tf.child_frame_id = child;
  tf.transform.translation.x = pose.position.x();
  tf.transform.translation.y = pose.position.y();
  tf.transform.translation.z = pose.position.z();
  tf.transform.rotation.w = pose.orientation.w();
  tf.transform.rotation.x = pose.orientation.x();
  tf.transform.rotation.y = pose.orientation.y();
  tf.transform.rotation.z = pose.orientation.z();
  return tf;
}

void RosWorld::spawn(
  const std::string & name, const std::string & model, const std::string & /*sdf*/,
  const ModelPose & pose, Done done)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_spawn_[name] = {Clock::now() + kSpawnTimeout, std::move(done)};
  }
  pub_spawn_->publish(toTransform(model, name, pose));
}

void RosWorld::remove(const std::string & name, Done done)
{
  std_msgs::msg::String msg;
  msg.data = name;
  pub_remove_->publish(msg);
  if (done) {
    done(true);
  }
}

void RosWorld::setPose(const std::string & name, const ModelPose & pose, bool /*skip_if_busy*/)
{
  pub_set_pose_->publish(toTransform(world_, name, pose));
}

void RosWorld::publishTwist(const std::string & ns, const Eigen::Vector3d & linear, double yaw_rate)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = node_.now();
  msg.header.frame_id = ns;
  msg.twist.linear.x = linear.x();
  msg.twist.linear.y = linear.y();
  msg.twist.linear.z = linear.z();
  msg.twist.angular.z = yaw_rate;
  pub_twist_->publish(msg);
}

void RosWorld::enable(const std::string & ns, bool on)
{
  std_msgs::msg::String msg;
  msg.data = ns + (on ? " 1" : " 0");
  pub_enable_->publish(msg);
}

}  // namespace aero_pursuit::sim
