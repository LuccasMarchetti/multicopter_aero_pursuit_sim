#pragma once

#include "aero_pursuit/sim/world.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace aero_pursuit::sim
{

/// World over plain ROS topics, with standard message types only, so a simulator without the
/// aero_pursuit interfaces (Isaac Sim, isaac/aero_world.py) can implement it:
///
///   <prefix>/poses     tf2_msgs/TFMessage            sim -> runner: every model, one message per
///                                                    sample (child_frame_id = model name)
///   <prefix>/spawn     geometry_msgs/TransformStamped  frame_id = catalog key, child = name
///   <prefix>/remove    std_msgs/String               name
///   <prefix>/set_pose  geometry_msgs/TransformStamped  child = name (teleport)
///   <prefix>/twist     geometry_msgs/TwistStamped    frame_id = name; linear in the heading
///                                                    frame, angular.z = yaw rate
///   <prefix>/enable    std_msgs/String               "<name> 1" | "<name> 0"
///
/// A spawn succeeds when the model shows up in <prefix>/poses (fails after kSpawnTimeout).
class RosWorld : public World
{
public:
  RosWorld(rclcpp::Node & node, std::string world, const std::string & prefix = "/sim");

  const std::string & world() const override {return world_;}

  bool subscribePoses() override;
  std::unordered_map<std::string, ModelPose> poses(double * stamp = nullptr) const override;
  std::optional<ModelPose> pose(const std::string & name) const override;

  void spawn(
    const std::string & name, const std::string & model, const std::string & sdf,
    const ModelPose & pose, Done done = {}) override;
  void remove(const std::string & name, Done done = {}) override;
  void setPose(const std::string & name, const ModelPose & pose, bool skip_if_busy = false) override;
  void publishTwist(const std::string & ns, const Eigen::Vector3d & linear, double yaw_rate) override;
  void enable(const std::string & ns, bool on) override;

  static constexpr std::chrono::seconds kSpawnTimeout{10};

private:
  using Clock = std::chrono::steady_clock;
  struct PendingSpawn
  {
    Clock::time_point deadline;
    Done done;
  };

  void onPoses(const tf2_msgs::msg::TFMessage & msg);
  geometry_msgs::msg::TransformStamped toTransform(
    const std::string & frame, const std::string & child, const ModelPose & pose) const;

  rclcpp::Node & node_;
  std::string world_;
  std::string prefix_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_poses_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_spawn_, pub_set_pose_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_remove_, pub_enable_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, ModelPose> poses_;
  double stamp_ = 0.0;
  std::unordered_map<std::string, PendingSpawn> pending_spawn_;
};

}  // namespace aero_pursuit::sim
