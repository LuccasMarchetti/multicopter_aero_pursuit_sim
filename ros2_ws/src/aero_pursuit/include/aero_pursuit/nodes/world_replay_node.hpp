#pragma once

#include "aero_pursuit/sim/world.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <array>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace aero_pursuit
{

/// Session replay in the simulator: rebuilds the recorded world from its poses instead of
/// simulating it again. The GCS plays the session bag with /tf and /gimbal/state remapped to
/// this node, which
///  - teleports every model to its recorded world pose (one batch per period: /set_pose_vector
///    in Gazebo, /sim/set_pose in Isaac Sim);
///  - drives the camera gimbal joints to the recorded angles (/gimbal/cmd/* bridged to Gazebo);
///  - re-publishes the recorded transforms below the models (camera, gimbal) on /tf.
/// The simulator then renders exactly what happened: pursuer camera video, free camera, 3D.
/// It runs with no pursuer nodes and no PX4 control, so nothing fights the teleports.
class WorldReplayNode : public rclcpp::Node
{
public:
  explicit WorldReplayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onTf(const tf2_msgs::msg::TFMessage & msg);
  void onGimbal(const sensor_msgs::msg::JointState & msg);
  void flush();

  std::string world_frame_;
  std::vector<std::string> exclude_;          ///< Model name prefixes left alone (free camera)
  std::unique_ptr<sim::World> world_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::mutex mutex_;
  std::map<std::string, sim::ModelPose> pending_;   ///< Latest recorded pose per model, unsent

  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 3> gimbal_cmd_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_gimbal_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace aero_pursuit
