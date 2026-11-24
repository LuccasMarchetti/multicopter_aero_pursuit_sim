#pragma once

#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>

namespace aero_pursuit
{

/// Converts the Gazebo model poses (Pose_V on /world/<w>/dynamic_pose/info)
/// into TFs `world_frame -> <model>` for visualization (Foxglove) and for the
/// omniscient mode. Only publishes the listed models, stamped with simulation time
/// (consistent with the ros_gz_bridge images) and at a limited rate.
class GazeboTfBridgeNode : public rclcpp::Node
{
public:
  explicit GazeboTfBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onPoses(const gz::msgs::Pose_V & msg);

  std::string world_frame_;
  std::unordered_set<std::string> models_;
  bool use_sim_stamp_ = true;
  int64_t min_period_ns_ = 0;
  int64_t last_publish_ns_ = 0;
  std::mutex mutex_;  ///< Callback runs on the gz-transport thread

  // Order matters (RAII): gz_node_ is destroyed first, stopping the callbacks
  // before the broadcaster they use.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  gz::transport::Node gz_node_;
};

}  // namespace aero_pursuit
