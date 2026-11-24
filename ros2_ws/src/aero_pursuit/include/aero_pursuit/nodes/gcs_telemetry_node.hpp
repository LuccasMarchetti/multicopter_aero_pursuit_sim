#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace aero_pursuit
{

/// Aggregates the TF tree for the GCS: every period it publishes, in a single TFMessage,
/// the pose of every frame in the world frame (header.stamp = TF instant).
/// Cheaper than having the GCS (Python) subscribe to /tf and /clock, which reach hundreds of
/// messages per second; the C++ tf2 buffer also clears itself when the simulation
/// time goes back (world restart/reset).
class GcsTelemetryNode : public rclcpp::Node
{
public:
  explicit GcsTelemetryNode(const rclcpp::NodeOptions & options);

private:
  void publish();

  std::string world_frame_;
  double stale_timeout_ = 2.0;  ///< Frames without a new TF for longer than this leave the GCS [s]
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace aero_pursuit
