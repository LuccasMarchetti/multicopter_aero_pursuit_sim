#include "aero_pursuit/nodes/gcs_telemetry_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace aero_pursuit
{

GcsTelemetryNode::GcsTelemetryNode(const rclcpp::NodeOptions & options)
: Node("gcs_telemetry", options)
{
  world_frame_ = declare_parameter<std::string>("world_frame", "lawn");
  const double rate = declare_parameter<double>("rate_hz", 30.0);
  const auto topic = declare_parameter<std::string>("output_topic", "/gcs/poses");
  stale_timeout_ = declare_parameter<double>("stale_timeout", 2.0);

  buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, this, false);
  pub_ = create_publisher<tf2_msgs::msg::TFMessage>(topic, rclcpp::QoS(1).best_effort());

  // Wall clock: the rate for the operator does not follow the real-time factor.
  auto make_timer = [this](double hz) {
      timer_ = create_wall_timer(
        std::chrono::nanoseconds(static_cast<std::int64_t>(1e9 / std::clamp(hz, 1.0, 120.0))),
        [this] {publish();});
    };
  make_timer(rate);
  param_cb_ = add_on_set_parameters_callback(
    [this, make_timer](const std::vector<rclcpp::Parameter> & params) {
      for (const auto & p : params) {
        if (p.get_name() == "rate_hz") {make_timer(p.as_double());}
        if (p.get_name() == "world_frame") {world_frame_ = p.as_string();}
        if (p.get_name() == "stale_timeout") {stale_timeout_ = p.as_double();}
      }
      rcl_interfaces::msg::SetParametersResult r;
      r.successful = true;
      return r;
    });
}

void GcsTelemetryNode::publish()
{
  if (pub_->get_subscription_count() == 0) {return;}  // GCS fechada: custo zero
  tf2_msgs::msg::TFMessage out;
  const rclcpp::Time now = get_clock()->now();
  for (const auto & frame : buffer_->getAllFrameNames()) {
    if (frame == world_frame_) {continue;}
    try {
      auto tf = buffer_->lookupTransform(world_frame_, frame, tf2::TimePointZero);
      // The tf2 buffer keeps forever the last pose of a removed model (unloaded
      // swarm): without this the GCS would draw the drone frozen where it vanished.
      // Stamp 0 = static TF (always valid); stamp "from the future" = the clock went back.
      const rclcpp::Time stamp(tf.header.stamp, now.get_clock_type());
      if (stamp.nanoseconds() != 0 && stale_timeout_ > 0.0 &&
        (now - stamp).seconds() > stale_timeout_)
      {
        continue;
      }
      out.transforms.push_back(std::move(tf));
    } catch (const tf2::TransformException &) {
      // Frame outside the world tree (still): ignore.
    }
  }
  if (!out.transforms.empty()) {pub_->publish(out);}
}

}  // namespace aero_pursuit

RCLCPP_COMPONENTS_REGISTER_NODE(aero_pursuit::GcsTelemetryNode)
