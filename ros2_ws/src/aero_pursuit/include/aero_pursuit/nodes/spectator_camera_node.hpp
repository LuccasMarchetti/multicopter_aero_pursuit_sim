#pragma once

#include "aero_pursuit/msg/spectator_command.hpp"
#include "aero_pursuit/sim/world.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <mutex>
#include <string>

namespace aero_pursuit
{

/// Free camera of the simulator for the GCS ("Gazebo"/"Isaac" view), on the world interface
/// (world_backend gz: Gazebo; ros: the Isaac Sim world, where aero_world.py renders the
/// spectator_camera model as a camera on /spectator/image). Spawns the
/// spectator_camera model (camera sensor on /spectator/image) and moves it with set_pose
/// according to /spectator/command — free pose (mouse/keyboard flight in the browser),
/// follow, orbit or look at a vehicle. The applied pose goes out on the TF
/// `<world> -> spectator_camera` (the GCS starts from it when going back to free mode).
class SpectatorCameraNode : public rclcpp::Node
{
public:
  explicit SpectatorCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void tick();
  void ensureModel();

  std::string world_frame_;
  std::string model_name_;
  std::string model_sdf_;
  double smoothing_ = 0.25;   ///< Filter time constant in the modes that follow a target [s]

  std::unique_ptr<sim::World> gz_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::mutex mutex_;
  msg::SpectatorCommand command_;
  bool command_changed_ = true;
  Eigen::Vector3d position_ = Eigen::Vector3d(-15.0, -15.0, 12.0);
  Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
  double orbit_azimuth_ = 0.0;
  rclcpp::Time last_tick_;
  rclcpp::Time last_sent_;
  rclcpp::Time last_spawn_;

  rclcpp::Subscription<msg::SpectatorCommand>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace aero_pursuit
