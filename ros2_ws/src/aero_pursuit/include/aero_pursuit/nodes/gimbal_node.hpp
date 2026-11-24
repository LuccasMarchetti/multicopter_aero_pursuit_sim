#pragma once

#include "aero_pursuit/msg/pursuit_status.hpp"
#include "aero_pursuit/msg/target_bearing.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <optional>
#include <string>

namespace aero_pursuit
{

/// Pitch-only camera gimbal of the pursuer (x500_gimbal_depth), the same in Gazebo and in Isaac
/// Sim. A "fixed" camera is the gimbal held at `pitch_deg` (auto_pitch false, the default).
///
/// Interface (ROS only; the simulator side is the gz bridge or isaac/aero_world.py):
///   /gimbal/joint_states   (JointState, measured)  -> TF gimbal_mount -> camera_link (pitch)
///   /gimbal/joint_command  (JointState, name gimbal_pitch_joint)   Isaac (aero_world.py Gimbal)
///   /gimbal/cmd/pitch      (Float64)                                 Gazebo (ros_gz_bridge)
///
/// Command (`auto_pitch`, the GCS check box): true = geometric law control::gimbalPitchToward
/// (target estimate, or the bounding-box direction without depth, seen from the body); false =
/// the fixed angle `pitch_deg`. Rate limited (`max_rate_deg`) and clamped to the joint limits.
/// Target briefly missing (COASTING/APPROACH without a fresh estimate): holds the last command.
/// Target really lost or pursuit off (/pursuit/status LOST, IDLE, DISABLED): back to `pitch_deg`.
class GimbalNode : public rclcpp::Node
{
public:
  explicit GimbalNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onJointStates(const sensor_msgs::msg::JointState & msg);
  void onOdometry(const px4_msgs::msg::VehicleOdometry & msg);
  void step();
  /// Desired pitch from the latest target direction, or nothing (no recent target / attitude).
  std::optional<double> lawPitch();

  std::string mount_frame_, camera_frame_, joint_name_;
  Eigen::Vector3d t_mount_;
  double dt_ = 0.02;
  double command_ = 0.0;   ///< last command [rad]
  bool have_command_ = false;
  rclcpp::Time last_tf_stamp_{0, 0, RCL_ROS_TIME};

  std::optional<std::pair<Eigen::Vector3d, rclcpp::Time>> target_;   ///< p_rel ENU (estimate)
  std::optional<std::pair<Eigen::Vector3d, rclcpp::Time>> bearing_;  ///< unit direction ENU (box)
  std::optional<std::pair<Eigen::Quaterniond, rclcpp::Time>> attitude_;  ///< FLU -> ENU
  std::optional<double> measured_;   ///< measured pitch [rad]
  uint8_t pursuit_mode_ = msg::PursuitStatus::IDLE;  ///< supervisor mode (/pursuit/status)

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joints_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_target_;
  rclcpp::Subscription<msg::TargetBearing>::SharedPtr sub_bearing_;
  rclcpp::Subscription<msg::PursuitStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odometry_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_command_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_gz_pitch_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace aero_pursuit
