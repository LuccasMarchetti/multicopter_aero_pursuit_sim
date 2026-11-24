#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace aero_pursuit
{

/// Joystick teleoperation of the target drones (Twist in Gazebo, one topic per drone
/// in `target_vehicles`) or of the pursuer (velocity TrajectorySetpoint to the
/// offboard_control).
///
/// Commands in the vehicle heading frame (forward, left, up, yaw), with
/// the axis layout of `joy game_controller_node` (0 LX, 1 LY, 2 RX, 3 RY, 4 LT, 5 RT;
/// buttons 9 LB, 10 RB):
///   linear.x <- eixo 3 | linear.y <- eixo 2 | linear.z <- eixo 1 | yaw <- eixo 0
///  - LB / RB: previous / next vehicle (targets... -> pursuer -> ...). When switching,
///    the previous vehicle gets a null command; when taking over the pursuer, automatic
///    pursuit is paused (`~/enable` of pursuit_controller_node) and resumed on the way back.
///  - `~/select` (std_msgs/String): selects by name ("X3", "chaser", "next", "prev").
///  - With `idle_silence` (default) it only publishes while there is a command and once on release
///    (null command): neutral sticks do not fight scenarios, evader or buttons.
///  - LT / RT: decreases / increases the maximum linear speed scale,
///    proportionally to the trigger pressure (`speed_scale_rate` per second).
/// The active vehicle and the scale are published on `~/status` (latched).
class JoyTeleopNode : public rclcpp::Node
{
public:
  explicit JoyTeleopNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:

  struct Axis
  {
    int index;
    double max;
    double sign;  ///< -1 inverts the axis
  };

  struct Trigger
  {
    int index;
    double released;  ///< Released axis value
    double pressed;   ///< Fully pressed axis value
  };

  /// Command in the heading frame: forward, left, up [m/s] and yaw [rad/s].
  struct Command
  {
    double forward = 0.0;
    double left = 0.0;
    double up = 0.0;
    double yaw_rate = 0.0;
  };

  void loadParameters();
  void onJoy(const sensor_msgs::msg::Joy & msg);
  bool risingEdge(const sensor_msgs::msg::Joy & msg, int button) const;
  double triggerPressure(const sensor_msgs::msg::Joy & msg, const Trigger & t, int slot);
  void updateSpeedScale(const sensor_msgs::msg::Joy & msg);
  void onSelect(const std::string & name);
  void selectVehicle(int index);
  void cycleVehicle(int step);
  void send(int vehicle, const Command & cmd);
  void setPursuitEnabled(bool enabled);
  void publishStatus();
  int vehicleCount() const {return static_cast<int>(target_names_.size()) + 1;}
  bool isChaser(int vehicle) const {return vehicle == static_cast<int>(target_names_.size());}
  std::string vehicleName(int vehicle) const;

  // Configuration
  std::array<Axis, 4> axes_{};  ///< Ordem: linear x, linear y, linear z, angular z
  int button_prev_ = 9;
  int button_next_ = 10;
  Trigger trigger_slower_{4, 0.0, -1.0};
  Trigger trigger_faster_{5, 0.0, -1.0};
  double min_scale_ = 0.1;
  double max_scale_ = 2.0;
  double scale_rate_ = 0.5;
  bool pause_pursuit_ = true;
  bool idle_silence_ = true;
  std::vector<std::string> target_names_;

  // Estado
  int active_ = 0;
  bool was_idle_ = true;
  double speed_scale_ = 1.0;
  double reported_scale_ = 1.0;
  std::vector<int32_t> last_buttons_;
  std::array<bool, 2> trigger_seen_{false, false};
  double chaser_heading_ = NAN;  ///< Pursuer NED heading [rad]
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_joy_time_{0, 0, RCL_STEADY_TIME};

  // Interfaces ROS
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> pub_targets_;
  // target_interface "ros" (Isaac Sim world, sim/ros_world.hpp): TwistStamped on /sim/twist with
  // the drone name as frame_id, and "<name> 1" on /sim/enable when it is selected.
  bool ros_world_{false};
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_sim_twist_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_sim_enable_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_chaser_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_select_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_position_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr pursuit_enable_client_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_param_handle_;
};

}  // namespace aero_pursuit
