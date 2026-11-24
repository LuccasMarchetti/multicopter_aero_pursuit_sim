#pragma once

#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <functional>
#include <mutex>
#include <optional>

namespace aero_pursuit
{

/// Low-level interface with PX4 in Offboard mode.
///
/// Sequence: WAIT_PX4 -> PRE_STREAM -> SET_OFFBOARD -> ARMING -> TAKEOFF -> HOLD <-> EXTERNAL
///  - HOLD: holds position (position setpoint) and waits for external commands.
///  - EXTERNAL: forwards external setpoints immediately. If they get older than
///    `setpoint_timeout`, goes back to HOLD at the current position (avoids drifting with the
///    last velocity command).
///  - Direct external setpoints of the dynamic MPC are forwarded with the matching Offboard
///    mode and a shorter timeout (`attitude_setpoint_timeout`):
///    atitude (VehicleAttitudeSetpoint), empuxo+torque normalizados
///    (VehicleThrust/TorqueSetpoint, `thrust_and_torque`) or motors
///    (ActuatorMotors, `direct_actuator`). On timeout it goes back to holding position with the
///    PX4 controllers.
///  - The state is published on ~/state (latched) for the scenario_runner.
///  - `px4_namespace` (e.g. /px4_1) and `target_system` allow one instance per
///    vehicle in SITL with several PX4s (swarm).
class OffboardControlNode : public rclcpp::Node
{
public:
  explicit OffboardControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum class State { kWaitForPx4, kPreStream, kSetOffboard, kArming, kTakeoff, kHold, kExternal };
  using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;
  using AttitudeSetpoint = px4_msgs::msg::VehicleAttitudeSetpoint;
  using VehicleCommandSrv = px4_msgs::srv::VehicleCommand;
  using ThrustSetpoint = px4_msgs::msg::VehicleThrustSetpoint;
  using TorqueSetpoint = px4_msgs::msg::VehicleTorqueSetpoint;
  using ActuatorMotors = px4_msgs::msg::ActuatorMotors;

  static const char * toString(State s);
  void transition(State next);

  void onTimer();
  bool acceptExternal();
  void onExternalSetpoint(const TrajectorySetpoint & msg);
  void onExternalAttitude(const AttitudeSetpoint & msg);
  void onExternalThrust(const ThrustSetpoint & msg);
  void onExternalTorque(const TorqueSetpoint & msg);
  void relayThrustTorque();
  void onExternalMotors(const ActuatorMotors & msg);
  void publishDirectMode(bool attitude, bool thrust_and_torque, bool direct_actuator);
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition & msg);

  void latchHold(float x, float y, float z, float yaw);
  void latchHoldAtCurrent();
  void publish(const TrajectorySetpoint & sp);
  void sendCommand(
    uint16_t command, float param1, float param2, std::function<void()> on_accepted);
  uint64_t timestampUs() const;

  // Configuration
  float takeoff_altitude_;
  float takeoff_tolerance_;
  double setpoint_timeout_;
  double attitude_timeout_;
  double command_retry_period_;
  int prestream_cycles_;
  uint8_t target_system_ = 1;

  // Estado
  State state_ = State::kWaitForPx4;
  int prestream_count_ = 0;
  bool command_pending_ = false;
  rclcpp::Time command_sent_time_;
  rclcpp::Time last_external_time_;
  bool external_attitude_ = false;  ///< Last external setpoint was direct (attitude/thrust/motors)
  /// Thrust and torque arrive on separate topics, in any order: the pair is only
  /// forwarded when both have the same timestamp_sample (same inner-loop cycle).
  std::optional<TorqueSetpoint> pending_torque_;
  std::optional<ThrustSetpoint> pending_thrust_;
  std::mutex pair_mutex_;
  TrajectorySetpoint hold_setpoint_;
  px4_msgs::msg::VehicleLocalPosition local_position_;
  bool have_local_position_ = false;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // Interfaces ROS
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_mode_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr pub_setpoint_;
  rclcpp::Publisher<AttitudeSetpoint>::SharedPtr pub_attitude_;
  rclcpp::Publisher<ThrustSetpoint>::SharedPtr pub_thrust_;
  rclcpp::Publisher<TorqueSetpoint>::SharedPtr pub_torque_;
  rclcpp::Publisher<ActuatorMotors>::SharedPtr pub_motors_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
  rclcpp::Subscription<TrajectorySetpoint>::SharedPtr sub_external_;
  rclcpp::Subscription<AttitudeSetpoint>::SharedPtr sub_external_attitude_;
  rclcpp::Subscription<ThrustSetpoint>::SharedPtr sub_external_thrust_;
  rclcpp::Subscription<TorqueSetpoint>::SharedPtr sub_external_torque_;
  rclcpp::Subscription<ActuatorMotors>::SharedPtr sub_external_motors_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_position_;
  /// PX4 arming state: a vehicle that landed (PX4 disarms it) or a new PX4 (new simulation) is
  /// armed and taken off again; before, the node did it once and then held a landed vehicle forever.
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr sub_status_;
  std::optional<bool> armed_;
  std::optional<rclcpp::Time> disarmed_since_;  ///< steady clock, while airborne states see DISARMED
  /// Flipped vehicle (tilt > 75 deg for 1 s): force disarm (PX4 kept a crashed x500 armed upside
  /// down, so the re-arm above never ran); the scenario runner puts it back upright.
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr sub_attitude_;
  double tilt_deg_ = 0.0;
  std::optional<rclcpp::Time> flipped_since_;
  rclcpp::Client<VehicleCommandSrv>::SharedPtr command_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::atomic<bool> acceleration_feedforward_{true};
  bool px4_receive_stamp_ = false;  ///< timestamp 0: PX4 stamps on arrival (lockstep sim with RTF != 1)
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

}  // namespace aero_pursuit
