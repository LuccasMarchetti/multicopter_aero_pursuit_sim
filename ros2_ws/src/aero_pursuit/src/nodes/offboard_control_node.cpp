#include "aero_pursuit/nodes/offboard_control_node.hpp"

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <cmath>

namespace aero_pursuit
{

using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleCommandAck;

namespace
{
bool finite3(const std::array<float, 3> & v)
{
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}
}  // namespace

OffboardControlNode::OffboardControlNode(const rclcpp::NodeOptions & options)
: Node("offboard_control", options),
  command_sent_time_(0, 0, RCL_STEADY_TIME),
  last_external_time_(0, 0, RCL_STEADY_TIME)
{
  // PX4 topic prefix: "" (instance 0) or /px4_<i> (SITL with several vehicles).
  const auto ns = declare_parameter<std::string>("px4_namespace", "");
  target_system_ = static_cast<uint8_t>(declare_parameter<int>("target_system", 1));
  const auto input_topic = declare_parameter<std::string>("input_topic", "/offboard/trajectory_setpoint");
  const auto attitude_input_topic =
    declare_parameter<std::string>("attitude_input_topic", "/offboard/attitude_setpoint");
  const auto attitude_output_topic =
    ns + declare_parameter<std::string>("attitude_output_topic", "/fmu/in/vehicle_attitude_setpoint_v1");
  const auto thrust_input_topic =
    declare_parameter<std::string>("thrust_input_topic", "/offboard/thrust_setpoint");
  const auto torque_input_topic =
    declare_parameter<std::string>("torque_input_topic", "/offboard/torque_setpoint");
  const auto motors_input_topic =
    declare_parameter<std::string>("motors_input_topic", "/offboard/actuator_motors");
  const auto local_position_topic =
    ns + declare_parameter<std::string>("local_position_topic", "/fmu/out/vehicle_local_position_v1");
  const auto status_topic =
    ns + declare_parameter<std::string>("vehicle_status_topic", "/fmu/out/vehicle_status_v4");
  takeoff_altitude_ = static_cast<float>(declare_parameter<double>("takeoff_altitude", 5.0));
  takeoff_tolerance_ = static_cast<float>(declare_parameter<double>("takeoff_tolerance", 0.5));
  setpoint_timeout_ = declare_parameter<double>("setpoint_timeout", 0.5);
  attitude_timeout_ = declare_parameter<double>("attitude_setpoint_timeout", 0.25);
  command_retry_period_ = declare_parameter<double>("command_retry_period", 1.0);
  prestream_cycles_ = static_cast<int>(declare_parameter<int>("prestream_cycles", 10));
  const double rate_hz = declare_parameter<double>("rate_hz", 20.0);
  // false: TrajectorySetpoint.acceleration = NaN, so PX4 drops the acceleration
  // feedforward and closes only its velocity PID (kappa = 0, "no feedforward" variant).
  acceleration_feedforward_ = declare_parameter<bool>("acceleration_feedforward", true);
  // Lockstep simulators whose real-time factor is not 1 (Isaac Sim + Pegasus): the uXRCE timesync
  // offset lags the drift between PX4 and agent clocks and the offboard stream looks stale.
  // With timestamp 0, PX4 stamps each message on arrival (uxrce_dds_client).
  px4_receive_stamp_ = declare_parameter<bool>("px4_receive_stamp", false);
  param_callback_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      for (const auto & p : params) {
        if (p.get_name() == "acceleration_feedforward") {
          acceleration_feedforward_ = p.as_bool();
          RCLCPP_INFO(get_logger(), "acceleration_feedforward = %s",
            acceleration_feedforward_ ? "true" : "false");
        }
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    });

  pub_mode_ = create_publisher<px4_msgs::msg::OffboardControlMode>(ns + "/fmu/in/offboard_control_mode", 10);
  pub_setpoint_ = create_publisher<TrajectorySetpoint>(ns + "/fmu/in/trajectory_setpoint", 10);
  pub_attitude_ = create_publisher<AttitudeSetpoint>(attitude_output_topic, 10);
  pub_thrust_ = create_publisher<ThrustSetpoint>(ns + "/fmu/in/vehicle_thrust_setpoint", 10);
  pub_torque_ = create_publisher<TorqueSetpoint>(ns + "/fmu/in/vehicle_torque_setpoint", 10);
  pub_motors_ = create_publisher<ActuatorMotors>(ns + "/fmu/in/actuator_motors", 10);
  pub_state_ = create_publisher<std_msgs::msg::String>("~/state", rclcpp::QoS(1).reliable().transient_local());
  command_client_ = create_client<VehicleCommandSrv>(ns + "/fmu/vehicle_command");

  sub_external_ = create_subscription<TrajectorySetpoint>(
    input_topic, 10, [this](const TrajectorySetpoint & msg) {onExternalSetpoint(msg);});
  sub_external_attitude_ = create_subscription<AttitudeSetpoint>(
    attitude_input_topic, 10, [this](const AttitudeSetpoint & msg) {onExternalAttitude(msg);});
  sub_external_torque_ = create_subscription<TorqueSetpoint>(
    torque_input_topic, 10, [this](const TorqueSetpoint & msg) {onExternalTorque(msg);});
  sub_external_thrust_ = create_subscription<ThrustSetpoint>(
    thrust_input_topic, 10, [this](const ThrustSetpoint & msg) {onExternalThrust(msg);});
  sub_external_motors_ = create_subscription<ActuatorMotors>(
    motors_input_topic, 10, [this](const ActuatorMotors & msg) {onExternalMotors(msg);});
  sub_local_position_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    local_position_topic, rclcpp::SensorDataQoS(),
    [this](const px4_msgs::msg::VehicleLocalPosition & msg) {onLocalPosition(msg);});
  sub_attitude_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
    ns + "/fmu/out/vehicle_attitude", rclcpp::SensorDataQoS(),
    [this](const px4_msgs::msg::VehicleAttitude & m) {
      // Angle between body z and world z: cos = 1 - 2 (qx² + qy²)
      const double c = 1.0 - 2.0 * (static_cast<double>(m.q[1]) * m.q[1] + static_cast<double>(m.q[2]) * m.q[2]);
      tilt_deg_ = std::acos(std::clamp(c, -1.0, 1.0)) * 180.0 / M_PI;
    });
  sub_status_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    status_topic, rclcpp::SensorDataQoS(), [this](const px4_msgs::msg::VehicleStatus & msg) {
      armed_ = msg.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    });

  // Wall timer: the Offboard heartbeat must not depend on /clock.
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz), [this]() {onTimer();});
  std_msgs::msg::String st;
  st.data = toString(state_);
  pub_state_->publish(st);
}

const char * OffboardControlNode::toString(State s)
{
  switch (s) {
    case State::kWaitForPx4: return "WAIT_PX4";
    case State::kPreStream: return "PRE_STREAM";
    case State::kSetOffboard: return "SET_OFFBOARD";
    case State::kArming: return "ARMING";
    case State::kTakeoff: return "TAKEOFF";
    case State::kHold: return "HOLD";
    case State::kExternal: return "EXTERNAL";
  }
  return "UNKNOWN";
}

void OffboardControlNode::transition(State next)
{
  if (next != state_) {
    RCLCPP_INFO(get_logger(), "Offboard: %s -> %s", toString(state_), toString(next));
    state_ = next;
    std_msgs::msg::String st;
    st.data = toString(state_);
    pub_state_->publish(st);
  }
}

uint64_t OffboardControlNode::timestampUs() const
{
  if (px4_receive_stamp_) {
    return 0;
  }
  return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
}

void OffboardControlNode::latchHold(float x, float y, float z, float yaw)
{
  hold_setpoint_ = TrajectorySetpoint{};
  hold_setpoint_.position = {x, y, z};
  hold_setpoint_.velocity = {NAN, NAN, NAN};
  hold_setpoint_.acceleration = {NAN, NAN, NAN};
  hold_setpoint_.jerk = {NAN, NAN, NAN};
  hold_setpoint_.yaw = yaw;
  hold_setpoint_.yawspeed = NAN;
}

void OffboardControlNode::latchHoldAtCurrent()
{
  latchHold(local_position_.x, local_position_.y, local_position_.z, local_position_.heading);
}

void OffboardControlNode::publish(const TrajectorySetpoint & sp)
{
  // The mode derives from the setpoint: position if there is a position, otherwise velocity/acceleration.
  px4_msgs::msg::OffboardControlMode mode{};
  mode.position = finite3(sp.position);
  mode.velocity = !mode.position && finite3(sp.velocity);
  mode.acceleration = !mode.position && !mode.velocity && finite3(sp.acceleration);
  mode.timestamp = timestampUs();
  pub_mode_->publish(mode);

  TrajectorySetpoint out = sp;
  out.timestamp = mode.timestamp;
  pub_setpoint_->publish(out);
}

void OffboardControlNode::sendCommand(
  uint16_t command, float param1, float param2, std::function<void()> on_accepted)
{
  const rclcpp::Time now = steady_clock_.now();
  if (command_pending_ && (now - command_sent_time_).seconds() < command_retry_period_) {
    return;
  }

  auto request = std::make_shared<VehicleCommandSrv::Request>();
  auto & cmd = request->request;
  cmd.command = command;
  cmd.param1 = param1;
  cmd.param2 = param2;
  cmd.target_system = target_system_;
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 1;
  cmd.from_external = true;
  cmd.timestamp = timestampUs();

  // Requests without a reply (PX4 restarted, XRCE agent down) would stay forever
  // in the client's pending map at every new attempt.
  command_client_->prune_pending_requests();
  command_pending_ = true;
  command_sent_time_ = now;
  command_client_->async_send_request(
    request,
    [this, command, on_accepted = std::move(on_accepted)](
      rclcpp::Client<VehicleCommandSrv>::SharedFuture future) {
      command_pending_ = false;
      const auto & reply = future.get()->reply;
      if (reply.result == VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
        on_accepted();
      } else {
        RCLCPP_WARN(get_logger(), "Command %u refused (result=%u). Retrying in %.1f s.",
          command, reply.result, command_retry_period_);
      }
    });
}

void OffboardControlNode::onLocalPosition(const px4_msgs::msg::VehicleLocalPosition & msg)
{
  local_position_ = msg;
  have_local_position_ = msg.xy_valid && msg.z_valid;
}

bool OffboardControlNode::acceptExternal()
{
  if (state_ != State::kHold && state_ != State::kExternal) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Setpoint externo ignorado no estado %s.", toString(state_));
    return false;
  }
  last_external_time_ = steady_clock_.now();
  transition(State::kExternal);
  return true;
}

void OffboardControlNode::onExternalSetpoint(const TrajectorySetpoint & msg)
{
  if (acceptExternal()) {
    external_attitude_ = false;
    if (!acceleration_feedforward_ && finite3(msg.velocity)) {
      TrajectorySetpoint sp = msg;
      sp.acceleration = {NAN, NAN, NAN};
      publish(sp);
      return;
    }
    publish(msg);  // Immediate forwarding: no extra timer latency
  }
}

void OffboardControlNode::onExternalAttitude(const AttitudeSetpoint & msg)
{
  const bool finite = std::isfinite(msg.q_d[0]) && std::isfinite(msg.q_d[1]) &&
    std::isfinite(msg.q_d[2]) && std::isfinite(msg.q_d[3]) && std::isfinite(msg.thrust_body[2]);
  if (!finite) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid attitude setpoint (NaN).");
    return;
  }
  if (!acceptExternal()) {
    return;
  }
  external_attitude_ = true;
  publishDirectMode(true, false, false);
  AttitudeSetpoint out = msg;
  out.timestamp = timestampUs();
  pub_attitude_->publish(out);
}

void OffboardControlNode::publishDirectMode(bool attitude, bool thrust_and_torque, bool direct_actuator)
{
  px4_msgs::msg::OffboardControlMode mode{};
  mode.attitude = attitude;
  mode.thrust_and_torque = thrust_and_torque;
  mode.direct_actuator = direct_actuator;
  mode.timestamp = timestampUs();
  pub_mode_->publish(mode);
}

void OffboardControlNode::onExternalTorque(const TorqueSetpoint & msg)
{
  std::lock_guard<std::mutex> lock(pair_mutex_);
  pending_torque_ = msg;
  relayThrustTorque();
}

void OffboardControlNode::onExternalThrust(const ThrustSetpoint & msg)
{
  std::lock_guard<std::mutex> lock(pair_mutex_);
  pending_thrust_ = msg;
  relayThrustTorque();
}

void OffboardControlNode::relayThrustTorque()
{
  if (!pending_torque_ || !pending_thrust_ ||
    pending_torque_->timestamp_sample != pending_thrust_->timestamp_sample)
  {
    return;  // Waits for the other member of the pair (or a newer pair)
  }
  const auto & t = pending_torque_->xyz;
  const auto & f = pending_thrust_->xyz;
  const bool finite = std::isfinite(f[0]) && std::isfinite(f[1]) && std::isfinite(f[2]) &&
    std::isfinite(t[0]) && std::isfinite(t[1]) && std::isfinite(t[2]);
  if (!finite) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid thrust/torque setpoint (NaN).");
  } else if (acceptExternal()) {
    external_attitude_ = true;
    publishDirectMode(false, true, false);
    const uint64_t ts = timestampUs();
    TorqueSetpoint torque = *pending_torque_;
    torque.timestamp = ts;
    pub_torque_->publish(torque);
    ThrustSetpoint thrust = *pending_thrust_;
    thrust.timestamp = ts;
    pub_thrust_->publish(thrust);
  }
  pending_torque_.reset();
  pending_thrust_.reset();
}

void OffboardControlNode::onExternalMotors(const ActuatorMotors & msg)
{
  for (int i = 0; i < 4; ++i) {
    const float u = msg.control[static_cast<std::size_t>(i)];
    if (!std::isfinite(u) || u < 0.0f || u > 1.0f) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid motor command: %f", u);
      return;
    }
  }
  if (!acceptExternal()) {
    return;
  }
  external_attitude_ = true;
  publishDirectMode(false, false, true);
  ActuatorMotors out = msg;
  out.timestamp = timestampUs();
  pub_motors_->publish(out);
}

void OffboardControlNode::onTimer()
{
  // Airborne states with the PX4 disarmed (landed, or a new PX4): arm and take off again. Debounced:
  // right after the arm command the status may still read DISARMED.
  constexpr double kDisarmedFor = 2.0;  // [s]
  const bool airborne_state = state_ == State::kTakeoff || state_ == State::kHold ||
    state_ == State::kExternal;
  if (airborne_state && armed_.has_value() && !*armed_) {
    if (!disarmed_since_) {
      disarmed_since_ = steady_clock_.now();
    } else if ((steady_clock_.now() - *disarmed_since_).seconds() > kDisarmedFor) {
      RCLCPP_WARN(get_logger(), "PX4 disarmed (landed?): arming and taking off again");
      disarmed_since_.reset();
      latchHoldAtCurrent();
      prestream_count_ = 0;
      transition(State::kPreStream);
    }
  } else {
    disarmed_since_.reset();
  }
  constexpr double kFlippedDeg = 75.0, kFlippedFor = 1.0;
  if (armed_.value_or(false) && tilt_deg_ > kFlippedDeg) {
    if (!flipped_since_) {
      flipped_since_ = steady_clock_.now();
    } else if ((steady_clock_.now() - *flipped_since_).seconds() > kFlippedFor) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "Vehicle flipped (tilt %.0f deg): forcing disarm", tilt_deg_);
      sendCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f, 21196.0f, []() {});
    }
  } else {
    flipped_since_.reset();
  }

  switch (state_) {
    case State::kWaitForPx4:
      if (command_client_->service_is_ready() && have_local_position_) {
        latchHoldAtCurrent();
        prestream_count_ = 0;
        transition(State::kPreStream);
      }
      return;

    case State::kPreStream:
      // PX4 requires a setpoint stream before accepting Offboard mode.
      if (++prestream_count_ >= prestream_cycles_) {
        transition(State::kSetOffboard);
      }
      break;

    case State::kSetOffboard:
      sendCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f,
        [this]() {transition(State::kArming);});
      break;

    case State::kArming:
      sendCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f,
        [this]() {
          latchHold(local_position_.x, local_position_.y, -takeoff_altitude_, local_position_.heading);
          transition(State::kTakeoff);
        });
      break;

    case State::kTakeoff:
      if (std::abs(local_position_.z + takeoff_altitude_) < takeoff_tolerance_) {
        RCLCPP_INFO(get_logger(), "Takeoff done (%.1f m). Waiting for setpoints.",
          takeoff_altitude_);
        transition(State::kHold);
      }
      break;

    case State::kHold:
      break;

    case State::kExternal: {
      // An old attitude is more dangerous than an old velocity (tilt kept): shorter timeout.
      const double timeout = external_attitude_ ? attitude_timeout_ : setpoint_timeout_;
      if ((steady_clock_.now() - last_external_time_).seconds() > timeout) {
        RCLCPP_WARN(get_logger(), "No external setpoints for %.2f s: holding position.", timeout);
        latchHoldAtCurrent();
        transition(State::kHold);
        break;
      }
      return;  // External setpoints already published in the callback
    }
  }
  publish(hold_setpoint_);
}

}  // namespace aero_pursuit

RCLCPP_COMPONENTS_REGISTER_NODE(aero_pursuit::OffboardControlNode)
