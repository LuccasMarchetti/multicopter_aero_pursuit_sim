#include "aero_pursuit/nodes/joy_teleop_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <cctype>
#include <cstdio>

namespace aero_pursuit
{

JoyTeleopNode::JoyTeleopNode(const rclcpp::NodeOptions & options)
: Node("drone_joy_teleop", options)
{
  target_names_ = declare_parameter<std::vector<std::string>>(
    "target_vehicles", std::vector<std::string>{"X4", "X3", "Tiltrotor", "X5"});
  const auto target_pattern =
    declare_parameter<std::string>("target_topic_pattern", "/{name}/gazebo/command/twist");
  const auto chaser_topic =
    declare_parameter<std::string>("chaser_topic", "/offboard/trajectory_setpoint");
  const auto chaser_position_topic = declare_parameter<std::string>(
    "chaser_local_position_topic", "/fmu/out/vehicle_local_position_v1");
  const auto enable_service = declare_parameter<std::string>(
    "pursuit_enable_service", "/pursuit_controller_node/enable");

  // Symmetric limits [m/s, rad/s], mapping and axis signs
  declare_parameter<double>("max_linear_x", 5.0);
  declare_parameter<double>("max_linear_y", 5.0);
  declare_parameter<double>("max_linear_z", 5.0);
  declare_parameter<double>("max_angular_z", 3.0);
  declare_parameter<int>("axis_linear_x", 3);   // Right stick - vertical (forward/back)
  declare_parameter<int>("axis_linear_y", 2);   // Right stick - horizontal (left/right)
  declare_parameter<int>("axis_linear_z", 1);   // Left stick - vertical (up/down)
  declare_parameter<int>("axis_angular_z", 0);  // Left stick - horizontal (yaw)
  declare_parameter<bool>("invert_linear_x", false);
  declare_parameter<bool>("invert_linear_y", false);
  declare_parameter<bool>("invert_linear_z", false);
  declare_parameter<bool>("invert_angular_z", false);

  // Vehicle switch (LB/RB) and speed scale (LT/RT)
  declare_parameter<int>("button_previous_vehicle", 9);  // LB (game_controller_node)
  declare_parameter<int>("button_next_vehicle", 10);     // RB
  declare_parameter<int>("axis_speed_down", 4);          // LT
  declare_parameter<int>("axis_speed_up", 5);            // RT
  declare_parameter<double>("trigger_released_value", 0.0);
  declare_parameter<double>("trigger_pressed_value", -1.0);
  declare_parameter<double>("speed_scale_min", 0.1);
  declare_parameter<double>("speed_scale_max", 2.0);
  declare_parameter<double>("speed_scale_rate", 0.5);    // [1/s] with the trigger pressed
  declare_parameter<bool>("pause_pursuit_on_chaser", true);
  declare_parameter<bool>("idle_silence", true);
  speed_scale_ = std::max(0.0, declare_parameter<double>("speed_scale_initial", 1.0));
  loadParameters();
  speed_scale_ = std::clamp(speed_scale_, min_scale_, max_scale_);
  reported_scale_ = speed_scale_;

  ros_world_ = declare_parameter<std::string>("target_interface", "gz") == "ros";
  if (ros_world_) {
    const auto reliable = rclcpp::QoS(50).reliable();
    pub_sim_twist_ = create_publisher<geometry_msgs::msg::TwistStamped>("/sim/twist", reliable);
    pub_sim_enable_ = create_publisher<std_msgs::msg::String>("/sim/enable", reliable);
  }
  for (const auto & name : target_names_) {
    std::string topic = target_pattern;
    const auto pos = topic.find("{name}");
    if (pos != std::string::npos) {
      topic.replace(pos, 6, name);
    }
    pub_targets_.push_back(create_publisher<geometry_msgs::msg::Twist>(topic, 10));
  }
  pub_chaser_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(chaser_topic, 10);
  pub_status_ = create_publisher<std_msgs::msg::String>(
    "~/status", rclcpp::QoS(1).transient_local());
  pursuit_enable_client_ = create_client<std_srvs::srv::SetBool>(enable_service);

  sub_joy_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, [this](const sensor_msgs::msg::Joy & msg) {onJoy(msg);});
  sub_select_ = create_subscription<std_msgs::msg::String>(
    "~/select", 10, [this](const std_msgs::msg::String & msg) {onSelect(msg.data);});
  sub_local_position_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    chaser_position_topic, rclcpp::SensorDataQoS(),
    [this](const px4_msgs::msg::VehicleLocalPosition & msg) {
      chaser_heading_ = std::isfinite(msg.heading) ? static_cast<double>(msg.heading) : NAN;
    });
  post_param_handle_ = add_post_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &) {loadParameters();});

  publishStatus();
  RCLCPP_INFO(get_logger(), "Teleop started: %zu targets + pursuer on '%s' (LB/RB or ~/select switches).",
    target_names_.size(), chaser_topic.c_str());
}

std::string JoyTeleopNode::vehicleName(int vehicle) const
{
  return isChaser(vehicle) ? "chaser" : target_names_[static_cast<std::size_t>(vehicle)];
}

void JoyTeleopNode::loadParameters()
{
  auto axis = [this](const std::string & name) {
      return Axis{
        static_cast<int>(get_parameter("axis_" + name).as_int()),
        get_parameter("max_" + name).as_double(),
        get_parameter("invert_" + name).as_bool() ? -1.0 : 1.0};
    };
  axes_ = {axis("linear_x"), axis("linear_y"), axis("linear_z"), axis("angular_z")};

  button_prev_ = static_cast<int>(get_parameter("button_previous_vehicle").as_int());
  button_next_ = static_cast<int>(get_parameter("button_next_vehicle").as_int());
  const double released = get_parameter("trigger_released_value").as_double();
  const double pressed = get_parameter("trigger_pressed_value").as_double();
  trigger_slower_ = {static_cast<int>(get_parameter("axis_speed_down").as_int()), released, pressed};
  trigger_faster_ = {static_cast<int>(get_parameter("axis_speed_up").as_int()), released, pressed};
  min_scale_ = std::max(0.0, get_parameter("speed_scale_min").as_double());
  max_scale_ = std::max(min_scale_, get_parameter("speed_scale_max").as_double());
  scale_rate_ = std::max(0.0, get_parameter("speed_scale_rate").as_double());
  pause_pursuit_ = get_parameter("pause_pursuit_on_chaser").as_bool();
  idle_silence_ = get_parameter("idle_silence").as_bool();
  speed_scale_ = std::clamp(speed_scale_, min_scale_, max_scale_);
}

bool JoyTeleopNode::risingEdge(const sensor_msgs::msg::Joy & msg, int button) const
{
  if (button < 0 || static_cast<size_t>(button) >= msg.buttons.size()) {
    return false;
  }
  const bool was = static_cast<size_t>(button) < last_buttons_.size() && last_buttons_[button] != 0;
  return msg.buttons[button] != 0 && !was;
}

double JoyTeleopNode::triggerPressure(
  const sensor_msgs::msg::Joy & msg, const Trigger & t, int slot)
{
  if (t.index < 0 || static_cast<size_t>(t.index) >= msg.axes.size() || t.pressed == t.released) {
    return 0.0;
  }
  const double value = msg.axes[t.index];
  // joy_node reports 0 on the triggers until the first motion, even when the
  // "released" value is +1: only trust the axis after seeing it non-zero.
  if (!trigger_seen_[slot]) {
    if (value == 0.0 && t.released != 0.0) {
      return 0.0;
    }
    trigger_seen_[slot] = true;
  }
  return std::clamp((value - t.released) / (t.pressed - t.released), 0.0, 1.0);
}

void JoyTeleopNode::updateSpeedScale(const sensor_msgs::msg::Joy & msg)
{
  const rclcpp::Time now = steady_clock_.now();
  const double dt = last_joy_time_.nanoseconds() == 0 ?
    0.0 : std::clamp((now - last_joy_time_).seconds(), 0.0, 0.2);
  last_joy_time_ = now;

  const double push = triggerPressure(msg, trigger_faster_, 1) - triggerPressure(msg, trigger_slower_, 0);
  speed_scale_ = std::clamp(speed_scale_ + scale_rate_ * push * dt, min_scale_, max_scale_);
  const bool at_limit = speed_scale_ == min_scale_ || speed_scale_ == max_scale_;
  if (std::abs(speed_scale_ - reported_scale_) >= 0.05 ||
    (at_limit && speed_scale_ != reported_scale_))
  {
    reported_scale_ = speed_scale_;
    RCLCPP_INFO(get_logger(), "Speed scale: %.2f", speed_scale_);
    publishStatus();
  }
}

void JoyTeleopNode::cycleVehicle(int step)
{
  const int n = vehicleCount();
  selectVehicle(((active_ + step) % n + n) % n);
}

void JoyTeleopNode::onSelect(const std::string & name)
{
  auto lower = [](std::string t) {
      std::transform(t.begin(), t.end(), t.begin(), [](unsigned char c) {return std::tolower(c);});
      return t;
    };
  const std::string wanted = lower(name);
  if (wanted == "next") {
    cycleVehicle(+1);
    return;
  }
  if (wanted == "prev" || wanted == "previous") {
    cycleVehicle(-1);
    return;
  }
  for (int i = 0; i < vehicleCount(); ++i) {
    if (lower(vehicleName(i)) == wanted || (isChaser(i) && (wanted == "pursuer" || wanted == "perseguidor"))) {
      selectVehicle(i);
      return;
    }
  }
  RCLCPP_WARN(get_logger(), "Unknown vehicle: '%s'", name.c_str());
}

void JoyTeleopNode::selectVehicle(int index)
{
  const int previous = active_;
  active_ = index;
  if (active_ == previous) {
    publishStatus();
    return;
  }
  // Gazebo's MulticopterVelocityControl keeps the last Twist: zero the previous one.
  send(previous, Command{});
  if (ros_world_ && !isChaser(active_)) {
    std_msgs::msg::String on;
    on.data = vehicleName(active_) + " 1";   // Isaac targets start disabled unless spawned as base drones
    pub_sim_enable_->publish(on);
  }
  if (pause_pursuit_ && (isChaser(active_) || isChaser(previous))) {
    setPursuitEnabled(!isChaser(active_));
  }
  was_idle_ = true;
  RCLCPP_INFO(get_logger(), "Joystick controlando: %s", vehicleName(active_).c_str());
  publishStatus();
}

void JoyTeleopNode::setPursuitEnabled(bool enabled)
{
  if (!pursuit_enable_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service '%s' unavailable: pursuit was not %s.",
      pursuit_enable_client_->get_service_name(), enabled ? "retomada" : "pausada");
    return;
  }
  pursuit_enable_client_->prune_pending_requests();
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enabled;
  pursuit_enable_client_->async_send_request(
    request, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      RCLCPP_INFO(get_logger(), "%s", future.get()->message.c_str());
    });
}

void JoyTeleopNode::send(int vehicle, const Command & cmd)
{
  if (!isChaser(vehicle)) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = cmd.forward;
    twist.linear.y = cmd.left;
    twist.linear.z = cmd.up;
    twist.angular.z = cmd.yaw_rate;
    if (ros_world_) {
      geometry_msgs::msg::TwistStamped stamped;
      stamped.header.stamp = now();
      stamped.header.frame_id = vehicleName(vehicle);
      stamped.twist = twist;
      pub_sim_twist_->publish(stamped);
      return;
    }
    pub_targets_[static_cast<std::size_t>(vehicle)]->publish(twist);
    return;
  }

  if (!std::isfinite(chaser_heading_)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "No pursuer heading (vehicle_local_position): command not sent.");
    return;
  }
  // Heading frame (forward, left, up) -> NED; FLU yaw (counter-clockwise) -> NED.
  const double c = std::cos(chaser_heading_);
  const double s = std::sin(chaser_heading_);
  px4_msgs::msg::TrajectorySetpoint sp;
  sp.timestamp = static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  sp.position = {NAN, NAN, NAN};
  sp.velocity = {
    static_cast<float>(c * cmd.forward + s * cmd.left),
    static_cast<float>(s * cmd.forward - c * cmd.left),
    static_cast<float>(-cmd.up)};
  sp.acceleration = {NAN, NAN, NAN};
  sp.jerk = {NAN, NAN, NAN};
  sp.yaw = NAN;
  sp.yawspeed = static_cast<float>(-cmd.yaw_rate);
  pub_chaser_->publish(sp);
}

void JoyTeleopNode::publishStatus()
{
  std_msgs::msg::String msg;
  char text[64];
  std::snprintf(text, sizeof(text), "%s (escala %.2f)", vehicleName(active_).c_str(), speed_scale_);
  msg.data = text;
  pub_status_->publish(msg);
}

void JoyTeleopNode::onJoy(const sensor_msgs::msg::Joy & msg)
{
  if (risingEdge(msg, button_prev_)) {
    cycleVehicle(-1);
  } else if (risingEdge(msg, button_next_)) {
    cycleVehicle(+1);
  }
  last_buttons_ = msg.buttons;
  updateSpeedScale(msg);

  auto read = [&msg](const Axis & a, double scale) {
      if (a.index < 0 || static_cast<size_t>(a.index) >= msg.axes.size()) {
        return 0.0;
      }
      const double max = a.max * scale;
      return a.sign * std::clamp(msg.axes[a.index] * max, -max, max);
    };

  Command cmd;
  cmd.forward = read(axes_[0], speed_scale_);
  cmd.left = read(axes_[1], speed_scale_);
  cmd.up = read(axes_[2], speed_scale_);
  cmd.yaw_rate = read(axes_[3], 1.0);

  // Neutral sticks: one null command on release and then silence, so as not to
  // override the scenario/evader commanding the same drone.
  const bool idle = cmd.forward == 0.0 && cmd.left == 0.0 && cmd.up == 0.0 && cmd.yaw_rate == 0.0;
  if (idle_silence_ && idle && was_idle_) {
    return;
  }
  was_idle_ = idle;
  send(active_, cmd);
}

}  // namespace aero_pursuit

RCLCPP_COMPONENTS_REGISTER_NODE(aero_pursuit::JoyTeleopNode)
