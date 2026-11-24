// Parameters of PursuitControllerNode: declaration (with descriptions and ranges), reading into the
// controller configuration, live validation and the rebuild/runtime split. The control loop and
// the topic handling are in pursuit_controller_node.cpp.
#include "aero_pursuit/nodes/pursuit_controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace aero_pursuit
{

namespace
{
/// Parameters of the law (`mpc.params.<name>`) are forwarded without touching the rest.
bool isMpcParameter(const std::string & name)
{
  return name.rfind("mpc.params.", 0) == 0;
}

}  // namespace

void PursuitControllerNode::declareControllerParameters()
{
  auto described = [](const std::string & text) {
      rcl_interfaces::msg::ParameterDescriptor d;
      d.description = text;
      return d;
    };
  declare_parameter<double>("target_distance", 5.0);
  declare_parameter<double>("max_vel", 2.0);
  declare_parameter<double>("max_acc_xy", 8.0,
    described("Safety: norm of the horizontal acceleration sent to the PX4 [m/s²] (0: off)"));
  declare_parameter<double>("min_altitude", 3.0,
    described("Safety floor: the pursuer never descends below this height above the takeoff point [m] (0: off)"));
  declare_parameter<double>("coast_timeout", 0.3);
  declare_parameter<double>("lost_timeout", 1.5);
  declare_parameter<double>("bearing_timeout", 0.5);
  declare_parameter<double>("velocity_timeout", 0.5);

  auto ranged = [](const std::string & text, double lo, double hi) {
      rcl_interfaces::msg::ParameterDescriptor d;
      d.description = text;
      rcl_interfaces::msg::FloatingPointRange r;
      r.from_value = lo;
      r.to_value = hi;
      d.floating_point_range.push_back(r);
      return d;
    };
  // Camera mount (TF base -> camera), only for the image metrics (target azimuth/elevation).
  declare_parameter<std::string>("camera_base_frame", "x500_depth_0",
    described("Pursuer body frame (root of the camera TF; image metrics)"));
  declare_parameter<std::string>("camera_frame", "gimbal_mount",
    described("Camera gimbal pivot (static TF); the measured pitch is added (image metrics)"));

  const TargetAccelConfig ta;
  declare_parameter<bool>("target_accel.enabled", ta.enabled,
    described("Use the estimated target acceleration (/target_3d_accel_filtered; kf_model=singer in the estimator)"));
  declare_parameter<double>("target_accel.tau", ta.tau,
    ranged("Decay of the target acceleration in the predictions (Singer model) [s]", 0.05, 10.0));
  declare_parameter<double>("target_accel.max", ta.max,
    ranged("Clamp of |a_target| [m/s²]", 0.0, 20.0));
  declare_parameter<double>("target_accel.timeout", ta.timeout,
    ranged("Maximum age of the acceleration estimate [s]", 0.05, 2.0));

}

void PursuitControllerNode::loadRuntimeParameters()
{
  auto num = [this](const std::string & name) {return get_parameter(name).as_double();};

  standoff_ = num("target_distance");
  limits_.max_velocity = num("max_vel");
  min_altitude_ = num("min_altitude");
  max_acc_xy_ = num("max_acc_xy");
  velocity_timeout_ = num("velocity_timeout");
  accel_cfg_.enabled = get_parameter("target_accel.enabled").as_bool();
  accel_cfg_.tau = num("target_accel.tau");
  accel_cfg_.max = num("target_accel.max");
  accel_cfg_.timeout = num("target_accel.timeout");
  supervisor_.setParams({num("coast_timeout"), num("lost_timeout"), num("bearing_timeout")});
  bearing_timeout_ = num("bearing_timeout");
  lost_timeout_ = num("lost_timeout");


  runtime_pending_ = false;
}

void PursuitControllerNode::applyMpcParameters(const std::vector<rclcpp::Parameter> & params)
{
  for (const auto & p : params) {
    if (!isMpcParameter(p.get_name())) {
      continue;
    }
    const auto short_name = p.get_name().substr(std::string("mpc.params.").size());
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      controller_->eraseParameter(short_name);  // removed from the GCS list
      continue;
    }
    double value = NAN;
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      value = p.as_double();
    } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      value = static_cast<double>(p.as_int());
    } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      value = p.as_bool() ? 1.0 : 0.0;
    } else {
      continue;
    }
    controller_->setParameter(short_name, value);
  }
}

rcl_interfaces::msg::SetParametersResult PursuitControllerNode::validateParameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : params) {
    const auto & name = p.get_name();
    if (isMpcParameter(name)) {
      const auto t = p.get_type();
      if (t != rclcpp::ParameterType::PARAMETER_NOT_SET && t != rclcpp::ParameterType::PARAMETER_DOUBLE &&
        t != rclcpp::ParameterType::PARAMETER_INTEGER &&
        t != rclcpp::ParameterType::PARAMETER_BOOL)
      {
        result.successful = false;
        result.reason = name + " must be numeric";
      }
    } else if (name == "max_vel" && p.as_double() <= 0.0) {
      result.successful = false;
      result.reason = name + " must be > 0";
    }
  }
  return result;
}

void PursuitControllerNode::onParametersSet(const std::vector<rclcpp::Parameter> & params)
{
  // Applied in the next control cycle (same thread as the loop), except the law's own numbers.
  applyMpcParameters(params);
  for (const auto & p : params) {
    const auto & name = p.get_name();
    if (name == "start_enabled" || name == "use_sim_time" || isMpcParameter(name)) {
      continue;
    }
    runtime_pending_ = true;
  }
}

}  // namespace aero_pursuit
