#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace aero_pursuit::control
{

// The data exchanged between the pursuit node and the MPC (no ROS here):
//   PursuitState    what the node measured and estimated (input of Mpc::compute)
//   ControlCommand  what the MPC decided (output, sent to the PX4 as a TrajectorySetpoint)
// plus small helpers: limitVelocity (the node's safety saturation), standoffPoint (metrics) and
// offAxisAngle (bounding-box angle).

/// Measured pursuer state (PX4 odometry converted to ENU/FLU).
struct VehicleState
{
  Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();  ///< FLU -> ENU
  Eigen::Vector3d body_rates = Eigen::Vector3d::Zero();          ///< p, q, r (FLU) [rad/s]
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();            ///< ENU [m/s]
};

/// Safety limit applied by the node after the law (limitVelocity).
struct Limits
{
  double max_velocity = 2.0;      ///< Maximum velocity norm [m/s]
};

/// Bounding box of the target in the image, as measured by the tracker (/target_bearing_enu).
/// Always available when the tracker has a box, with or without depth.
struct TargetBox
{
  Eigen::Vector3d direction = Eigen::Vector3d::UnitX();  ///< Unit line of sight to the box center (ENU)
  double off_axis = 0.0;       ///< Angle between the line of sight and the optical axis [rad]
  double angular_size = std::numeric_limits<double>::quiet_NaN();  ///< Angular size of the box [rad]
  /// Target beyond the depth range: distance > this [m]; NaN if the depth saw the target.
  double range_lower_bound = std::numeric_limits<double>::quiet_NaN();
  double age = 0.0;            ///< Age of the measurement [s]
};

/// Engagement state, expressed in the ENU frame centered on the pursuer.
struct PursuitState
{
  /// False when there is no usable 3D estimate (mode APPROACH: only `box` is valid). p_rel,
  /// v_rel and v_target are then zero and the law decides what to do.
  bool has_position = true;
  double standoff = 5.0;  ///< Desired distance to the target (`target_distance`) [m]
  Limits limits;          ///< `max_vel` [m/s]
  std::optional<TargetBox> box;  ///< Last bounding box (empty if none within `bearing_timeout`)
  Eigen::Vector3d p_rel = Eigen::Vector3d::Zero();    ///< Target position relative to the pursuer [m]
  Eigen::Vector3d v_rel = Eigen::Vector3d::Zero();    ///< Target velocity relative to the pursuer [m/s]
  Eigen::Vector3d v_chaser = Eigen::Vector3d::Zero(); ///< Pursuer reference velocity [m/s]
  Eigen::Vector3d v_target = Eigen::Vector3d::Zero(); ///< Estimated absolute target velocity [m/s]
  /// Estimated absolute target acceleration [m/s²]; zero = constant-velocity prediction.
  Eigen::Vector3d a_target = Eigen::Vector3d::Zero();
  /// Decay time of a_target in the predictions (Singer model) [s].
  double accel_tau = 1.0;

  /// Target displacement after t seconds: v t + a (τ t − τ² (1 − e^{−t/τ})).
  Eigen::Vector3d targetDisplacement(double t) const
  {
    if (a_target.isZero()) {
      return v_target * t;
    }
    const double tau = std::max(accel_tau, 1e-3);
    return v_target * t + a_target * (tau * t - tau * tau * (1.0 - std::exp(-t / tau)));
  }

  /// Target velocity after t seconds: v + a τ (1 − e^{−t/τ}).
  Eigen::Vector3d targetVelocityAt(double t) const
  {
    if (a_target.isZero()) {
      return v_target;
    }
    const double tau = std::max(accel_tau, 1e-3);
    return v_target + a_target * (tau * (1.0 - std::exp(-t / tau)));
  }
  /// Measured attitude/rates/velocity (empty without recent odometry). Used by the
  /// dynamic models; the kinematic laws ignore it.
  std::optional<VehicleState> vehicle;
  /// Measured pursuer heading, ENU yaw (0 = east, counterclockwise) [rad]; NaN if unknown.
  double heading = std::numeric_limits<double>::quiet_NaN();
};


/// Output command (ENU). The predicted trajectory is optional (empty if not supported).
struct ControlCommand
{
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  /// Jerk feedforward [m/s³]; NaN when the law does not provide it (PX4 ignores NaN).
  Eigen::Vector3d jerk = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  /// Future pursuer positions relative to the current position (ENU) [m].
  std::vector<Eigen::Vector3d> predicted_path;
  /// Heading setpoint, ENU yaw [rad]. Empty: the node holds the heading measured at the
  /// engagement.
  std::optional<double> yaw;
  /// Heading rate feedforward, ENU (counterclockwise +) [rad/s].
  double yaw_rate = 0.0;
};

/// Angle between the line of sight and the optical axis from the azimuth/elevation offsets
/// (cos θ = cos az · cos el) [rad].
inline double offAxisAngle(double azimuth, double elevation)
{
  return std::acos(std::clamp(std::cos(azimuth) * std::cos(elevation), -1.0, 1.0));
}

/// Pursuit point: relative position `standoff` meters before the target,
/// along the line of sight.
Eigen::Vector3d standoffPoint(const Eigen::Vector3d & p_rel, double standoff);

/// Safety saturation of the law's command: |velocity| <= max_velocity (direction kept). The
/// acceleration is left as the law computed it.
ControlCommand limitVelocity(const ControlCommand & cmd, const Limits & limits);

}  // namespace aero_pursuit::control
