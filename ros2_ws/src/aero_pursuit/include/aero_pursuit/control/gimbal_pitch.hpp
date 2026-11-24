#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace aero_pursuit::control
{

/// Geometric pointing law of a pitch-only gimbal (x500_gimbal_depth with yaw and roll held at 0).
///
/// The gimbal pitch joint turns the camera about the body -y axis (+ = up) relative to the BODY,
/// so the angle that points the camera at the target is its elevation seen from the body frame:
///
///   d_b = R_body^T (p_rel - R_body t_mount)        target from the gimbal pivot, body FLU
///   theta = atan2(d_b.z, d_b.x)                    up/down angle in the body x-z plane
///
/// Working in the body frame compensates the body tilt on its own (when the drone pitches down
/// to accelerate, d_b.z grows and the gimbal tilts up). The lateral offset (d_b.y) is the yaw's
/// job (the MPC heading). `p_rel` in ENU, the attitude FLU -> ENU, `t_mount` the pivot in the body.
double gimbalPitchToward(
  const Eigen::Vector3d & p_rel, const Eigen::Quaterniond & q_enu_flu,
  const Eigen::Vector3d & t_mount = Eigen::Vector3d::Zero());

/// Moves `current` toward `target` by at most max_rate * dt and clamps to [lo, hi] [rad].
double stepToward(double current, double target, double max_rate, double dt, double lo, double hi);

}  // namespace aero_pursuit::control
