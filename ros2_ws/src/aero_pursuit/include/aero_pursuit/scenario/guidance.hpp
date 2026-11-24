#pragma once

#include "aero_pursuit/scenario/spec.hpp"
#include "aero_pursuit/scenario/trajectory.hpp"

#include <vector>

namespace aero_pursuit::scenario
{

struct TrackerGains
{
  double position_gain = 1.2;   ///< [1/s]
  double max_speed = 6.0;       ///< [m/s]
  double yaw_gain = 1.5;        ///< [1/s]
  double max_yaw_rate = 1.2;    ///< [rad/s]
};

struct VelocityCommand
{
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();  ///< ENU [m/s]
  double yaw_rate = 0.0;        ///< [rad/s]
};

/// Reference follower for velocity-commanded drones: direct feed of the
/// reference velocity + proportional position feedback, saturated.
VelocityCommand trackReference(const Reference & ref, const AgentState & self, const TrackerGains & gains);

/// ENU velocity in the vehicle heading frame (that of MulticopterVelocityControl).
Eigen::Vector3d worldToHeading(const Eigen::Vector3d & v_world, double yaw);

}  // namespace aero_pursuit::scenario
