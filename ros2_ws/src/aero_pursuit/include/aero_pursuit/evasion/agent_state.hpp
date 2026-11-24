#pragma once

#include <Eigen/Geometry>

#include <Eigen/Dense>

namespace aero_pursuit::evasion
{

/// Kinematic state of an agent in the world ENU frame.
struct AgentState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  double yaw = 0.0;  ///< ENU heading [rad] (0 = east)
  Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();  ///< Body -> world (ENU)
};

}  // namespace aero_pursuit::evasion
