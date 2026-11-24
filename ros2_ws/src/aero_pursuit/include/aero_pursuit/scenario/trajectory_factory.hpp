#pragma once

#include "aero_pursuit/scenario/spec.hpp"
#include "aero_pursuit/scenario/trajectory.hpp"

namespace aero_pursuit::scenario
{

/// Builds the trajectory from a specification. `anchor` shifts the coordinates when
/// `spec.relative` (drone position at the start of the leg); `hover` without an explicit center
/// uses the anchor itself. Throws std::invalid_argument on invalid specifications.
Trajectory buildTrajectory(const TrajectorySpec & spec, const Eigen::Vector3d & anchor);

}  // namespace aero_pursuit::scenario
