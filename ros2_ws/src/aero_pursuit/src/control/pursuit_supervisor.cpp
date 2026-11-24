#include "aero_pursuit/control/pursuit_supervisor.hpp"

namespace aero_pursuit::control
{

const char * toString(PursuitMode mode)
{
  switch (mode) {
    case PursuitMode::kIdle: return "IDLE";
    case PursuitMode::kTracking: return "TRACKING";
    case PursuitMode::kCoasting: return "COASTING";
    case PursuitMode::kLost: return "LOST";
    case PursuitMode::kDisabled: return "DISABLED";
    case PursuitMode::kApproach: return "APPROACH";
  }
  return "UNKNOWN";
}

PursuitMode PursuitSupervisor::update(
  bool enabled, std::optional<double> target_age, std::optional<double> bearing_age)
{
  const bool was_pursuing = mode_ == PursuitMode::kTracking || mode_ == PursuitMode::kCoasting;
  PursuitMode next;
  if (!enabled) {
    next = PursuitMode::kDisabled;
  } else if (!target_age && !bearing_age) {
    next = PursuitMode::kIdle;
  } else if (target_age && *target_age <= params_.coast_timeout) {
    next = PursuitMode::kTracking;
  } else if (bearing_age && *bearing_age <= params_.bearing_timeout) {
    // The current direction is more informative than extrapolating an old 3D estimate.
    next = PursuitMode::kApproach;
  } else if (target_age && *target_age <= params_.lost_timeout && was_pursuing) {
    // Only extrapolates if it was already pursuing: an old estimate never engages.
    next = PursuitMode::kCoasting;
  } else {
    next = (mode_ == PursuitMode::kIdle) ? PursuitMode::kIdle : PursuitMode::kLost;
  }

  engaged_ = isActive(next) && !isActive(mode_);
  changed_ = next != mode_;
  previous_ = mode_;
  mode_ = next;
  return mode_;
}

}  // namespace aero_pursuit::control
