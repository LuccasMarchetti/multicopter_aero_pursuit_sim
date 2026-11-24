#pragma once

#include <cstdint>
#include <optional>

namespace aero_pursuit::control
{

/// Values aligned with aero_pursuit/msg/PursuitStatus.
enum class PursuitMode : std::uint8_t
{
  kIdle = 0,      ///< No target received yet
  kTracking = 1,  ///< Recent estimate: pursuit active
  kCoasting = 2,  ///< Aging estimate: extrapolates with the KF
  kLost = 3,      ///< Target lost: pursuit off (offboard holds position)
  kDisabled = 4,  ///< Disabled by the operator
  kApproach = 5,  ///< No depth: approach guided only by the line of sight
};

const char * toString(PursuitMode mode);

/// State machine that closes the pursuit loop as a function of the age
/// of the last target estimate. ROS independent.
class PursuitSupervisor
{
public:
  struct Params
  {
    double coast_timeout = 0.3;  ///< Up to here: TRACKING [s]
    double lost_timeout = 1.5;   ///< Up to here: COASTING; then: LOST [s]
    double bearing_timeout = 0.5;  ///< Maximum line-of-sight age for APPROACH [s]
  };

  PursuitSupervisor() : PursuitSupervisor(Params{}) {}
  explicit PursuitSupervisor(const Params & params) : params_(params) {}

  void setParams(const Params & params) {params_ = params;}

  /// Updates the mode. An empty `target_age` means there was never a 3D target;
  /// an empty `bearing_age` means there is no line of sight (fallback without depth).
  /// Priority: TRACKING (recent 3D) > APPROACH (direction only) > COASTING (extrapolation).
  PursuitMode update(
    bool enabled, std::optional<double> target_age,
    std::optional<double> bearing_age = std::nullopt);

  PursuitMode mode() const {return mode_;}
  PursuitMode previous() const {return previous_;}
  /// True if the current mode commands the vehicle.
  bool active() const {return isActive(mode_);}
  /// True in the cycle in which pursuit was (re)engaged.
  bool engaged() const {return engaged_;}
  /// True in the cycle in which the mode changed.
  bool changed() const {return changed_;}

  static bool isActive(PursuitMode m)
  {
    return m == PursuitMode::kTracking || m == PursuitMode::kCoasting ||
           m == PursuitMode::kApproach;
  }

private:
  Params params_;
  PursuitMode mode_ = PursuitMode::kIdle;
  PursuitMode previous_ = PursuitMode::kIdle;
  bool engaged_ = false;
  bool changed_ = false;
};

}  // namespace aero_pursuit::control
