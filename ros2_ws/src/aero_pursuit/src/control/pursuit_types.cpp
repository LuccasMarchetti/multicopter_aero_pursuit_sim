#include "aero_pursuit/control/pursuit_types.hpp"

namespace aero_pursuit::control
{

namespace
{
Eigen::Vector3d clampNorm(const Eigen::Vector3d & v, double max_norm)
{
  const double n = v.norm();
  return (n > max_norm && n > 0.0) ? Eigen::Vector3d(v * (max_norm / n)) : v;
}
}  // namespace

Eigen::Vector3d standoffPoint(const Eigen::Vector3d & p_rel, double standoff)
{
  const double range = p_rel.norm();
  if (range < 1e-3) {
    return Eigen::Vector3d::Zero();
  }
  return p_rel - (standoff / range) * p_rel;
}

ControlCommand limitVelocity(const ControlCommand & cmd, const Limits & limits)
{
  ControlCommand out = cmd;
  out.velocity = clampNorm(cmd.velocity, limits.max_velocity);
  return out;
}

}  // namespace aero_pursuit::control
