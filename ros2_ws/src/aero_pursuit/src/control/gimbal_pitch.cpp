#include "aero_pursuit/control/gimbal_pitch.hpp"

#include <algorithm>
#include <cmath>

namespace aero_pursuit::control
{

double gimbalPitchToward(
  const Eigen::Vector3d & p_rel, const Eigen::Quaterniond & q_enu_flu,
  const Eigen::Vector3d & t_mount)
{
  const Eigen::Quaterniond q = q_enu_flu.normalized();
  const Eigen::Vector3d d_b = q.conjugate() * p_rel - t_mount;  // body FLU, from the pivot
  return std::atan2(d_b.z(), d_b.x());
}

double stepToward(double current, double target, double max_rate, double dt, double lo, double hi)
{
  const double step = std::max(0.0, max_rate * dt);
  return std::clamp(current + std::clamp(target - current, -step, step), lo, hi);
}

}  // namespace aero_pursuit::control
