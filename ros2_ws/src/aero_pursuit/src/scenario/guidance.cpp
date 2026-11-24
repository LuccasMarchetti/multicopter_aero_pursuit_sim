#include "aero_pursuit/scenario/guidance.hpp"

#include <algorithm>
#include <cmath>

namespace aero_pursuit::scenario
{
namespace
{

Eigen::Vector3d saturate(const Eigen::Vector3d & v, double max_norm)
{
  const double n = v.norm();
  return n > max_norm && n > 0.0 ? v * (max_norm / n) : v;
}

/// Smooth repulsion: grows from 0 (at `radius`) to 1 (at contact), in the direction a -> self.
Eigen::Vector3d repulsion(const Eigen::Vector3d & self, const Eigen::Vector3d & other, double radius)
{
  const Eigen::Vector3d d = self - other;
  const double dist = d.norm();
  if (dist >= radius) {
    return Eigen::Vector3d::Zero();
  }
  const Eigen::Vector3d dir = dist > 1e-3 ? Eigen::Vector3d(d / dist) : Eigen::Vector3d::UnitZ();
  const double x = 1.0 - dist / radius;
  return dir * x * x;
}

}  // namespace

VelocityCommand trackReference(const Reference & ref, const AgentState & self, const TrackerGains & g)
{
  VelocityCommand cmd;
  cmd.velocity = ref.velocity;
  if (ref.has_position) {
    cmd.velocity += g.position_gain * (ref.position - self.position);
  }
  cmd.velocity = saturate(cmd.velocity, g.max_speed);
  if (ref.has_yaw) {
    cmd.yaw_rate = std::clamp(g.yaw_gain * wrapAngle(ref.yaw - self.yaw), -g.max_yaw_rate, g.max_yaw_rate);
  }
  return cmd;
}

Eigen::Vector3d worldToHeading(const Eigen::Vector3d & v, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c * v.x() + s * v.y(), -s * v.x() + c * v.y(), v.z()};
}


}  // namespace aero_pursuit::scenario
