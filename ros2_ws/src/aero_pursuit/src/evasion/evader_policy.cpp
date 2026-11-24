#include "aero_pursuit/evasion/evader_policy.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aero_pursuit::evasion
{

std::unique_ptr<EvaderPolicy> makeEvaderPolicy(const EvaderConfig & config)
{
  if (config.type == "hover") {
    return std::make_unique<HoverEvader>();
  }
  if (config.type == "random") {
    return std::make_unique<RandomWalkEvader>(config);
  }
  if (config.type == "flee") {
    return std::make_unique<FleeEvader>(config);
  }
  throw std::invalid_argument("Unknown evasion policy: '" + config.type + "'");
}

std::vector<std::string> availableEvaderPolicies()
{
  return {"hover", "random", "flee"};
}

Eigen::Vector3d constrainToArena(
  const Eigen::Vector3d & velocity, const AgentState & evader, const ArenaLimits & arena)
{
  Eigen::Vector3d v = velocity;

  // Outside the radius: removes the outward radial component and pulls back.
  const Eigen::Vector2d offset = evader.position.head<2>() - arena.center;
  const double dist = offset.norm();
  if (dist > arena.radius && dist > 1e-6) {
    const Eigen::Vector2d outward = offset / dist;
    const double radial = v.head<2>().dot(outward);
    if (radial > 0.0) {
      v.head<2>() -= radial * outward;
    }
    v.head<2>() -= 0.5 * arena.max_speed * outward;
  }

  const double z = evader.position.z();
  if (z < arena.min_altitude) {
    v.z() = std::max(v.z(), 0.5 * arena.max_speed);
  } else if (z > arena.max_altitude) {
    v.z() = std::min(v.z(), -0.5 * arena.max_speed);
  }

  const double n = v.norm();
  if (n > arena.max_speed) {
    v *= arena.max_speed / n;
  }
  return v;
}

RandomWalkEvader::RandomWalkEvader(const EvaderConfig & config)
: config_(config),
  rng_(config.seed != 0 ? config.seed : std::random_device{}())
{
}

Eigen::Vector3d RandomWalkEvader::compute(const AgentState & evader, const AgentState &, double dt)
{
  const double tau = std::max(config_.random_tau, 1e-3);
  const double s = config_.random_sigma * std::sqrt(std::max(dt, 0.0));
  const Eigen::Vector3d w(noise_(rng_), noise_(rng_), 0.3 * noise_(rng_));
  v_ += -v_ * (dt / tau) + s * w;
  v_ = constrainToArena(v_, evader, config_.arena);
  return v_;
}

Eigen::Vector3d FleeEvader::compute(const AgentState & evader, const AgentState & pursuer, double dt)
{
  t_ += dt;
  Eigen::Vector3d away = evader.position - pursuer.position;
  away.z() = 0.0;  // Flees in the plane; altitude is left to the envelope
  if (away.norm() < 1e-3) {
    away = Eigen::Vector3d::UnitX();
  }
  away.normalize();

  const Eigen::Vector3d lateral = Eigen::Vector3d::UnitZ().cross(away);
  const double phase = config_.jink_period > 0.0 ?
    std::sin(2.0 * M_PI * t_ / config_.jink_period) : 0.0;
  const double a = std::clamp(config_.jink_amplitude, 0.0, 1.0);

  const Eigen::Vector3d dir = (std::sqrt(1.0 - a * a * phase * phase) * away +
    a * phase * lateral).normalized();
  return constrainToArena(config_.arena.max_speed * dir, evader, config_.arena);
}

}  // namespace aero_pursuit::evasion
