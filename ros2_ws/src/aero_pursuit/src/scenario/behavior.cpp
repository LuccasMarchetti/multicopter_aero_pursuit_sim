#include "aero_pursuit/scenario/behavior.hpp"

#include "aero_pursuit/evasion/evader_policy.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aero_pursuit::scenario
{
namespace
{

Reference velocityOnly(const Eigen::Vector3d & v)
{
  Reference r;
  r.velocity = v;
  return r;
}

Reference hold(const AgentState & self)
{
  Reference r;
  r.position = self.position;
  r.has_position = true;
  return r;
}

/// Rule-based escape policies (flee, random) as a scenario behavior.
class EvaderBehavior : public Behavior
{
public:
  EvaderBehavior(const BehaviorSpec & spec, std::string frame)
  : frame_(std::move(frame))
  {
    evasion::EvaderConfig c;
    c.type = spec.type;
    c.arena.center = spec.arena_center;
    c.arena.radius = spec.arena_radius;
    c.arena.min_altitude = spec.min_altitude;
    c.arena.max_altitude = spec.max_altitude;
    c.arena.max_speed = spec.speed;
    c.jink_period = spec.period;
    c.jink_amplitude = std::clamp(spec.amplitude, 0.0, 1.0);
    c.seed = spec.seed ? spec.seed : 1u;
    policy_ = evasion::makeEvaderPolicy(c);
  }

  Reference update(const AgentState & self, const WorldView & world, double, double dt) override
  {
    const AgentState * pursuer = world.find(frame_);
    AgentState far;  // No visible pursuer (preview): distant reference
    far.position = self.position - Eigen::Vector3d(100.0, 0.0, 0.0);
    return velocityOnly(policy_->compute(self, pursuer ? *pursuer : far, dt));
  }

private:
  std::string frame_;
  std::unique_ptr<evasion::EvaderPolicy> policy_;
};

class OrbitBehavior : public Behavior
{
public:
  explicit OrbitBehavior(const BehaviorSpec & spec) : spec_(spec) {}

  Reference update(const AgentState & self, const WorldView & world, double t, double) override
  {
    const AgentState * c = world.find(spec_.frame);
    if (!c) {
      return hold(self);
    }
    const double w = spec_.speed / std::max(0.5, spec_.radius);
    const double a = spec_.phase + w * t;
    Reference r;
    r.has_position = true;
    r.position = c->position + Eigen::Vector3d(
      spec_.radius * std::cos(a), spec_.radius * std::sin(a), spec_.offset.z());
    r.velocity = c->velocity + spec_.speed * Eigen::Vector3d(-std::sin(a), std::cos(a), 0.0);
    return r;
  }

private:
  BehaviorSpec spec_;
};

}  // namespace

std::unique_ptr<Behavior> makeBehavior(const BehaviorSpec & spec, const std::string & pursuer_frame)
{
  BehaviorSpec s = spec;
  if (s.frame.empty()) {
    s.frame = pursuer_frame;
  }
  if (s.type == "flee" || s.type == "random") {
    return std::make_unique<EvaderBehavior>(s, s.frame);
  }
  if (s.type == "orbit") {
    return std::make_unique<OrbitBehavior>(s);
  }
  throw std::invalid_argument("unknown behavior: " + spec.type);
}

}  // namespace aero_pursuit::scenario
