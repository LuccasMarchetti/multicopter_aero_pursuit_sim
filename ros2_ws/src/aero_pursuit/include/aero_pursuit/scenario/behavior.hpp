#pragma once

#include "aero_pursuit/scenario/spec.hpp"
#include "aero_pursuit/scenario/trajectory.hpp"

#include <map>
#include <memory>
#include <string>

namespace aero_pursuit::scenario
{

/// State (ground truth) of every vehicle in the simulation, by model name.
struct WorldView
{
  std::map<std::string, AgentState> agents;

  const AgentState * find(const std::string & name) const
  {
    const auto it = agents.find(name);
    return it == agents.end() ? nullptr : &it->second;
  }
};

/// Reactive leg: the reference depends on the current state of the other vehicles.
class Behavior
{
public:
  virtual ~Behavior() = default;
  /// `t`: time since the start of the leg [s].
  virtual Reference update(const AgentState & self, const WorldView & world, double t, double dt) = 0;
};

/// `pursuer_frame` is the default `frame` (the drone fled from / orbited).
/// Throws std::invalid_argument for unknown types.
std::unique_ptr<Behavior> makeBehavior(const BehaviorSpec & spec, const std::string & pursuer_frame);

}  // namespace aero_pursuit::scenario
