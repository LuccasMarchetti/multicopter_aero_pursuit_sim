#pragma once

#include "aero_pursuit/evasion/agent_state.hpp"

#include <memory>
#include <random>
#include <string>
#include <vector>

namespace aero_pursuit::evasion
{

/// Envelope common to the escape policies (keeps the evader inside the arena).
struct ArenaLimits
{
  Eigen::Vector2d center = Eigen::Vector2d::Zero();  ///< Centro horizontal (ENU) [m]
  double radius = 50.0;         ///< Horizontal arena radius [m]
  double min_altitude = 2.0;    ///< [m]
  double max_altitude = 15.0;   ///< [m]
  double max_speed = 3.0;       ///< [m/s]
};

/// Interface of the evader strategies (Strategy). Returns the desired ENU velocity.

class EvaderPolicy
{
public:
  virtual ~EvaderPolicy() = default;
  virtual std::string name() const = 0;
  virtual void reset() {}
  virtual Eigen::Vector3d compute(const AgentState & evader, const AgentState & pursuer, double dt) = 0;
};

struct EvaderConfig
{
  std::string type = "flee";    ///< hover | random | flee
  ArenaLimits arena;
  double random_tau = 2.0;      ///< random: time constant of the OU process [s]
  double random_sigma = 1.5;    ///< random: noise intensity [m/s/√s]
  double jink_period = 4.0;     ///< flee: period of the lateral manoeuvre [s]
  double jink_amplitude = 0.6;  ///< flee: lateral fraction of the speed [0..1]
  unsigned seed = 0;            ///< 0: random seed
};

/// Throws std::invalid_argument for unknown types.
std::unique_ptr<EvaderPolicy> makeEvaderPolicy(const EvaderConfig & config);
std::vector<std::string> availableEvaderPolicies();

/// Saturates the velocity and redirects it into the arena (horizontal and altitude).
Eigen::Vector3d constrainToArena(
  const Eigen::Vector3d & velocity, const AgentState & evader, const ArenaLimits & arena);

class HoverEvader : public EvaderPolicy
{
public:
  std::string name() const override {return "hover";}
  Eigen::Vector3d compute(const AgentState &, const AgentState &, double) override
  {
    return Eigen::Vector3d::Zero();
  }
};

/// Smooth random walk (Ornstein-Uhlenbeck on the velocity): basis for
/// generating varied non-cooperative behavior data.
class RandomWalkEvader : public EvaderPolicy
{
public:
  explicit RandomWalkEvader(const EvaderConfig & config);
  std::string name() const override {return "random";}
  void reset() override {v_.setZero();}
  Eigen::Vector3d compute(const AgentState & evader, const AgentState & pursuer, double dt) override;

private:
  EvaderConfig config_;
  std::mt19937 rng_;
  std::normal_distribution<double> noise_{0.0, 1.0};
  Eigen::Vector3d v_ = Eigen::Vector3d::Zero();
};

/// Heuristic escape: moves away from the pursuer along the line of sight with
/// periodic lateral manoeuvring (jinking), which degrades constant-velocity predictions.
class FleeEvader : public EvaderPolicy
{
public:
  explicit FleeEvader(const EvaderConfig & config) : config_(config) {}
  std::string name() const override {return "flee";}
  void reset() override {t_ = 0.0;}
  Eigen::Vector3d compute(const AgentState & evader, const AgentState & pursuer, double dt) override;

private:
  EvaderConfig config_;
  double t_ = 0.0;
};

}  // namespace aero_pursuit::evasion
