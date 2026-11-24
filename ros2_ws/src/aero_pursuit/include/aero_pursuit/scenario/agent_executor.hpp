#pragma once

#include "aero_pursuit/scenario/behavior.hpp"
#include "aero_pursuit/scenario/spec.hpp"
#include "aero_pursuit/scenario/trajectory.hpp"

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace aero_pursuit::scenario
{

struct ExecutorConfig
{
  double transit_speed = 3.0;      ///< Transit to the start of a leg [m/s]
  double transit_threshold = 1.5;  ///< Distance to the start that triggers the transit [m]
  double min_heading_speed = 0.3;  ///< Below this the velocity heading is frozen [m/s]
  std::string pursuer_frame = "x500_depth_0";
};

struct AgentCommand
{
  Reference reference;
};

/// Runs the target's path: goes through the legs (each with its own laps, the whole path
/// `plan_loops` times), inserts a straight transit when the start of a leg is far, and applies
/// the heading policy. Not copyable: the cursor points into the path itself.
class AgentExecutor
{
public:
  AgentExecutor(DroneSpec spec, ExecutorConfig config);
  AgentExecutor(const AgentExecutor &) = delete;
  AgentExecutor & operator=(const AgentExecutor &) = delete;

  const DroneSpec & spec() const {return spec_;}

  /// Where the drone must be when the run starts (staging). nullopt if the plan
  /// starts with a relative/reactive leg or is empty.
  std::optional<Eigen::Vector3d> startPoint() const;

  /// Back to the first leg (new run).
  void reset();

  /// `t`: run time [s]. Call in increasing order of t.
  AgentCommand update(double t, double dt, const AgentState & self, const WorldView & world);

  bool finished() const {return finished_;}
  const std::string & legLabel() const {return label_;}
  std::size_t legsStarted() const {return legs_started_;}
  int legIteration() const {return leaf_iter_;}
  bool inTransit() const {return phase_ == Phase::kTransit;}
  bool reactive() const {return phase_ == Phase::kBehavior;}

private:
  enum class Phase { kTransit, kTrajectory, kBehavior };
  bool nextLeaf();
  void activate(double t, const AgentState & self);
  bool phaseDone(double t) const;
  void applyYaw(Reference & ref, const YawSpec & yaw, const AgentState & self, const WorldView & world);

  DroneSpec spec_;
  ExecutorConfig config_;

  std::size_t index_ = 0;
  int plan_loop_ = 0;
  const LegSpec * leaf_ = nullptr;
  int leaf_iter_ = 0;
  bool started_ = false;
  bool finished_ = false;
  std::size_t legs_started_ = 0;
  std::string label_;

  Phase phase_ = Phase::kTrajectory;
  double phase_t0_ = 0.0;
  Trajectory transit_;
  Trajectory main_;
  std::unique_ptr<Behavior> behavior_;
  double last_yaw_ = 0.0;
  Eigen::Vector3d last_position_ = Eigen::Vector3d::Zero();
};

/// Predicted path of a plan with perfect tracking (reactive behaviors
/// stay still, without the state of the pursuer). One polyline.
struct PlanPreview
{
  std::vector<std::vector<Eigen::Vector3d>> paths;
  bool reactive = false;
  bool finite = false;   ///< The plan ends within the horizon
  double duration = 0.0;
};

/// Plan time to preview for a scenario: the whole path (it stops by itself when the path ends),
/// up to 30 min for one that repeats forever; only the duration when it limits a run flown at
/// the path's own speeds (target_speed > 0 changes the clock, so the whole path is shown).
inline double previewHorizon(const Scenario & s)
{
  constexpr double kMax = 1800.0;
  return s.duration > 0.0 && s.target_speed <= 0.0 ? std::min(s.duration, kMax) : kMax;
}

PlanPreview previewPlan(
  const DroneSpec & spec, const ExecutorConfig & config, const Eigen::Vector3d & start,
  double horizon, double dt = 0.1);

}  // namespace aero_pursuit::scenario
