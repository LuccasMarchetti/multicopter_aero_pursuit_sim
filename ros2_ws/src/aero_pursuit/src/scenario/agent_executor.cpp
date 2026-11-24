#include "aero_pursuit/scenario/agent_executor.hpp"

#include "aero_pursuit/scenario/trajectory_factory.hpp"

#include <Eigen/Geometry>

#include <cmath>

namespace aero_pursuit::scenario
{
namespace
{

double headingOf(const Eigen::Vector3d & v, double fallback, double min_speed)
{
  return v.head<2>().norm() > min_speed ? std::atan2(v.y(), v.x()) : fallback;
}
}  // namespace

AgentExecutor::AgentExecutor(DroneSpec spec, ExecutorConfig config)
: spec_(std::move(spec)), config_(std::move(config))
{
  reset();
}

void AgentExecutor::reset()
{
  index_ = 0;
  plan_loop_ = 0;
  leaf_ = nullptr;
  leaf_iter_ = 0;
  started_ = false;
  finished_ = false;
  legs_started_ = 0;
  label_.clear();
  behavior_.reset();
}

bool AgentExecutor::nextLeaf()
{
  ++index_;
  if (index_ >= spec_.plan.size()) {
    ++plan_loop_;
    if (spec_.plan_loops != 0 && plan_loop_ >= spec_.plan_loops) {
      leaf_ = nullptr;
      return false;
    }
    index_ = 0;
  }
  leaf_ = &spec_.plan[index_];
  return true;
}

std::optional<Eigen::Vector3d> AgentExecutor::startPoint() const
{
  // First leg of the path.
  if (spec_.plan.empty()) {
    return std::nullopt;
  }
  const LegSpec & leg = spec_.plan.front();
  switch (leg.kind) {
    case LegSpec::Kind::kTrajectory:
      if (leg.trajectory.relative) {
        return std::nullopt;
      }
      return buildTrajectory(leg.trajectory, Eigen::Vector3d::Zero()).start();
    case LegSpec::Kind::kHover:
      return leg.point;
    default:
      return std::nullopt;
  }
}

void AgentExecutor::activate(double t, const AgentState & self)
{
  ++legs_started_;
  phase_t0_ = t;
  behavior_.reset();
  label_ = leaf_->label;
  const Eigen::Vector3d & plan_pos = self.position;
  const double speed = leaf_->speed > 0.0 ? leaf_->speed : config_.transit_speed;

  switch (leaf_->kind) {
    case LegSpec::Kind::kTrajectory:
      main_ = buildTrajectory(leaf_->trajectory, plan_pos);
      break;
    case LegSpec::Kind::kHover:
      main_ = Trajectory::hover(leaf_->point.value_or(plan_pos), leaf_->duration);
      break;
    case LegSpec::Kind::kGoto:
      main_ = Trajectory::line(plan_pos, *leaf_->point, speed);
      break;
    case LegSpec::Kind::kBehavior:
      behavior_ = makeBehavior(leaf_->behavior, config_.pursuer_frame);
      phase_ = Phase::kBehavior;
      return;
  }
  // Start far from the current position: straight-line transit before the leg.
  const Eigen::Vector3d start = main_.start();
  if ((start - self.position).norm() > config_.transit_threshold) {
    transit_ = Trajectory::line(self.position, start, std::max(speed, 0.5));
    phase_ = Phase::kTransit;
  } else {
    phase_ = Phase::kTrajectory;
  }
}

bool AgentExecutor::phaseDone(double t) const
{
  const double elapsed = t - phase_t0_;
  switch (phase_) {
    case Phase::kTransit: return elapsed >= transit_.duration();
    case Phase::kTrajectory: return elapsed >= main_.duration();
    case Phase::kBehavior: return leaf_->duration > 0.0 && elapsed >= leaf_->duration;
  }
  return true;
}

void AgentExecutor::applyYaw(
  Reference & ref, const YawSpec & yaw, const AgentState & self, const WorldView & world)
{
  switch (yaw.mode) {
    case YawSpec::Mode::kVelocity:
      last_yaw_ = headingOf(ref.velocity, last_yaw_, config_.min_heading_speed);
      break;
    case YawSpec::Mode::kFixed:
      last_yaw_ = yaw.fixed;
      break;
    case YawSpec::Mode::kFace:
      if (const AgentState * other = world.find(yaw.frame)) {
        const Eigen::Vector3d d = other->position - self.position;
        last_yaw_ = headingOf(d, last_yaw_, 0.1);
      }
      break;
    case YawSpec::Mode::kHold:
      ref.has_yaw = false;
      return;
  }
  ref.yaw = last_yaw_;
  ref.has_yaw = true;
}

AgentCommand AgentExecutor::update(double t, double dt, const AgentState & self, const WorldView & world)
{
  AgentCommand cmd;
  if (!started_) {
    started_ = true;
    last_yaw_ = self.yaw;
    last_position_ = self.position;
    if (spec_.plan.empty()) {
      finished_ = true;
    } else {
      leaf_ = &spec_.plan.front();
      activate(t, self);
    }
  }
  for (int guard = 0; !finished_ && phaseDone(t) && guard < 64; ++guard) {
    if (phase_ == Phase::kTransit) {
      phase_ = Phase::kTrajectory;
      phase_t0_ = t;
      continue;
    }
    ++leaf_iter_;
    if (leaf_->loops == 0 || leaf_iter_ < leaf_->loops) {
      activate(t, self);
      continue;
    }
    leaf_iter_ = 0;
    if (!nextLeaf()) {
      if (phase_ == Phase::kTrajectory) {
        last_position_ = main_.end();  // hold the path's last point, not the previous tick's
      }
      finished_ = true;
      label_ = "end";
      break;
    }
    activate(t, self);
  }

  Reference & ref = cmd.reference;
  if (finished_) {
    ref.position = last_position_;
    ref.has_position = true;
    ref.has_yaw = false;
    return cmd;
  }
  const double elapsed = t - phase_t0_;
  switch (phase_) {
    case Phase::kTransit:
      ref = transit_.sample(elapsed);
      break;
    case Phase::kTrajectory:
      ref = main_.sample(elapsed);
      break;
    case Phase::kBehavior:
      ref = behavior_->update(self, world, elapsed, dt);
      break;
  }
  last_position_ = ref.has_position ? ref.position : self.position;
  const YawSpec yaw = phase_ == Phase::kTransit ? YawSpec{} : leaf_->yaw.value_or(spec_.yaw);
  applyYaw(ref, yaw, self, world);
  return cmd;
}

PlanPreview previewPlan(
  const DroneSpec & spec, const ExecutorConfig & config, const Eigen::Vector3d & start,
  double horizon, double dt)
{
  PlanPreview out;
  AgentExecutor exec(spec, config);
  WorldView world;
  AgentState self;
  self.position = start;
  out.paths.push_back({start});
  double t = 0.0;
  for (; t <= horizon; t += dt) {
    const AgentCommand cmd = exec.update(t, dt, self, world);
    out.reactive = out.reactive || exec.reactive();
    if (exec.finished()) {
      out.finite = true;
      break;
    }
    const Reference & r = cmd.reference;
    self.position = r.has_position ? r.position : self.position + r.velocity * dt;
    self.velocity = r.velocity;
    if (r.has_yaw) {
      self.yaw = r.yaw;
    }
    auto & path = out.paths.back();
    if ((self.position - path.back()).norm() > 0.05) {
      path.push_back(self.position);
    }
  }
  out.duration = std::min(t, horizon);
  return out;
}

}  // namespace aero_pursuit::scenario
