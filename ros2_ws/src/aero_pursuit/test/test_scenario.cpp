#include "aero_pursuit/scenario/agent_executor.hpp"
#include "aero_pursuit/scenario/guidance.hpp"
#include "aero_pursuit/scenario/scenario_io.hpp"
#include "aero_pursuit/scenario/trajectory_factory.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <set>

using namespace aero_pursuit::scenario;

namespace
{

TrajectorySpec spec(const std::string & type)
{
  TrajectorySpec s;
  s.type = type;
  s.center = {0, 0, 5};
  s.speed = 2.0;
  return s;
}

/// Runs the plan with perfect tracking; returns the sampled positions.
std::vector<Eigen::Vector3d> fly(AgentExecutor & exec, Eigen::Vector3d start, double horizon, double dt = 0.05)
{
  AgentState self;
  self.position = start;
  WorldView world;
  std::vector<Eigen::Vector3d> out;
  for (double t = 0.0; t <= horizon && !exec.finished(); t += dt) {
    const auto cmd = exec.update(t, dt, self, world);
    self.position = cmd.reference.has_position ? cmd.reference.position :
      self.position + cmd.reference.velocity * dt;
    out.push_back(self.position);
  }
  return out;
}

}  // namespace

TEST(Trajectory, ConstantSpeedAlongClosedCircle)
{
  auto s = spec("circle");
  s.radius = 10.0;
  const Trajectory t = buildTrajectory(s, Eigen::Vector3d::Zero());
  EXPECT_NEAR(t.path().length(), 2 * M_PI * 10.0, 0.05);
  EXPECT_NEAR(t.duration(), t.path().length() / 2.0, 1e-9);
  EXPECT_TRUE(t.path().closed());
  for (double tt = 0.0; tt < t.duration(); tt += 1.7) {
    const Reference r = t.sample(tt);
    EXPECT_NEAR((r.position - s.center).head<2>().norm(), 10.0, 0.05);
    EXPECT_NEAR(r.velocity.norm(), 2.0, 1e-6);
  }
  EXPECT_LT((t.sample(t.duration()).position - t.start()).norm(), 1e-6);  // fecha
}

TEST(Trajectory, WaypointHoldsAndSegmentSpeeds)
{
  auto s = spec("waypoints");
  s.points = {{{0, 0, 5}, 0.0, 0.0, {}}, {{10, 0, 5}, 5.0, 2.0, {}}, {{10, 10, 5}, 0.0, 0.0, {}}};
  const Trajectory t = buildTrajectory(s, Eigen::Vector3d::Zero());
  // 10 m a 2 m/s, 2 s parado, 10 m a 5 m/s.
  EXPECT_NEAR(t.duration(), 5.0 + 2.0 + 2.0, 1e-6);
  EXPECT_NEAR(t.sample(5.5).position.x(), 10.0, 1e-6);
  EXPECT_NEAR(t.sample(5.5).velocity.norm(), 0.0, 1e-9);
  EXPECT_NEAR(t.sample(8.0).velocity.norm(), 5.0, 1e-6);
  EXPECT_NEAR(t.sample(8.0).position.y(), 5.0, 1e-6);
}

TEST(Trajectory, SplinePassesThroughControlPoints)
{
  auto s = spec("waypoints");
  s.interpolation = "spline";
  s.points = {{{0, 0, 5}, 0, 0, {}}, {{10, 5, 6}, 0, 0, {}}, {{20, -5, 7}, 0, 0, {}}, {{30, 0, 5}, 0, 0, {}}};
  const Trajectory t = buildTrajectory(s, Eigen::Vector3d::Zero());
  for (const auto & w : s.points) {
    EXPECT_NEAR((t.path().position(t.path().project(w.position)) - w.position).norm(), 0.0, 1e-6);
  }
}

TEST(Trajectory, RelativeIsAnchored)
{
  auto s = spec("polygon");
  s.relative = true;
  s.center = {0, 0, 0};
  s.radius = 3.0;
  const Trajectory t = buildTrajectory(s, {100, 50, 8});
  EXPECT_NEAR((t.start() - Eigen::Vector3d(103, 50, 8)).norm(), 0.0, 1e-9);
}

TEST(Trajectory, AllTypesBuild)
{
  for (const auto * type : {"polygon", "circle", "figure8", "lissajous", "helix", "random"}) {
    const Trajectory t = buildTrajectory(spec(type), Eigen::Vector3d::Zero());
    EXPECT_GT(t.duration(), 0.0) << type;
  }
}

TEST(Executor, LoopsAndTransit)
{
  // Line 2x (each lap goes back to the start by transit) + hover; the whole path 2x.
  DroneSpec d;
  d.name = "X4";
  LegSpec square;
  square.kind = LegSpec::Kind::kTrajectory;
  square.trajectory = spec("line");
  square.trajectory.points = {{{0, 0, 5}, 0, 0, {}}, {{10, 0, 5}, 0, 0, {}}};
  square.trajectory.speed = 5.0;
  square.loops = 2;
  LegSpec hover;
  hover.kind = LegSpec::Kind::kHover;
  hover.duration = 1.0;
  d.plan = {square, hover};
  d.plan_loops = 2;

  ExecutorConfig cfg;
  cfg.transit_speed = 10.0;
  AgentExecutor exec(d, cfg);
  ASSERT_TRUE(exec.startPoint());
  EXPECT_NEAR(exec.startPoint()->x(), 0.0, 1e-9);
  const auto path = fly(exec, {0, 0, 5}, 60.0);
  EXPECT_TRUE(exec.finished());
  // 2 x (2 s + transit back 1 s + 2 s + 1 s hover) + transit back between path laps (1 s).
  EXPECT_NEAR(path.size() * 0.05, 13.0, 0.3);
  EXPECT_EQ(exec.legsStarted(), 2u * 3u);
}

TEST(Executor, HoldsTheLastPointWhenFinished)
{
  DroneSpec d;
  LegSpec leg;
  leg.kind = LegSpec::Kind::kTrajectory;
  leg.trajectory = spec("line");
  leg.trajectory.points = {{{0, 0, 5}, 0, 0, {}}, {{10, 0, 5}, 0, 0, {}}};
  leg.trajectory.speed = 3.0;
  d.plan = {leg};
  AgentExecutor exec(d, ExecutorConfig{});
  fly(exec, {0, 0, 5}, 20.0, 0.1);
  ASSERT_TRUE(exec.finished());
  AgentState self;
  WorldView w;
  EXPECT_NEAR(exec.update(30.0, 0.1, self, w).reference.position.x(), 10.0, 1e-9);
}

TEST(Executor, InfiniteLoopNeverFinishes)
{
  DroneSpec d;
  LegSpec leg;
  leg.kind = LegSpec::Kind::kTrajectory;
  leg.trajectory = spec("circle");
  leg.trajectory.radius = 2.0;
  leg.loops = 0;
  d.plan = {leg};
  AgentExecutor exec(d, ExecutorConfig{});
  fly(exec, {2, 0, 5}, 50.0);
  EXPECT_FALSE(exec.finished());
}

TEST(Trajectory, TimedPassesEachPointAtItsTime)
{
  TrajectorySpec t = spec("timed");
  t.points = {{{0, 0, 5}, 0, 0, 0.0}, {{4, 0, 5}, 0, 0, 2.0}, {{4, 0, 5}, 0, 0, 3.0},
    {{4, 6, 5}, 0, 0, 4.0}};
  const auto traj = buildTrajectory(t, Eigen::Vector3d::Zero());
  EXPECT_NEAR(traj.duration(), 4.0, 1e-9);
  EXPECT_NEAR(traj.sample(1.0).position.x(), 2.0, 1e-9);        // 2 m/s
  EXPECT_NEAR(traj.sample(1.0).velocity.norm(), 2.0, 1e-9);
  EXPECT_NEAR(traj.sample(2.5).velocity.norm(), 0.0, 1e-9);     // repeated point: wait
  EXPECT_NEAR(traj.sample(3.5).position.y(), 3.0, 1e-9);        // 6 m/s
  EXPECT_NEAR(traj.sample(3.5).velocity.norm(), 6.0, 1e-9);
}

TEST(Behavior, OrbitKeepsRadiusAroundPursuer)
{
  DroneSpec d;
  LegSpec leg;
  leg.kind = LegSpec::Kind::kBehavior;
  leg.behavior.type = "orbit";
  leg.behavior.radius = 5.0;
  d.plan = {leg};
  AgentExecutor exec(d, ExecutorConfig{});
  WorldView w;
  w.agents["x500_depth_0"].position = {10, 10, 8};
  AgentState self;
  for (double t = 0; t < 5; t += 0.5) {
    const auto cmd = exec.update(t, 0.5, self, w);
    EXPECT_NEAR((cmd.reference.position - w.agents["x500_depth_0"].position).head<2>().norm(), 5.0, 1e-6);
  }
}

TEST(Tracker, HeadingFrame)
{
  const Eigen::Vector3d v = worldToHeading({0, 1, 0}, M_PI / 2);  // Northwards, heading north
  EXPECT_NEAR(v.x(), 1.0, 1e-9);
  EXPECT_NEAR(v.y(), 0.0, 1e-9);
}

TEST(ScenarioIo, ErrorsCarryFieldPath)
{
  try {
    parseScenario("name: x\ntarget:\n  path: [nope]\n");
    FAIL();
  } catch (const ScenarioError & e) {
    EXPECT_NE(std::string(e.what()).find("target.path[0]"), std::string::npos) << e.what();
  }
  EXPECT_THROW(parseScenario("name: x\nbogus: 1\n"), ScenarioError);
  EXPECT_THROW(parseScenario("target: {name: x500_depth_0}\n"), ScenarioError);  // the pursuer
  EXPECT_THROW(parseScenario("target: {path: [{trajectory: {type: timed, points: [[0,0,0,5], [0,1,0,5]]}}]}\n"),
    ScenarioError);  // times must increase
}

TEST(ScenarioIo, ParameterTypes)
{
  const auto s = parseScenario(
    "parameters:\n  pursuit_controller_node: {a: true, b: 3, c: 2.5, d: lqi, e: '7', f: [1, 2]}\n");
  const auto & p = s.parameters.at("/pursuit_controller_node");
  EXPECT_TRUE(std::get<bool>(p.at("a")));
  EXPECT_EQ(std::get<std::int64_t>(p.at("b")), 3);
  EXPECT_DOUBLE_EQ(std::get<double>(p.at("c")), 2.5);
  EXPECT_EQ(std::get<std::string>(p.at("d")), "lqi");
  EXPECT_EQ(std::get<std::string>(p.at("e")), "7");
  EXPECT_EQ(std::get<std::vector<double>>(p.at("f")).size(), 2u);
}

TEST(ScenarioIo, ExampleFilesAreValid)
{
  namespace fs = std::filesystem;
  int n = 0;
  for (const auto & e : fs::directory_iterator(AERO_PURSUIT_SCENARIO_DIR)) {
    if (e.path().extension() != ".yaml") {
      continue;
    }
    SCOPED_TRACE(e.path().string());
    const Scenario s = loadScenarioFile(e.path().string());
    ExecutorConfig cfg;
    for (const auto & d : s.drones) {
      const auto pv = previewPlan(d, cfg, d.spawn.value_or(Eigen::Vector3d(0, 0, 5)), 60.0, 0.2);
      EXPECT_FALSE(pv.paths.empty()) << d.name;
    }
    ++n;
  }
  EXPECT_GE(n, 5);
}

TEST(ScenarioIo, OneTargetAndTimedCsvRows)
{
  const auto s = parseScenario(
    "target:\n  name: T1\n  start: [1, 2, 3]\n  loops: 0\n  path:\n"
    "    - trajectory: {type: timed, points: [[0, 0, 0, 5], [2, 4, 0, 5]]}\n");
  ASSERT_EQ(s.drones.size(), 1u);
  EXPECT_EQ(s.drones[0].name, "T1");
  EXPECT_EQ(s.pursuer.target, "T1");
  EXPECT_EQ(s.drones[0].plan_loops, 0);
  EXPECT_NEAR(*s.drones[0].plan[0].trajectory.points[1].time, 2.0, 1e-12);
  EXPECT_EQ(parseScenario("name: y\n").drones.size(), 1u);  // default target: X4
}

TEST(ScenarioIo, FaceTargetDefaults)
{
  const auto s = parseScenario("name: x\npursuer: {face_target: false, heading_tolerance_deg: 5}\n");
  EXPECT_FALSE(s.pursuer.face_target);
  EXPECT_NEAR(s.pursuer.yaw_tolerance, 5.0 * M_PI / 180.0, 1e-12);
  EXPECT_TRUE(parseScenario("name: y\n").pursuer.face_target);  // default: camera on the target
}

TEST(ScenarioIo, TargetSpeedScale)
{
  EXPECT_DOUBLE_EQ(parseScenario("name: x\n").target_speed, 0.0);
  EXPECT_DOUBLE_EQ(parseScenario("name: x\ntarget_speed: 2.5\n").target_speed, 2.5);
  EXPECT_THROW(parseScenario("name: x\ntarget_speed: -1\n"), ScenarioError);
  EXPECT_THROW(parseScenario("name: x\ntarget_speed_scale: 1.5\n"), ScenarioError);  // old field
}
