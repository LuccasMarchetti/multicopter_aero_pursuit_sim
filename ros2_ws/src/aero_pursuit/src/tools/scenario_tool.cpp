// Scenario command-line tool (no ROS): validates and previews a
// file with the same code the scenario_runner executes. Used by the GCS.
//
//   scenario_tool validate <file|->
//   scenario_tool preview  <file|-> [--horizon s] [--dt s]       JSON with the paths
//   scenario_tool get <file|-> <field>                            world | name
//   scenario_tool sample <trajectory|-> [--dt s]                 one trajectory (YAML/JSON map):
//       {"duration", "length", "knot_times": [...], "samples": [[t, x, y, z, speed], ...]}

#include "aero_pursuit/scenario/agent_executor.hpp"
#include "aero_pursuit/scenario/scenario_io.hpp"
#include "aero_pursuit/scenario/trajectory_factory.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <optional>
#include <sstream>
#include <string>

namespace sc = aero_pursuit::scenario;

namespace
{

std::string jsonString(const std::string & s)
{
  std::ostringstream o;
  o << '"';
  for (const char c : s) {
    switch (c) {
      case '"': o << "\\\""; break;
      case '\\': o << "\\\\"; break;
      case '\n': o << "\\n"; break;
      case '\t': o << "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          o << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c) << std::dec;
        } else {
          o << c;
        }
    }
  }
  o << '"';
  return o.str();
}

std::string jsonVec(const Eigen::Vector3d & v)
{
  std::ostringstream o;
  o << std::fixed << std::setprecision(2) << '[' << v.x() << ',' << v.y() << ',' << v.z() << ']';
  return o.str();
}

std::string jsonOpt(const std::optional<Eigen::Vector3d> & v)
{
  return v ? jsonVec(*v) : "null";
}

std::string jsonPaths(const std::vector<std::vector<Eigen::Vector3d>> & paths)
{
  std::string out = "[";
  for (std::size_t i = 0; i < paths.size(); ++i) {
    out += i ? ",[" : "[";
    for (std::size_t k = 0; k < paths[i].size(); ++k) {
      out += (k ? "," : "") + jsonVec(paths[i][k]);
    }
    out += "]";
  }
  return out + "]";
}

std::string readInput(const std::string & path)
{
  if (path == "-") {
    return std::string(std::istreambuf_iterator<char>(std::cin), {});
  }
  std::ifstream in(path);
  if (!in) {
    throw sc::ScenarioError("could not open " + path);
  }
  return std::string(std::istreambuf_iterator<char>(in), {});
}

int preview(const sc::Scenario & s, double horizon, double dt)
{
  if (horizon <= 0.0) {
    horizon = sc::previewHorizon(s);
  }
  sc::ExecutorConfig cfg;
  cfg.pursuer_frame = s.pursuer.frame;
  std::cout << "{\"ok\":true,\"name\":" << jsonString(s.name)
            << ",\"description\":" << jsonString(s.description)
            << ",\"duration\":" << s.duration << ",\"repeat\":" << s.repeat
            << ",\"horizon\":" << horizon << ",\"drones\":[";
  for (std::size_t i = 0; i < s.drones.size(); ++i) {
    const auto & d = s.drones[i];
    std::optional<Eigen::Vector3d> start;
    try {
      start = sc::AgentExecutor(d, cfg).startPoint();
    } catch (const std::exception &) {
    }
    const Eigen::Vector3d origin = start.value_or(d.spawn.value_or(Eigen::Vector3d(0.0, 0.0, 5.0)));
    const auto pv = sc::previewPlan(d, cfg, origin, horizon, dt);
    std::cout << (i ? "," : "") << "{\"name\":" << jsonString(d.name)
              << ",\"model\":" << jsonString(d.model) << ",\"role\":\"target\""
              << ",\"spawn\":" << jsonOpt(d.spawn)
              << ",\"start\":" << jsonOpt(start) << ",\"reactive\":" << (pv.reactive ? "true" : "false")
              << ",\"finite\":" << (pv.finite ? "true" : "false") << ",\"plan_duration\":" << pv.duration
              << ",\"paths\":" << jsonPaths(pv.paths) << '}';
  }
  std::cout << "],\"pursuer\":{\"frame\":" << jsonString(s.pursuer.frame)
            << ",\"start\":" << jsonOpt(s.pursuer.start) << ",\"target\":" << jsonString(s.pursuer.target)
            << ",\"detection\":" << jsonString(s.pursuer.detection)
            << ",\"acquisition\":" << jsonString(s.pursuer.acquisition) << ",\"paths\":";
  if (!s.pursuer.plan.empty()) {
    sc::DroneSpec p;
    p.name = s.pursuer.frame;
    p.plan = s.pursuer.plan;
    const auto pv = sc::previewPlan(p, cfg, s.pursuer.start.value_or(Eigen::Vector3d(0.0, 0.0, 5.0)),
        horizon, dt);
    std::cout << jsonPaths(pv.paths);
  } else {
    std::cout << "[]";
  }
  std::cout << "}}\n";
  return 0;
}

int sample(const std::string & text, double dt)
{
  const sc::TrajectorySpec spec = sc::parseTrajectoryText(text);
  const sc::Trajectory traj = sc::buildTrajectory(spec, Eigen::Vector3d::Zero());
  const double duration = traj.duration();
  constexpr double kMaxSamples = 3000.0;
  dt = std::max(dt, duration / kMaxSamples);
  std::ostringstream o;
  o << std::fixed << std::setprecision(3);
  o << "{\"ok\":true,\"duration\":" << duration << ",\"length\":" << traj.path().length()
    << ",\"knot_times\":[";
  const auto & kt = traj.knotTimes();
  for (std::size_t i = 0; i < kt.size(); ++i) {
    o << (i ? "," : "") << kt[i];
  }
  o << "],\"samples\":[";
  bool first = true;
  for (double t = 0.0;; t += dt) {
    const bool last = t >= duration;
    const double tt = last ? duration : t;
    const sc::Reference r = traj.sample(tt);
    o << (first ? "" : ",") << '[' << tt << ',' << r.position.x() << ',' << r.position.y() << ','
      << r.position.z() << ',' << r.velocity.norm() << ']';
    first = false;
    if (last) {
      break;
    }
  }
  o << "]}\n";
  std::cout << o.str();
  return 0;
}

int usage()
{
  std::cerr << "usage: scenario_tool {validate|preview|get|sample} <file|-> [options]\n";
  return 2;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    return usage();
  }
  const std::string cmd = argv[1];
  double horizon = 0.0;
  double dt = 0.1;
  for (int i = 3; i + 1 < argc; i += 2) {
    const std::string opt = argv[i];
    if (opt == "--horizon") {
      horizon = std::atof(argv[i + 1]);
    } else if (opt == "--dt") {
      dt = std::max(0.02, std::atof(argv[i + 1]));
    }
  }
  try {
    if (cmd == "sample") {
      return sample(readInput(argv[2]), dt);
    }
    const sc::Scenario s = sc::parseScenario(readInput(argv[2]));
    if (cmd == "validate") {
      std::cout << "{\"ok\":true,\"name\":" << jsonString(s.name) << ",\"drones\":" << s.drones.size()
                << "}\n";
      return 0;
    }
    if (cmd == "preview") {
      return preview(s, horizon, dt);
    }
    if (cmd == "get" && argc >= 4) {
      const std::string field = argv[3];
      if (field == "world") {
        std::cout << s.world << '\n';
      } else if (field == "name") {
        std::cout << s.name << '\n';
      } else {
        std::cerr << "unknown field: " << field << '\n';
        return 2;
      }
      return 0;
    }
    return usage();
  } catch (const std::exception & e) {
    std::cout << "{\"ok\":false,\"error\":" << jsonString(e.what()) << "}\n";
    return 1;
  }
}
