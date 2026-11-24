// replay.yaml of each session (SessionRecorderNode::snapshotForReplay): the simulation environment
// and every parameter of the pursuer/scenario nodes at the start of the run, read with
// asynchronous parameter clients. The GCS re-simulation (aero_pursuit_gcs/resim.py) starts from it.
#include "aero_pursuit/nodes/session_recorder_node.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace aero_pursuit
{

namespace
{
/// Parameter as {type, value} in the GCS type names (aero_pursuit_gcs ros_bridge).
void emitParameter(YAML::Emitter & e, const rclcpp::Parameter & p)
{
  using T = rclcpp::ParameterType;
  e << YAML::Key << p.get_name() << YAML::Value << YAML::Flow << YAML::BeginMap;
  switch (p.get_type()) {
    case T::PARAMETER_BOOL: e << YAML::Key << "type" << YAML::Value << "bool" << YAML::Key << "value" << YAML::Value << p.as_bool(); break;
    case T::PARAMETER_INTEGER: e << YAML::Key << "type" << YAML::Value << "int" << YAML::Key << "value" << YAML::Value << p.as_int(); break;
    case T::PARAMETER_DOUBLE: e << YAML::Key << "type" << YAML::Value << "double" << YAML::Key << "value" << YAML::Value << p.as_double(); break;
    case T::PARAMETER_STRING: e << YAML::Key << "type" << YAML::Value << "string" << YAML::Key << "value" << YAML::Value << YAML::DoubleQuoted << p.as_string(); break;
    case T::PARAMETER_BOOL_ARRAY: e << YAML::Key << "type" << YAML::Value << "bool[]" << YAML::Key << "value" << YAML::Value << YAML::Flow << p.as_bool_array(); break;
    case T::PARAMETER_INTEGER_ARRAY: e << YAML::Key << "type" << YAML::Value << "int[]" << YAML::Key << "value" << YAML::Value << YAML::Flow << p.as_integer_array(); break;
    case T::PARAMETER_DOUBLE_ARRAY: e << YAML::Key << "type" << YAML::Value << "double[]" << YAML::Key << "value" << YAML::Value << YAML::Flow << p.as_double_array(); break;
    case T::PARAMETER_STRING_ARRAY: e << YAML::Key << "type" << YAML::Value << "string[]" << YAML::Key << "value" << YAML::Value << YAML::Flow << p.as_string_array(); break;
    default: e << YAML::Key << "type" << YAML::Value << "unsupported"; break;
  }
  e << YAML::EndMap;
}

struct Snapshot
{
  std::mutex mutex;
  std::string dir;
  double start = 0.0;
  size_t pending = 0;
  bool written = false;
  std::map<std::string, std::vector<rclcpp::Parameter>> params;
  std::vector<std::shared_ptr<rclcpp::AsyncParametersClient>> clients;
  rclcpp::TimerBase::SharedPtr timeout;

  /// Writes replay.yaml once (all nodes answered, or the timeout fired).
  void write()
  {
    if (written) {
      return;
    }
    written = true;
    YAML::Emitter e;
    e << YAML::BeginMap;
    e << YAML::Key << "version" << YAML::Value << 1;
    e << YAML::Key << "start_time" << YAML::Value << start;
    e << YAML::Key << "env" << YAML::Value << YAML::BeginMap;
    for (const char * k : {"WORLD_NAME", "AERO_CAMERA_MOUNT", "AERO_SCENERY", "AERO_PURSUER_HOST",
        "AERO_SIM_BACKEND", "AERO_GIT_REV"})
    {
      const char * v = std::getenv(k);
      e << YAML::Key << k << YAML::Value << YAML::DoubleQuoted << (v ? v : "");
    }
    e << YAML::EndMap;
    e << YAML::Key << "parameters" << YAML::Value << YAML::BeginMap;
    for (const auto & [node, list] : params) {
      e << YAML::Key << node << YAML::Value << YAML::BeginMap;
      for (const auto & p : list) {
        emitParameter(e, p);
      }
      e << YAML::EndMap;
    }
    e << YAML::EndMap << YAML::EndMap;
    std::ofstream(dir + "/replay.yaml") << "# GCS replay: environment and parameters at the start of the session\n"
                                        << e.c_str() << "\n";
    clients.clear();
    if (timeout) {
      timeout->cancel();
    }
  }

  void done()
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (pending > 0 && --pending == 0) {
      write();
    }
  }
};
}  // namespace

void SessionRecorderNode::snapshotForReplay(const std::string & dir, const rclcpp::Time & start)
{
  auto snap = std::make_shared<Snapshot>();
  snap->dir = dir;
  snap->start = start.seconds();
  std::lock_guard<std::mutex> lock(snap->mutex);
  for (const auto & name : snapshot_nodes_) {
    auto client = std::make_shared<rclcpp::AsyncParametersClient>(this, name);
    if (!client->service_is_ready()) {
      continue;
    }
    ++snap->pending;
    snap->clients.push_back(client);
    client->list_parameters({}, 0,
      [snap, client, name](std::shared_future<rcl_interfaces::msg::ListParametersResult> listed) {
        const auto names = listed.get().names;
        client->get_parameters(names,
          [snap, name](std::shared_future<std::vector<rclcpp::Parameter>> got) {
            {
              std::lock_guard<std::mutex> lock(snap->mutex);
              snap->params[name] = got.get();
            }
            snap->done();
          });
      });
  }
  if (snap->pending == 0) {
    snap->write();
    return;
  }
  // Nodes that never answer must not hold the file back.
  snap->timeout = create_wall_timer(std::chrono::seconds(5), [snap]() {
      std::lock_guard<std::mutex> lock(snap->mutex);
      snap->write();
    });
}

}  // namespace aero_pursuit
