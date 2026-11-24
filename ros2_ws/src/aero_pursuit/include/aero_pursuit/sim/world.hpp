#pragma once

#include <Eigen/Geometry>

#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace aero_pursuit::sim
{

struct ModelPose
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  double stamp = 0.0;  ///< Simulation time it was last seen [s]
};

/// ENU heading [rad] of an orientation.
double yawOf(const Eigen::Quaterniond & q);

/// Replaces the content of <robotNamespace> with `ns`: the same SDF serves several drones.
std::string withNamespace(const std::string & sdf, const std::string & ns);

/// Access to the simulated world used by the scenario runner: poses of every model, spawning
/// and removal, teleport and velocity commands to the (non-PX4) drones. Implemented over
/// gz-transport (GzWorld, Gazebo) and over plain ROS topics (RosWorld, Isaac Sim).
/// Implementations are thread-safe and never block the caller.
class World
{
public:
  using Done = std::function<void(bool ok)>;

  static constexpr double kStaleTime = 1.0;  ///< [s] of simulation time

  virtual ~World() = default;

  virtual const std::string & world() const = 0;

  /// Subscribes to the poses. Returns false if the subscription fails.
  virtual bool subscribePoses() = 0;
  /// Last pose of each model and the simulation time of the sample [s]. Models not seen for
  /// kStaleTime are left out.
  virtual std::unordered_map<std::string, ModelPose> poses(double * stamp = nullptr) const = 0;
  virtual std::optional<ModelPose> pose(const std::string & name) const = 0;

  /// Spawns `model` (catalog key; `sdf` is its description, namespace already applied) as `name`.
  virtual void spawn(
    const std::string & name, const std::string & model, const std::string & sdf,
    const ModelPose & pose, Done done = {}) = 0;
  virtual void remove(const std::string & name, Done done = {}) = 0;
  /// Teleport. With `skip_if_busy`, the request may be dropped if the previous one is pending.
  virtual void setPose(const std::string & name, const ModelPose & pose, bool skip_if_busy = false) = 0;
  /// Teleports several models at once (session replay); dropped while the previous batch is pending.
  virtual void setPoses(const std::vector<std::pair<std::string, ModelPose>> & poses)
  {
    for (const auto & [name, pose] : poses) {
      setPose(name, pose);
    }
  }

  /// Linear velocity in the heading frame + yaw rate of drone `ns`.
  virtual void publishTwist(const std::string & ns, const Eigen::Vector3d & linear, double yaw_rate) = 0;
  /// Turns the velocity controller of drone `ns` on/off.
  virtual void enable(const std::string & ns, bool on) = 0;
};

}  // namespace aero_pursuit::sim
