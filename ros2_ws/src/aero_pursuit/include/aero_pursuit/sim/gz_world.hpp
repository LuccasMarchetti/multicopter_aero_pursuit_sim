#pragma once

#include "aero_pursuit/sim/world.hpp"

#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

#include "aero_pursuit/sim/gz_callback_gate.hpp"

#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>

namespace aero_pursuit::sim
{

/// Direct access to the Gazebo world through gz-transport, without ros_gz_bridge: poses of
/// all models (one subscription to /world/<w>/dynamic_pose/info), spawning and
/// removal of models, teleport and commands to the velocity controllers of
/// any number of drones (without one bridge entry per topic).
/// Thread-safe; service requests are asynchronous (they never block the executor).
class GzWorld : public World
{
public:
  explicit GzWorld(std::string world);
  ~GzWorld() override;
  GzWorld(const GzWorld &) = delete;
  GzWorld & operator=(const GzWorld &) = delete;

  const std::string & world() const override {return world_;}

  bool subscribePoses() override;
  std::unordered_map<std::string, ModelPose> poses(double * stamp = nullptr) const override;
  std::optional<ModelPose> pose(const std::string & name) const override;

  /// Spawns the SDF (the catalog key is not needed by Gazebo).
  void spawn(
    const std::string & name, const std::string & model, const std::string & sdf,
    const ModelPose & pose, Done done = {}) override;
  void remove(const std::string & name, Done done = {}) override;
  void setPose(const std::string & name, const ModelPose & pose, bool skip_if_busy = false) override;
  /// One /world/<w>/set_pose_vector request per batch.
  void setPoses(const std::vector<std::pair<std::string, ModelPose>> & poses) override;

  /// /<ns>/gazebo/command/twist.
  void publishTwist(const std::string & ns, const Eigen::Vector3d & linear, double yaw_rate) override;
  /// MulticopterVelocityControl on/off: /<ns>/enable.
  void enable(const std::string & ns, bool on) override;

private:
  void onPoses(const gz::msgs::Pose_V & msg);
  template<typename Msg>
  gz::transport::Node::Publisher & publisher(const std::string & topic);

  std::string world_;
  gz::transport::Node node_;
  GzCallbackGate gate_;  ///< poses arrive on the gz-transport thread
  mutable std::mutex mutex_;
  std::unordered_map<std::string, ModelPose> poses_;
  double stamp_ = 0.0;
  std::mutex pub_mutex_;
  std::unordered_map<std::string, gz::transport::Node::Publisher> publishers_;
  std::atomic<int> pending_set_pose_{0};
  std::atomic<int> pending_set_poses_{0};
};

}  // namespace aero_pursuit::sim
