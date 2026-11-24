#include "aero_pursuit/sim/gz_world.hpp"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/twist.pb.h>

#include <cmath>

namespace aero_pursuit::sim
{
namespace
{
void toMsg(const ModelPose & p, gz::msgs::Pose & out)
{
  out.mutable_position()->set_x(p.position.x());
  out.mutable_position()->set_y(p.position.y());
  out.mutable_position()->set_z(p.position.z());
  out.mutable_orientation()->set_w(p.orientation.w());
  out.mutable_orientation()->set_x(p.orientation.x());
  out.mutable_orientation()->set_y(p.orientation.y());
  out.mutable_orientation()->set_z(p.orientation.z());
}
}  // namespace

GzWorld::GzWorld(std::string world)
: world_(std::move(world))
{
}

GzWorld::~GzWorld()
{
  gate_.shutdown(node_);  // node_ outlives the mutex and map the callback uses
}

bool GzWorld::subscribePoses()
{
  return node_.Subscribe("/world/" + world_ + "/dynamic_pose/info",
      std::function<void(const gz::msgs::Pose_V &)>(
        [this](const gz::msgs::Pose_V & m) {gate_.run([&] {onPoses(m);});}));
}

void GzWorld::onPoses(const gz::msgs::Pose_V & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg.has_header() && msg.header().has_stamp()) {
    const double t = msg.header().stamp().sec() + msg.header().stamp().nsec() * 1e-9;
    if (t + 0.5 < stamp_) {
      poses_.clear();  // World reset: discards models that may have vanished
    }
    stamp_ = t;
  }
  for (const auto & p : msg.pose()) {
    ModelPose & m = poses_[p.name()];
    m.position = {p.position().x(), p.position().y(), p.position().z()};
    m.orientation = Eigen::Quaterniond(
      p.orientation().w(), p.orientation().x(), p.orientation().y(), p.orientation().z());
    m.stamp = stamp_;
  }
}

std::unordered_map<std::string, ModelPose> GzWorld::poses(double * stamp) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (stamp) {
    *stamp = stamp_;
  }
  std::unordered_map<std::string, ModelPose> fresh;
  for (const auto & [name, pose] : poses_) {
    if (pose.stamp >= stamp_ - kStaleTime) {
      fresh.emplace(name, pose);
    }
  }
  return fresh;
}

std::optional<ModelPose> GzWorld::pose(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = poses_.find(name);
  if (it == poses_.end() || it->second.stamp < stamp_ - kStaleTime) {
    return std::nullopt;
  }
  return it->second;
}

void GzWorld::spawn(
  const std::string & name, const std::string & /*model*/, const std::string & sdf,
  const ModelPose & pose, Done done)
{
  gz::msgs::EntityFactory req;
  req.set_sdf(sdf);
  req.set_name(name);
  req.set_allow_renaming(false);
  toMsg(pose, *req.mutable_pose());
  std::function<void(const gz::msgs::Boolean &, const bool)> cb =
    [done](const gz::msgs::Boolean & rep, const bool result) {
      if (done) {
        done(result && rep.data());
      }
    };
  if (!node_.Request("/world/" + world_ + "/create", req, cb) && done) {
    done(false);
  }
}

void GzWorld::remove(const std::string & name, Done done)
{
  gz::msgs::Entity req;
  req.set_name(name);
  req.set_type(gz::msgs::Entity::MODEL);
  std::function<void(const gz::msgs::Boolean &, const bool)> cb =
    [this, name, done](const gz::msgs::Boolean & rep, const bool result) {
      if (result && rep.data()) {
        std::lock_guard<std::mutex> lock(mutex_);
        poses_.erase(name);
      }
      if (done) {
        done(result && rep.data());
      }
    };
  if (!node_.Request("/world/" + world_ + "/remove", req, cb) && done) {
    done(false);
  }
}

void GzWorld::setPose(const std::string & name, const ModelPose & pose, bool skip_if_busy)
{
  if (skip_if_busy && pending_set_pose_.load() > 0) {
    return;
  }
  gz::msgs::Pose req;
  req.set_name(name);
  toMsg(pose, req);
  ++pending_set_pose_;
  std::function<void(const gz::msgs::Boolean &, const bool)> cb =
    [this](const gz::msgs::Boolean &, const bool) {--pending_set_pose_;};
  if (!node_.Request("/world/" + world_ + "/set_pose", req, cb)) {
    --pending_set_pose_;
  }
}

void GzWorld::setPoses(const std::vector<std::pair<std::string, ModelPose>> & poses)
{
  if (poses.empty() || pending_set_poses_.load() > 0) {
    return;  // replay: the next batch carries newer poses anyway
  }
  gz::msgs::Pose_V req;
  for (const auto & [name, pose] : poses) {
    gz::msgs::Pose * p = req.add_pose();
    p->set_name(name);
    toMsg(pose, *p);
  }
  ++pending_set_poses_;
  std::function<void(const gz::msgs::Boolean &, const bool)> cb =
    [this](const gz::msgs::Boolean &, const bool) {--pending_set_poses_;};
  if (!node_.Request("/world/" + world_ + "/set_pose_vector", req, cb)) {
    --pending_set_poses_;
  }
}

template<typename Msg>
gz::transport::Node::Publisher & GzWorld::publisher(const std::string & topic)
{
  std::lock_guard<std::mutex> lock(pub_mutex_);
  auto it = publishers_.find(topic);
  if (it == publishers_.end()) {
    it = publishers_.emplace(topic, node_.Advertise<Msg>(topic)).first;
  }
  return it->second;
}

void GzWorld::publishTwist(const std::string & ns, const Eigen::Vector3d & linear, double yaw_rate)
{
  gz::msgs::Twist msg;
  msg.mutable_linear()->set_x(linear.x());
  msg.mutable_linear()->set_y(linear.y());
  msg.mutable_linear()->set_z(linear.z());
  msg.mutable_angular()->set_z(yaw_rate);
  publisher<gz::msgs::Twist>("/" + ns + "/gazebo/command/twist").Publish(msg);
}

void GzWorld::enable(const std::string & ns, bool on)
{
  gz::msgs::Boolean msg;
  msg.set_data(on);
  publisher<gz::msgs::Boolean>("/" + ns + "/enable").Publish(msg);
}

}  // namespace aero_pursuit::sim
