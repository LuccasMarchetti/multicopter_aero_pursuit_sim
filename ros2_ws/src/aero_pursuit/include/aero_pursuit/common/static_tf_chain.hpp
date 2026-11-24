#pragma once

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <string>

namespace aero_pursuit
{

/// Rotations of a static TF tree (/tf_static): child -> (parent, R_parent_child).
/// Answers the rotation of a frame relative to an ancestor, e.g. the camera mount
/// (base -> camera_link) without a full tf2 listener on the high-rate /tf.
class StaticTfChain
{
public:
  void set(
    const std::string & parent, const std::string & child, const Eigen::Quaterniond & q,
    const Eigen::Vector3d & t = Eigen::Vector3d::Zero())
  {
    links_[child] = Link{parent, q.normalized(), t};
  }

  /// Pose of `frame` in `root` (T_root_frame: origin and axes); nullopt if not a static
  /// descendant.
  std::optional<Eigen::Isometry3d> transform(const std::string & root, const std::string & frame) const
  {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    std::string f = frame;
    for (std::size_t depth = 0; depth <= links_.size(); ++depth) {
      if (f == root) {
        return T;
      }
      const auto it = links_.find(f);
      if (it == links_.end()) {
        return std::nullopt;
      }
      Eigen::Isometry3d link = Eigen::Isometry3d::Identity();
      link.linear() = it->second.rotation.toRotationMatrix();
      link.translation() = it->second.translation;
      T = link * T;
      f = it->second.parent;
    }
    return std::nullopt;
  }

  /// Rotation that takes vectors in `frame` to `root` (R_root_frame); nullopt if
  /// `frame` is not a static descendant of `root`.
  std::optional<Eigen::Quaterniond> rotation(const std::string & root, const std::string & frame) const
  {
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    std::string f = frame;
    for (std::size_t depth = 0; depth <= links_.size(); ++depth) {
      if (f == root) {
        return q;
      }
      const auto it = links_.find(f);
      if (it == links_.end()) {
        return std::nullopt;
      }
      q = it->second.rotation * q;
      f = it->second.parent;
    }
    return std::nullopt;  // Cycle
  }

  bool empty() const {return links_.empty();}

private:
  struct Link
  {
    std::string parent;
    Eigen::Quaterniond rotation;
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  };
  std::map<std::string, Link> links_;
};

/// Elevation of a camera's optical axis in its base (FLU) frame: the camera_link
/// convention (ROS, x forward) is assumed. + = up [rad].
inline double opticalAxisPitch(const Eigen::Quaterniond & r_base_camera)
{
  const Eigen::Vector3d axis = r_base_camera * Eigen::Vector3d::UnitX();
  return std::asin(std::clamp(axis.z() / std::max(axis.norm(), 1e-9), -1.0, 1.0));
}

}  // namespace aero_pursuit
