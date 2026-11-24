#pragma once

#include <Eigen/Geometry>

#include <array>
#include <cmath>

// Frame conventions used in the package:
//  - ENU (East-North-Up): mundo ROS / Gazebo.
//  - NED (North-East-Down): mundo PX4.
//  - FLU (Forward-Left-Up): corpo ROS (base_link).
//  - FRD (Forward-Right-Down): corpo PX4.
namespace aero_pursuit::frames
{

/// NED <-> ENU axis swap (the operation is its own inverse).
inline Eigen::Vector3d nedToEnu(const Eigen::Vector3d & v) {return {v.y(), v.x(), -v.z()};}
inline Eigen::Vector3d enuToNed(const Eigen::Vector3d & v) {return {v.y(), v.x(), -v.z()};}

/// Converts the PX4 attitude quaternion [w, x, y, z] (FRD -> NED)
/// into the equivalent FLU -> ENU rotation.
inline Eigen::Quaterniond px4AttitudeToEnuFlu(const std::array<float, 4> & q_px4)
{
  static const Eigen::Quaterniond kNedToEnu(0.0, M_SQRT1_2, M_SQRT1_2, 0.0);
  static const Eigen::Quaterniond kFluToFrd(0.0, 1.0, 0.0, 0.0);
  const Eigen::Quaterniond q_ned_frd(q_px4[0], q_px4[1], q_px4[2], q_px4[3]);
  return (kNedToEnu * q_ned_frd.normalized() * kFluToFrd).normalized();
}

/// Inverse of px4AttitudeToEnuFlu: FLU -> ENU attitude to the PX4 quaternion
/// [w, x, y, z] (FRD -> NED), with hemisphere w >= 0.
inline std::array<float, 4> enuFluToPx4Attitude(const Eigen::Quaterniond & q_enu_flu)
{
  static const Eigen::Quaterniond kNedToEnu(0.0, M_SQRT1_2, M_SQRT1_2, 0.0);
  static const Eigen::Quaterniond kFluToFrd(0.0, 1.0, 0.0, 0.0);
  Eigen::Quaterniond q = (kNedToEnu.conjugate() * q_enu_flu.normalized() * kFluToFrd.conjugate());
  q.normalize();
  if (q.w() < 0.0) {
    q.coeffs() *= -1.0;
  }
  return {static_cast<float>(q.w()), static_cast<float>(q.x()),
    static_cast<float>(q.y()), static_cast<float>(q.z())};
}

/// Body axis swap FRD <-> FLU (angular rates, forces).
inline Eigen::Vector3d frdToFlu(const Eigen::Vector3d & v) {return {v.x(), -v.y(), -v.z()};}

/// PX4 yaw (NED, rad) that aligns the body x axis with an ENU direction.
inline double enuDirectionToNedYaw(const Eigen::Vector3d & dir_enu)
{
  return std::atan2(dir_enu.x(), dir_enu.y());
}

}  // namespace aero_pursuit::frames
