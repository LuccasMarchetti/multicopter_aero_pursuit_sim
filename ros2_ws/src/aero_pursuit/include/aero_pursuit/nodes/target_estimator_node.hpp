#pragma once

#include "aero_pursuit/estimation/constant_velocity_kf.hpp"
#include "aero_pursuit/estimation/singer_kf.hpp"
#include "aero_pursuit/msg/target_bearing.hpp"
#include "aero_pursuit/msg/target_tf_cmd.hpp"

#include <Eigen/Geometry>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include <deque>
#include <memory>
#include <optional>
#include <utility>
#include <string>
#include <unordered_map>
#include <vector>

namespace aero_pursuit
{

/// Estimates the target position and velocity in the `enu_frame` frame: origin at the
/// pursuer and axes aligned with the world (ENU). Being non-rotating, the KF's
/// constant-velocity model holds even with the drone rotating.
///
/// Measurement sources (exclusive):
///  - Vision: PointStamped in any frame static relative to `base_frame`
///    (e.g. camera_optical), converted via tf2 + PX4 attitude.
///  - Omniscient (TF): lookup `base_frame` -> target (Gazebo ground truth).
///
/// A line of sight without depth (TargetBearing, vision) is rotated into
/// `enu_frame` and republished for the direction-guided approach.
///
/// Measurement source, TF target, noise and gating can be tuned at runtime
/// (`ros2 param set`); /target_tf_cmd continua aceito.
///
/// The PX4 attitude is kept in a short history (reception instant on the
/// node clock) and interpolated (slerp) at each measurement stamp: the image arrives
/// tens of ms late, and rotating the vector by the latest attitude would get
/// the position wrong by ~ω·delay·distance during turns.
///
/// The frame is centered on the pursuer, which accelerates: the relative velocity
/// changes with the drone's own manoeuvre. The pursuer velocity (PX4
/// odometry, history interpolated like the attitude) enters the KF as a known input;
/// without it the process noise would have to absorb the pursuer acceleration, the
/// estimated relative velocity would lag the manoeuvre (the control loses
/// damping and oscillates) and the gating would reject the measurements.
///
/// TFs publicadas: base_frame -> enu_frame e enu_frame -> estimate_frame.
class TargetEstimatorNode : public rclcpp::Node
{
public:
  explicit TargetEstimatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using PointStamped = geometry_msgs::msg::PointStamped;

  void onMeasurement(const PointStamped & msg);
  void onBearing(const msg::TargetBearing & msg);
  void setTrackingSource(bool use_tf, const std::string & target_frame);
  rcl_interfaces::msg::SetParametersResult onParameters(const std::vector<rclcpp::Parameter> & params);
  void onAttitude(const px4_msgs::msg::VehicleAttitude & msg);
  void onOdometry(const px4_msgs::msg::VehicleOdometry & msg);
  void onTfCommand(const msg::TargetTfCmd & msg);
  void onOmniscientTimer();

  /// Static transform frame -> base_frame (looked up once and cached).
  /// Camera (measurement frame) -> base at the measurement instant (dynamic with the gimbal).
  std::optional<Eigen::Isometry3d> cameraToBase(const std::string & frame, const rclcpp::Time & stamp);
  /// FLU -> ENU attitude interpolated at instant `stamp` (empty without history).
  std::optional<Eigen::Quaterniond> attitudeAt(const rclcpp::Time & stamp) const;
  /// Pursuer ENU velocity interpolated at `stamp` (empty without history).
  std::optional<Eigen::Vector3d> chaserVelocityAt(const rclcpp::Time & stamp) const;
  void processMeasurement(const Eigen::Vector3d & p_base, const rclcpp::Time & stamp);
  /// Measurement already in the pursuer-centered ENU frame.
  void processMeasurementEnu(const Eigen::Vector3d & z, const rclcpp::Time & stamp);
  void publishEstimate(const rclcpp::Time & stamp);  ///< KF state (vision)
  void publishEstimate(const rclcpp::Time & stamp, const Eigen::Vector3d & p,
    const Eigen::Vector3d & v, const estimation::TargetFilter::CovariancePV & P,
    const Eigen::Vector3d & a = Eigen::Vector3d::Zero(),
    const Eigen::Matrix3d & P_a = Eigen::Matrix3d::Zero());
  /// (Re)creates the filter for kf_model (state is discarded: next measurement resets it).
  void buildFilter();
  /// Omniscient acceleration by a quadratic least-squares fit of the target positions in the window.
  std::optional<Eigen::Vector3d> windowedAcceleration(double t, const Eigen::Vector3d & p);
  rclcpp::Time measurementTime(const builtin_interfaces::msg::Time & stamp);

  // Configuration
  std::string base_frame_;
  std::string enu_frame_;
  std::string estimate_frame_;
  std::string tf_target_frame_;
  std::string world_frame_;  ///< ENU root of the Gazebo TFs (omniscient mode)
  bool use_tf_tracking_ = false;
  double reset_timeout_ = 1.0;
  double gate_chi2_ = 0.0;
  int max_gate_rejections_ = 5;
  double enu_tf_period_ = 0.02;

  // Estado
  std::string kf_model_ = "cv";                  ///< cv | singer
  estimation::ConstantVelocityKF::Params cv_params_;
  estimation::SingerKF::Params singer_params_;
  std::unique_ptr<estimation::TargetFilter> kf_;
  rclcpp::Time last_stamp_;
  /// Omniscient: last exact relative position (for the velocity by differencing).
  std::optional<std::pair<rclcpp::Time, Eigen::Vector3d>> omni_prev_;
  /// Omniscient: last exact target world position and velocity (acceleration by differencing).
  std::optional<std::pair<rclcpp::Time, Eigen::Vector3d>> omni_target_prev_;
  std::optional<Eigen::Vector3d> omni_target_vel_;
  /// Omniscient: target world positions in the last omniscient_accel_window seconds.
  std::deque<std::pair<double, Eigen::Vector3d>> omni_target_hist_;
  double omni_accel_window_ = 0.0;
  int gate_rejections_ = 0;
  std::optional<Eigen::Quaterniond> q_enu_flu_;
  std::deque<std::pair<rclcpp::Time, Eigen::Quaterniond>> attitude_history_;
  double attitude_history_sec_ = 1.0;
  std::deque<std::pair<rclcpp::Time, Eigen::Vector3d>> velocity_history_;
  bool compensate_chaser_motion_ = true;
  rclcpp::Time last_enu_tf_stamp_;
  bool warned_clock_ = false;

  // Interfaces ROS
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<PointStamped>::SharedPtr sub_measurement_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr sub_attitude_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<msg::TargetTfCmd>::SharedPtr sub_tf_cmd_;
  rclcpp::Subscription<msg::TargetBearing>::SharedPtr sub_bearing_;
  rclcpp::Publisher<msg::TargetBearing>::SharedPtr pub_bearing_;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr pub_accel_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_handle_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
  rclcpp::TimerBase::SharedPtr omniscient_timer_;
};

}  // namespace aero_pursuit
