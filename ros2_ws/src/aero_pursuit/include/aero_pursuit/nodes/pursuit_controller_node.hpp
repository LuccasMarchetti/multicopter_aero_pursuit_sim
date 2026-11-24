#pragma once

#include "aero_pursuit/common/static_tf_chain.hpp"
#include "aero_pursuit/control/mpc.hpp"
#include "aero_pursuit/control/course_rate.hpp"
#include "aero_pursuit/control/performance_metrics.hpp"
#include "aero_pursuit/control/pursuit_supervisor.hpp"
#include "aero_pursuit/msg/pursuit_metrics.hpp"
#include "aero_pursuit/msg/pursuit_status.hpp"
#include "aero_pursuit/msg/target_bearing.hpp"

#include <Eigen/Dense>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <string>
#include <vector>

namespace aero_pursuit
{

/// Closes the pursuit loop: consumes the target estimate (relative ENU), supervises its validity
/// (TRACKING/COASTING/APPROACH/LOST) and applies the pursuit law (control::Mpc, control/mpc.hpp).
///
/// Data flow of a cycle (controlLoop -> runController):
///   onTarget / onBearing / onLocalPosition / onOdometry  fill the latest measurements;
///   runController builds control::PursuitState -> Mpc::compute() -> limitVelocity (|v| <= max_vel)
///   -> TrajectorySetpoint (velocity + acceleration feedforward + heading) to the PX4 velocity loop
///   (/offboard/trajectory_setpoint, offboard_control).
///
/// No yaw or vertical policy: the law commands all three axes; the heading is the law's
/// (ControlCommand::yaw) or, when empty, the one measured at the engagement (held).
///
/// The GCS "Control" tab sets `mpc.params.<name>` (double); each value reaches
/// Mpc::setParameter(name, value). In APPROACH (no depth) the law still runs, with
/// PursuitState::has_position = false and only the bounding box (PursuitState::box).
class PursuitControllerNode : public rclcpp::Node
{
public:
  explicit PursuitControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct TargetEstimate
  {
    Eigen::Vector3d p_rel;
    Eigen::Vector3d v_rel;
    rclcpp::Time stamp;
  };

  /// Use of the estimated target acceleration (target_accel.*).
  struct TargetAccelConfig
  {
    bool enabled = false;
    double tau = 1.0;      ///< Decay of the acceleration in the predictions [s]
    double max = 5.0;      ///< Clamp of |a| [m/s²]
    double timeout = 0.5;  ///< Max. age of the acceleration estimate [s]
  };

  struct BearingEstimate
  {
    Eigen::Vector3d direction;
    double off_axis;
    rclcpp::Time stamp;
    double angular_size = NAN;  ///< Angular size of the ROI [rad]
    double range_lower_bound = NAN;  ///< Target beyond the depth range: distance > this [m]
  };

  void declareControllerParameters();
  /// Forwards the `mpc.params.<name>` values (GCS "Control" tab) to Mpc::setParameter().
  void applyMpcParameters(const std::vector<rclcpp::Parameter> & params);
  void loadRuntimeParameters();
  rcl_interfaces::msg::SetParametersResult validateParameters(
    const std::vector<rclcpp::Parameter> & params);
  void onParametersSet(const std::vector<rclcpp::Parameter> & params);

  void onTarget(const nav_msgs::msg::Odometry & msg);
  void onBearing(const msg::TargetBearing & msg);
  /// Camera mount (camera_frame relative to camera_base_frame) from /tf_static (image metrics).
  void onStaticTf(const tf2_msgs::msg::TFMessage & msg);
  /// Target azimuth/elevation in a body-fixed camera image from the estimate and the measured
  /// attitude [rad]; NaN without odometry.
  std::pair<double, double> imageFeatures(const control::PursuitState & state) const;
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition & msg);
  void onOdometry(const px4_msgs::msg::VehicleOdometry & msg);
  void controlLoop();
  void runController(
    control::PursuitMode mode, std::optional<double> age, const std::optional<Eigen::Vector3d> & v_meas);

  std::optional<Eigen::Vector3d> measuredChaserVelocity() const;
  std::optional<control::VehicleState> measuredVehicleState() const;
  /// Heading sent to the PX4, ENU [rad]: ControlCommand::yaw, else the one held since the engagement.
  double yawEnu(const control::ControlCommand & cmd) const;
  void publishSetpoint(const control::ControlCommand & cmd);
  void publishPath(const control::ControlCommand & cmd, const rclcpp::Time & stamp);
  void publishStatus(
    const rclcpp::Time & now, std::optional<double> age, std::optional<double> bearing_age);
  void publishMetrics(
    const rclcpp::Time & now, const control::PursuitState & state,
    const control::ControlCommand & cmd, double solve_time_us);
  double losError(const Eigen::Vector3d & p_rel) const;

  // Configuration
  double dt_ = 0.05;
  double standoff_ = 5.0;
  control::Limits limits_;
  double velocity_timeout_ = 0.5;
  /// Safety floor [m] above the takeoff point (0: off). A target sinking to the ground (the X4 on
  /// the helix at 10 m/s) took the pursuer down with it (same altitude): it landed, PX4 disarmed.
  double min_altitude_ = 3.0;
  /// Safety [m/s²] (0: off): the MPC bounds each horizontal axis by a_max_xy, so a diagonal asked
  /// 8 x 8 = 11.3 m/s²; the x500 tilted 58-68 deg, lost lift and fell 10 m (helix, 10 m/s).
  double max_acc_xy_ = 8.0;
  double altitude_ = NAN;            ///< height above the takeoff point (-z of the local position) [m]
  double lost_timeout_ = 1.5;
  double bearing_timeout_ = 0.5;  ///< Max. age of the bounding box given to the law [s]
  std::string frame_id_;

  // Estado
  std::unique_ptr<control::Mpc> controller_;
  control::PursuitSupervisor supervisor_;
  control::PerformanceMetrics metrics_;
  control::CourseRateEstimator target_course_;
  std::optional<TargetEstimate> target_;
  std::optional<BearingEstimate> bearing_;
  Eigen::Vector3d v_ref_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_measured_ = Eigen::Vector3d::Zero();
  rclcpp::Time v_measured_stamp_;
  control::VehicleState odometry_;
  rclcpp::Time odometry_stamp_;
  double heading_ = NAN;  ///< NED heading measured by PX4 [rad]
  double hold_yaw_ = NAN;  ///< ENU heading latched at the engagement (law without yaw) [rad]
  bool enabled_ = true;
  bool runtime_pending_ = false;
  TargetAccelConfig accel_cfg_;
  std::optional<std::pair<Eigen::Vector3d, rclcpp::Time>> target_accel_;  ///< Absolute, ENU
  StaticTfChain static_tf_;               ///< /tf_static rotations (camera mount)
  double gimbal_pitch_ = 0.0;             ///< Measured camera gimbal pitch (+ = up) [rad]

  // Interfaces ROS
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_target_;
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr sub_target_accel_;
  rclcpp::Subscription<msg::TargetBearing>::SharedPtr sub_bearing_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_position_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_static_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_gimbal_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_setpoint_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<msg::PursuitStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<msg::PursuitMetrics>::SharedPtr pub_metrics_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr validate_param_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_param_handle_;
};

}  // namespace aero_pursuit
