#pragma once

#include <Eigen/Core>

#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

// Data model of a scenario (YAML file in <repo>/scenarios). Structures only:
// reading/writing in scenario_io.hpp, execution in agent_executor.hpp.
namespace aero_pursuit::scenario
{

/// Scenario error with the field path (e.g. "drones[1].plan[0].trajectory").
class ScenarioError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// Heading policy of a leg.
struct YawSpec
{
  enum class Mode { kVelocity, kFixed, kFace, kHold };
  Mode mode = Mode::kVelocity;
  double fixed = 0.0;      ///< kFixed: ENU heading [rad]
  std::string frame;       ///< kFace: frame a encarar
};

struct Waypoint
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double speed = 0.0;      ///< Speed to the next point (0: the trajectory's)
  double hold = 0.0;       ///< Wait at the point [s]
  /// type timed: time of this point from the start of the trajectory [s] (from a CSV t,x,y,z).
  std::optional<double> time;
};

/// Geometric trajectory. Shape fields shared between the types:
///   waypoints  points, interpolation (linear | spline), closed
///   line       points[0] -> points[1]; closed = ida e volta
///   polygon    center, radius, sides, heading
///   circle     center, radius, clockwise, phase
///   figure8    center, size (x: half width, y: half height, z: vertical oscillation), heading
///   lissajous  center, size (amplitudes), frequency, phase3
///   helix      center, radius, height (subida total, depois desce), turns, clockwise
///   random     center, size (half extents), count (points), seed: closed spline
///   hover      center (or current position), duration
///   roll/pitch tilt polygon, circle, figure8, lissajous, helix and random about the center:
///              R = Rz(heading) Ry(pitch) Rx(roll) (shape frame: x along its length, z up)
///   timed      points with a time each (t, x, y, z, e.g. from a CSV): the drone is at each point
///              at its time (variable speed); interpolation linear | spline
struct TrajectorySpec
{
  std::string type = "waypoints";
  bool relative = false;   ///< Coordinates relative to the drone position at the start of the leg
  double speed = 2.0;      ///< [m/s]
  std::vector<Waypoint> points;
  std::string interpolation = "linear";
  bool closed = false;
  Eigen::Vector3d center = Eigen::Vector3d(0.0, 0.0, 5.0);
  double radius = 10.0;
  Eigen::Vector3d size = Eigen::Vector3d(10.0, 5.0, 0.0);
  Eigen::Vector3d frequency = Eigen::Vector3d(1.0, 2.0, 0.0);
  Eigen::Vector3d phase3 = Eigen::Vector3d::Zero();  ///< [rad]
  double phase = 0.0;      ///< [rad]
  double heading = 0.0;    ///< Rotation of the shape around Z [rad]
  double roll = 0.0;       ///< Tilt of the shape about its own x axis [rad]
  double pitch = 0.0;      ///< Tilt of the shape about its own y axis [rad]
  int sides = 4;
  double height = 0.0;
  double turns = 1.0;
  bool clockwise = false;
  int count = 6;
  std::uint32_t seed = 0;
  double duration = 0.0;
};

/// Reactive behavior of the target (depends on the pursuer's state at run time):
///   flee    flees from the pursuer with lateral manoeuvres
///   random  smooth random walk inside the arena
///   orbit   circles the pursuer (radius `radius`, phase `phase`, relative height `offset.z`)
struct BehaviorSpec
{
  std::string type = "flee";
  std::string frame;       ///< Reference drone (empty: the pursuer)
  double speed = 3.0;
  double radius = 6.0;
  double phase = 0.0;
  double amplitude = 4.0;  ///< flee: lateral fraction
  double period = 6.0;     ///< flee: manoeuvre period [s]
  Eigen::Vector3d offset = Eigen::Vector3d::Zero();
  Eigen::Vector2d arena_center = Eigen::Vector2d::Zero();
  double arena_radius = 50.0;
  double min_altitude = 2.0;
  double max_altitude = 20.0;
  std::uint32_t seed = 0;
};

/// Leg of a path: follow a trajectory, hover, go to a point, or a reactive behavior.
struct LegSpec
{
  enum class Kind { kTrajectory, kHover, kGoto, kBehavior };
  Kind kind = Kind::kHover;
  std::string label;            ///< Displayed name (referenced trajectory, type...)
  TrajectorySpec trajectory;
  BehaviorSpec behavior;
  std::optional<Eigen::Vector3d> point;  ///< hover/goto
  double duration = 0.0;        ///< hover / behavior (0 in a behavior: no end)
  double speed = 0.0;           ///< goto / overrides the trajectory's (0: default)
  int loops = 1;                ///< Repetitions (0: infinite)
  std::optional<YawSpec> yaw;
};

/// The target drone (YAML `target:`): the one the pursuer chases.
struct DroneSpec
{
  std::string name = "X4";      ///< Model name in the simulator (= topic namespace)
  std::string model = "x4";     ///< Model catalog key (x3, x4, tiltrotor, x5)
  std::optional<Eigen::Vector3d> spawn;  ///< YAML `start`
  std::vector<LegSpec> plan;    ///< YAML `path`
  int plan_loops = 1;           ///< YAML `loops`: repetitions of the whole path (0: forever)
  YawSpec yaw;                  ///< Default for legs without `yaw`
  double max_speed = 6.0;
};

/// ROS parameter value applied at the start of each run.
using ParamValue = std::variant<bool, std::int64_t, double, std::string,
    std::vector<double>, std::vector<std::string>>;

struct PursuerSpec
{
  std::string frame = "x500_depth_0";
  std::optional<Eigen::Vector3d> start;  ///< Start position (ENU)
  double start_yaw = 0.0;       ///< [rad] (ignored with face_target)
  /// Points the camera at the target: during staging the start heading is the direction of the target start
  /// (staging only ends aligned) and, before an engagement without patrol, it follows the target in yaw.
  bool face_target = true;
  double yaw_tolerance = 3.0 * M_PI / 180.0;  ///< Heading error accepted during staging [rad]
  std::string target = "X4";
  std::string detection = "vision";      ///< vision | omniscient
  std::string acquisition = "auto";      ///< auto (ROI from ground truth) | manual
  bool reacquire = true;        ///< auto: restarts the tracker if it loses the target
  double engage_after = 0.0;    ///< Enables the pursuit after this time [s]
  bool engage_on_detection = false;  ///< ... or only when the tracker confirms
  std::vector<LegSpec> plan;    ///< Patrol before engagement (optional)
};

struct StagingSpec
{
  double timeout = 45.0;        ///< Maximum time for everyone to reach the start [s]
  double tolerance = 1.0;       ///< [m]
  double settle = 1.0;          ///< Wait after everyone arrives [s]
  /// The pursuer (PX4) must be this close to its start [m]: every run, and a
  /// GCS re-simulation of it, then begins from (almost) the same state.
  double pursuer_tolerance = 0.3;
  double max_speed = 0.15;      ///< Everyone at rest before the run [m/s]
};

/// A pursuit scenario (YAML in <repo>/scenarios; field names in scenario_io.cpp / the GCS editor):
/// one target drone that follows a path, one pursuer (PX4) that chases it.
struct Scenario
{
  std::string name = "scenario";
  std::string description;
  std::string world = "lawn";
  double duration = 60.0;       ///< RUNNING [simulation s]; 0: until the target path ends
  int repeat = 1;               ///< YAML `repetitions`
  double capture_radius = 0.0;  ///< YAML `capture_distance`: > 0 ends the run if this close [m]
  /// Speed of the target along its path [m/s]; 0 = the path's own speeds. The scenario_runner
  /// parameter `target_speed` (> 0) overrides it live.
  double target_speed = 0.0;
  StagingSpec staging;
  PursuerSpec pursuer;
  std::map<std::string, TrajectorySpec> trajectories;
  std::vector<DroneSpec> drones;     ///< The target (YAML `target:`), always exactly one
  std::map<std::string, std::map<std::string, ParamValue>> parameters;  ///< node -> name -> value
};

}  // namespace aero_pursuit::scenario
