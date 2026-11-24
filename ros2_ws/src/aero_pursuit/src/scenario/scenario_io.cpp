#include "aero_pursuit/scenario/scenario_io.hpp"


#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <functional>
#include <regex>
#include <set>
#include <sstream>

namespace aero_pursuit::scenario
{
namespace
{

constexpr double kDeg = M_PI / 180.0;

/// Reading context: accumulates the field path for error messages.
class Reader
{
public:
  Reader(const YAML::Node & node, std::string path) : node_(node), path_(std::move(path)) {}

  const YAML::Node & node() const {return node_;}
  const std::string & path() const {return path_;}
  bool has(const std::string & key) const {return node_.IsMap() && node_[key].IsDefined() && !node_[key].IsNull();}
  Reader child(const std::string & key) const {return Reader(node_[key], path_ + "." + key);}
  Reader item(std::size_t i) const {return Reader(node_[i], path_ + "[" + std::to_string(i) + "]");}

  [[noreturn]] void fail(const std::string & what) const
  {
    throw ScenarioError((path_.empty() ? std::string("(raiz)") : path_) + ": " + what);
  }

  template<typename T>
  T as() const
  {
    try {
      return node_.as<T>();
    } catch (const YAML::Exception &) {
      fail("invalid value '" + YAML::Dump(node_) + "'");
    }
  }

  template<typename T>
  T get(const std::string & key, const T & fallback) const
  {
    return has(key) ? child(key).as<T>() : fallback;
  }

  double angle(const std::string & key, double fallback_rad) const
  {
    return has(key) ? child(key).as<double>() * kDeg : fallback_rad;
  }

  Eigen::Vector3d vec3(double default_z = 0.0) const
  {
    if (!node_.IsSequence() || node_.size() < 2 || node_.size() > 3) {
      fail("expected [x, y] or [x, y, z]");
    }
    return {item(0).as<double>(), item(1).as<double>(),
      node_.size() == 3 ? item(2).as<double>() : default_z};
  }

  Eigen::Vector3d vec3(const std::string & key, const Eigen::Vector3d & fallback) const
  {
    return has(key) ? child(key).vec3(fallback.z()) : fallback;
  }

  /// Warns about unknown keys (typos would pass silently).
  void allowOnly(const std::set<std::string> & keys) const
  {
    if (!node_.IsMap()) {
      fail("expected a map");
    }
    for (const auto & kv : node_) {
      const auto key = kv.first.as<std::string>();
      if (!keys.count(key)) {
        Reader(kv.second, path_ + "." + key).fail("unknown field");
      }
    }
  }

private:
  YAML::Node node_;
  std::string path_;
};

YawSpec parseYaw(const Reader & r)
{
  YawSpec y;
  const auto text = r.as<std::string>();
  if (text == "velocity") {
    y.mode = YawSpec::Mode::kVelocity;
  } else if (text == "hold") {
    y.mode = YawSpec::Mode::kHold;
  } else if (text.rfind("face:", 0) == 0 && text.size() > 5) {
    y.mode = YawSpec::Mode::kFace;
    y.frame = text.substr(5);
  } else {
    try {
      y.mode = YawSpec::Mode::kFixed;
      y.fixed = std::stod(text) * kDeg;
    } catch (const std::exception &) {
      r.fail("yaw: velocity | hold | face:<frame> | <degrees>");
    }
  }
  return y;
}

const std::set<std::string> kTrajectoryTypes = {
  "waypoints", "line", "polygon", "circle", "figure8", "lissajous", "helix", "random", "hover", "timed"};

TrajectorySpec parseTrajectory(const Reader & r)
{
  r.allowOnly({"type", "relative", "speed", "points", "interpolation", "closed", "center", "radius",
      "size", "frequency", "phase_deg", "phase3_deg", "heading_deg", "roll_deg", "pitch_deg", "sides", "height", "turns",
      "clockwise", "count", "seed", "duration", "description"});
  TrajectorySpec t;
  t.type = r.get<std::string>("type", t.type);
  if (!kTrajectoryTypes.count(t.type)) {
    r.child("type").fail("unknown type (waypoints, line, polygon, circle, figure8, lissajous, "
      "helix, random, hover, timed)");
  }
  t.relative = r.get<bool>("relative", t.relative);
  t.speed = r.get<double>("speed", t.speed);
  if (!(t.speed > 0.0)) {
    r.child("speed").fail("must be > 0");
  }
  if (r.has("points")) {
    const Reader pts = r.child("points");
    if (!pts.node().IsSequence()) {
      pts.fail("expected a list of points");
    }
    for (std::size_t i = 0; i < pts.node().size(); ++i) {
      const Reader p = pts.item(i);
      Waypoint w;
      if (p.node().IsMap()) {
        p.allowOnly({"p", "speed", "hold", "t"});
        if (!p.has("p")) {
          p.fail("point without 'p: [x, y, z]'");
        }
        w.position = p.child("p").vec3(t.center.z());
        w.speed = p.get<double>("speed", 0.0);
        w.hold = p.get<double>("hold", 0.0);
        if (p.has("t")) {
          w.time = p.child("t").as<double>();
        }
      } else if (t.type == "timed" && p.node().IsSequence() && p.node().size() == 4) {
        // timed shortcut: [t, x, y, z] (one CSV row)
        w.time = p.node()[0].as<double>();
        w.position = Eigen::Vector3d(p.node()[1].as<double>(), p.node()[2].as<double>(),
            p.node()[3].as<double>());
      } else {
        w.position = p.vec3(t.center.z());
      }
      t.points.push_back(w);
    }
  }
  t.interpolation = r.get<std::string>("interpolation", t.interpolation);
  if (t.interpolation != "linear" && t.interpolation != "spline") {
    r.child("interpolation").fail("linear | spline");
  }
  t.closed = r.get<bool>("closed", t.closed);
  t.center = r.vec3("center", t.center);
  t.radius = r.get<double>("radius", t.radius);
  t.size = r.vec3("size", t.size);
  t.frequency = r.vec3("frequency", t.frequency);
  t.phase = r.angle("phase_deg", t.phase);
  t.phase3 = r.vec3("phase3_deg", t.phase3 / kDeg) * kDeg;
  t.heading = r.angle("heading_deg", t.heading);
  t.roll = r.angle("roll_deg", t.roll);
  t.pitch = r.angle("pitch_deg", t.pitch);
  t.sides = r.get<int>("sides", t.sides);
  t.height = r.get<double>("height", t.height);
  t.turns = r.get<double>("turns", t.turns);
  t.clockwise = r.get<bool>("clockwise", t.clockwise);
  t.count = r.get<int>("count", t.count);
  t.seed = r.get<std::uint32_t>("seed", t.seed);
  t.duration = r.get<double>("duration", t.duration);

  const std::size_t min_points = t.type == "line" ? 2 : 1;
  if ((t.type == "waypoints" || t.type == "line") && t.points.size() < min_points) {
    r.fail(t.type + " needs at least " + std::to_string(min_points) + " point(s) in 'points'");
  }
  if ((t.type == "circle" || t.type == "polygon" || t.type == "helix") && !(t.radius > 0.0)) {
    r.child("radius").fail("must be > 0");
  }
  if (t.type == "polygon" && t.sides < 3) {
    r.child("sides").fail("must be >= 3");
  }
  if (t.type == "random" && t.count < 3) {
    r.child("count").fail("must be >= 3");
  }
  if (t.type == "timed") {
    if (t.points.size() < 2) {
      r.fail("timed needs at least 2 points [t, x, y, z]");
    }
    for (std::size_t i = 0; i < t.points.size(); ++i) {
      if (!t.points[i].time) {
        r.child("points").item(i).fail("timed: every point needs a time (t)");
      }
      if (i > 0 && !(*t.points[i].time > *t.points[i - 1].time)) {
        r.child("points").item(i).fail("timed: times must increase");
      }
    }
  }
  return t;
}

const std::set<std::string> kBehaviorTypes = {"flee", "random", "orbit"};
const std::set<std::string> kBehaviorKeys = {"radius", "phase_deg", "amplitude", "period",
  "height_offset", "arena_center", "arena_radius", "min_altitude", "max_altitude", "seed"};

BehaviorSpec parseBehavior(const Reader & leg, const std::string & type)
{
  if (!kBehaviorTypes.count(type)) {
    leg.child("behavior").fail("unknown behavior (flee, random, orbit)");
  }
  BehaviorSpec b;
  b.type = type;
  b.speed = leg.get<double>("speed", b.speed);
  b.radius = leg.get<double>("radius", b.radius);
  b.phase = leg.angle("phase_deg", b.phase);
  b.amplitude = leg.get<double>("amplitude", b.amplitude);
  b.period = leg.get<double>("period", b.period);
  b.offset.z() = leg.get<double>("height_offset", b.offset.z());
  if (leg.has("arena_center")) {
    b.arena_center = leg.child("arena_center").vec3().head<2>();
  }
  b.arena_radius = leg.get<double>("arena_radius", b.arena_radius);
  b.min_altitude = leg.get<double>("min_altitude", b.min_altitude);
  b.max_altitude = leg.get<double>("max_altitude", b.max_altitude);
  b.seed = leg.get<std::uint32_t>("seed", b.seed);
  return b;
}

LegSpec parseLeg(const Reader & r, const std::map<std::string, TrajectorySpec> & library);

std::vector<LegSpec> parsePlan(const Reader & r, const std::map<std::string, TrajectorySpec> & library)
{
  if (!r.node().IsSequence()) {
    r.fail("a path must be a list of legs");
  }
  std::vector<LegSpec> legs;
  for (std::size_t i = 0; i < r.node().size(); ++i) {
    legs.push_back(parseLeg(r.item(i), library));
  }
  return legs;
}

LegSpec parseLeg(const Reader & r, const std::map<std::string, TrajectorySpec> & library)
{
  LegSpec leg;
  auto reference = [&](const Reader & ref) {
      const auto name = ref.as<std::string>();
      const auto it = library.find(name);
      if (it == library.end()) {
        ref.fail("trajectory '" + name + "' not defined in 'trajectories'");
      }
      leg.kind = LegSpec::Kind::kTrajectory;
      leg.trajectory = it->second;
      leg.label = name;
    };
  if (r.node().IsScalar()) {  // "- oito": shortcut for {trajectory: oito}
    reference(r);
    return leg;
  }
  if (!r.node().IsMap()) {
    r.fail("invalid leg");
  }
  std::set<std::string> keys = {"trajectory", "hover", "goto", "behavior",
    "at", "duration", "speed", "loops", "yaw", "label"};
  keys.insert(kBehaviorKeys.begin(), kBehaviorKeys.end());
  r.allowOnly(keys);

  int kinds = 0;
  for (const char * k : {"trajectory", "hover", "goto", "behavior"}) {
    kinds += r.has(k) ? 1 : 0;
  }
  if (kinds != 1) {
    r.fail("each leg must have exactly one of: trajectory, hover, goto, behavior");
  }
  leg.speed = r.get<double>("speed", 0.0);
  leg.loops = r.get<int>("loops", 1);
  if (leg.loops < 0) {
    r.child("loops").fail("must be >= 0 (0: forever)");
  }
  if (r.has("yaw")) {
    leg.yaw = parseYaw(r.child("yaw"));
  }
  if (r.has("trajectory")) {
    const Reader t = r.child("trajectory");
    if (t.node().IsScalar()) {
      reference(t);
    } else {
      leg.kind = LegSpec::Kind::kTrajectory;
      leg.trajectory = parseTrajectory(t);
      leg.label = leg.trajectory.type;
    }
    if (leg.speed > 0.0) {
      leg.trajectory.speed = leg.speed;
    }
  } else if (r.has("hover")) {
    leg.kind = LegSpec::Kind::kHover;
    leg.duration = r.child("hover").as<double>();
    if (r.has("at")) {
      leg.point = r.child("at").vec3();
    }
    leg.label = "hover";
  } else if (r.has("goto")) {
    leg.kind = LegSpec::Kind::kGoto;
    leg.point = r.child("goto").vec3();
    leg.label = "goto";
  } else {
    leg.kind = LegSpec::Kind::kBehavior;
    leg.behavior = parseBehavior(r, r.child("behavior").as<std::string>());
    leg.duration = r.get<double>("duration", 0.0);
    leg.label = leg.behavior.type;
  }
  leg.label = r.get<std::string>("label", leg.label);
  return leg;
}

ParamValue parseParamValue(const Reader & r)
{
  const YAML::Node & n = r.node();
  if (n.IsSequence()) {
    std::vector<double> numbers;
    std::vector<std::string> strings;
    bool numeric = true;
    for (const auto & e : n) {
      strings.push_back(e.as<std::string>());
      try {
        numbers.push_back(e.as<double>());
      } catch (const YAML::Exception &) {
        numeric = false;
      }
    }
    if (numeric && !strings.empty()) {
      return numbers;
    }
    return strings;
  }
  if (!n.IsScalar()) {
    r.fail("a parameter value must be a scalar or a list");
  }
  const auto text = n.Scalar();
  if (n.Tag() == "!") {  // Entre aspas: sempre texto
    return text;
  }
  if (text == "true" || text == "false") {
    return text == "true";
  }
  static const std::regex integer(R"(^[-+]?\d+$)");
  if (std::regex_match(text, integer)) {
    return static_cast<std::int64_t>(std::stoll(text));
  }
  try {
    std::size_t used = 0;
    const double d = std::stod(text, &used);
    if (used == text.size()) {
      return d;
    }
  } catch (const std::exception &) {
  }
  return text;
}

DroneSpec parseTarget(const Reader & r, const std::map<std::string, TrajectorySpec> & library)
{
  r.allowOnly({"name", "model", "start", "path", "loops", "yaw", "max_speed"});
  DroneSpec d;
  d.name = r.get<std::string>("name", d.name);
  d.model = r.get<std::string>("model", d.model);
  if (r.has("start")) {
    d.spawn = r.child("start").vec3(1.0);
  }
  if (r.has("path")) {
    d.plan = parsePlan(r.child("path"), library);
  }
  d.plan_loops = r.get<int>("loops", d.plan_loops);
  if (d.plan_loops < 0) {
    r.child("loops").fail("must be >= 0 (0: forever)");
  }
  if (r.has("yaw")) {
    d.yaw = parseYaw(r.child("yaw"));
  }
  d.max_speed = r.get<double>("max_speed", d.max_speed);
  return d;
}


}  // namespace

bool isValidEntityName(const std::string & name)
{
  static const std::regex valid(R"(^[A-Za-z][A-Za-z0-9_]{0,62}$)");
  return std::regex_match(name, valid);
}

Scenario parseScenario(const std::string & yaml_text)
{
  YAML::Node root;
  try {
    root = YAML::Load(yaml_text);
  } catch (const YAML::Exception & e) {
    throw ScenarioError(std::string("invalid YAML: ") + e.what());
  }
  const Reader r(root, "");
  r.allowOnly({"name", "description", "world", "duration", "repetitions", "capture_distance",
      "target_speed", "target", "pursuer", "trajectories", "staging", "parameters"});

  Scenario s;
  s.name = r.get<std::string>("name", s.name);
  if (!std::regex_match(s.name, std::regex(R"(^[A-Za-z0-9_\-]{1,64}$)"))) {
    r.child("name").fail("use letters, digits, '_' or '-'");
  }
  s.description = r.get<std::string>("description", s.description);
  s.world = r.get<std::string>("world", s.world);
  s.duration = r.get<double>("duration", s.duration);
  s.repeat = std::max(1, r.get<int>("repetitions", s.repeat));
  s.capture_radius = r.get<double>("capture_distance", s.capture_radius);
  s.target_speed = r.get<double>("target_speed", s.target_speed);
  if (s.target_speed < 0.0 || s.target_speed > 30.0) {
    r.child("target_speed").fail("use a value in [0, 30] m/s (0 = the path's own speeds)");
  }

  if (r.has("staging")) {
    const Reader st = r.child("staging");
    st.allowOnly({"timeout", "tolerance", "settle", "pursuer_tolerance", "max_speed"});
    s.staging.timeout = st.get<double>("timeout", s.staging.timeout);
    s.staging.tolerance = st.get<double>("tolerance", s.staging.tolerance);
    s.staging.settle = st.get<double>("settle", s.staging.settle);
    s.staging.pursuer_tolerance = st.get<double>("pursuer_tolerance", s.staging.pursuer_tolerance);
    s.staging.max_speed = st.get<double>("max_speed", s.staging.max_speed);
  }

  if (r.has("trajectories")) {
    const Reader lib = r.child("trajectories");
    if (!lib.node().IsMap()) {
      lib.fail("expected a map name -> trajectory");
    }
    for (const auto & kv : lib.node()) {
      const auto name = kv.first.as<std::string>();
      s.trajectories[name] = parseTrajectory(lib.child(name));
    }
  }

  // The target: exactly one drone (default X4 hovering at its start if `target:` is missing).
  s.drones.push_back(r.has("target") ? parseTarget(r.child("target"), s.trajectories) : DroneSpec{});
  const auto & target = s.drones.front();
  if (!isValidEntityName(target.name)) {
    r.child("target").child("name").fail(
      "'" + target.name + "' invalid (a letter followed by letters, digits or '_')");
  }

  if (r.has("pursuer")) {
    const Reader p = r.child("pursuer");
    p.allowOnly({"start", "heading_deg", "face_target", "heading_tolerance_deg", "detection",
        "acquisition", "reacquire", "start_after", "wait_for_detection", "patrol"});
    auto & ps = s.pursuer;
    if (p.has("start")) {
      ps.start = p.child("start").vec3(5.0);
    }
    ps.start_yaw = p.angle("heading_deg", ps.start_yaw);
    ps.face_target = p.get<bool>("face_target", ps.face_target);
    ps.yaw_tolerance = p.angle("heading_tolerance_deg", ps.yaw_tolerance);
    if (!(ps.yaw_tolerance >= 0.5 * M_PI / 180.0)) {
      p.child("heading_tolerance_deg").fail("must be >= 0.5 deg (0 can never be reached: staging "
        "would wait until its timeout)");
    }
    ps.detection = p.get<std::string>("detection", ps.detection);
    if (ps.detection != "vision" && ps.detection != "omniscient") {
      p.child("detection").fail("vision | omniscient");
    }
    ps.acquisition = p.get<std::string>("acquisition", ps.acquisition);
    if (ps.acquisition != "auto" && ps.acquisition != "manual") {
      p.child("acquisition").fail("auto | manual");
    }
    ps.reacquire = p.get<bool>("reacquire", ps.reacquire);
    ps.engage_after = p.get<double>("start_after", ps.engage_after);
    ps.engage_on_detection = p.get<bool>("wait_for_detection", ps.engage_on_detection);
    if (p.has("patrol")) {
      ps.plan = parsePlan(p.child("patrol"), s.trajectories);
    }
  }
  s.pursuer.target = target.name;   // the pursuer always chases the scenario target
  if (target.name == s.pursuer.frame) {
    r.child("target").child("name").fail("'" + target.name + "' is the pursuer's name");
  }

  if (r.has("parameters")) {
    const Reader params = r.child("parameters");
    for (const auto & node_kv : params.node()) {
      auto node = node_kv.first.as<std::string>();
      const Reader pn = params.child(node);
      if (node.front() != '/') {
        node = "/" + node;
      }
      for (const auto & kv : pn.node()) {
        const auto name = kv.first.as<std::string>();
        s.parameters[node][name] = parseParamValue(pn.child(name));
      }
    }
  }
  return s;
}

TrajectorySpec parseTrajectoryText(const std::string & yaml_text)
{
  YAML::Node root;
  try {
    root = YAML::Load(yaml_text);
  } catch (const YAML::Exception & e) {
    throw ScenarioError(std::string("invalid YAML: ") + e.what());
  }
  return parseTrajectory(Reader(root, "trajectory"));
}

Scenario loadScenarioFile(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    throw ScenarioError("could not open " + path);
  }
  std::stringstream ss;
  ss << in.rdbuf();
  return parseScenario(ss.str());
}

}  // namespace aero_pursuit::scenario
