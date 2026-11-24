#include "aero_pursuit/control/mpc.hpp"

// Generated at configure time by acados/generate.py (build/aero_pursuit/acados_generated/):
// default N/DT, sizes and default weights.
#include "pursuit_mpc_dims.hpp"
#include "acados_c/ocp_nlp_interface.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

namespace aero_pursuit::control
{

namespace pm = pursuit_mpc;
static_assert(pm::NX == 7 && pm::NU == 4 && pm::NY == 13 && pm::NY_E == 9,
  "mpc.cpp assumes x = [p_rel, v_rel, psi], u = [a, r], y = [x, bearing (2), u]");

namespace
{
constexpr int kPsi = 6;  // index of the heading in x (and in y)
constexpr int kR = 3;    // index of the yaw rate in u
constexpr int kBearing = 7;  // y[7], y[8]: cos(psi) - px/rho, sin(psi) - py/rho
constexpr int kInputs = 9;   // y[9..12]: a, r

/// yaw_mode with the visibility (field-of-view) constraint: 2 (LOS tracking + visibility) and
/// 4 (perception-aware + visibility).
bool fovMode(int mode) {return mode == 2 || mode == 4;}
constexpr double kTwoPi = 2.0 * M_PI;

double wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}
}  // namespace

/// One solver instance of a variant (RAII): creates its capsule from the function table, frees
/// it in the destructor. The handles are the acados types behind the void * of AcadosApi.
struct Mpc::Solver
{
  const AcadosApi & api;
  void * capsule = nullptr;
  ocp_nlp_config * config = nullptr;
  ocp_nlp_dims * dims = nullptr;
  ocp_nlp_in * in = nullptr;    // problem data: yref, W, bounds, parameters...
  ocp_nlp_out * out = nullptr;  // solution (and initial guess): x[0..N], u[0..N-1]

  explicit Solver(const AcadosApi & a)
  : api(a)
  {
    capsule = api.create_capsule();
    if (api.create(capsule) != 0) {
      api.free_capsule(capsule);
      throw std::runtime_error("acados: could not create the solver " + api.name);
    }
    config = static_cast<ocp_nlp_config *>(api.get_config(capsule));
    dims = static_cast<ocp_nlp_dims *>(api.get_dims(capsule));
    in = static_cast<ocp_nlp_in *>(api.get_in(capsule));
    out = static_cast<ocp_nlp_out *>(api.get_out(capsule));
  }

  ~Solver()
  {
    api.free_solver(capsule);
    api.free_capsule(capsule);
  }

  Solver(const Solver &) = delete;
  Solver & operator=(const Solver &) = delete;
};

Mpc::Mpc()
: solver_(std::make_unique<Solver>(defaultAcadosApi()))
{
  applyWeightsAndBounds();
}

Mpc::~Mpc() = default;

void Mpc::reset()
{
  // (Re)engagement: the previous plan belongs to another situation; start from a fresh guess.
  prev_x_.clear();
  prev_u_.clear();
}

void Mpc::setParameter(const std::string & name, double value)
{
  params_[name] = value;
  params_dirty_ = true;
  if (name == "N" || name == "dt") {
    selectVariant();  // starts compiling right away, even with the pursuit idle
  }
}

void Mpc::eraseParameter(const std::string & name)
{
  params_.erase(name);
  params_dirty_ = true;
  if (name == "N" || name == "dt") {
    selectVariant();
  }
}

double Mpc::param(const std::string & name, double fallback) const
{
  const auto it = params_.find(name);
  return it == params_.end() ? fallback : it->second;
}

int Mpc::horizonSteps() const {return solver_->api.N;}
double Mpc::horizonStep() const {return solver_->api.dt;}
int Mpc::yawMode() const
{
  return static_cast<int>(std::clamp(std::lround(param("yaw_mode", 0.0)), 0L, 4L));
}

std::string Mpc::label() const
{
  static const char * kYaw[] = {"no heading control", "LOS heading tracking",
    "LOS heading tracking + visibility constraint", "perception-aware (PAMPC)",
    "perception-aware + visibility constraint"};
  std::string text = "MPC N=" + std::to_string(solver_->api.N) + ", dt=" +
    std::to_string(solver_->api.dt).substr(0, 5) + " s, heading: " + kYaw[yawMode()];
  if (variants_.building()) {
    text += " (compiling " + variants_.buildingName() + ")";
  }
  return text;
}

std::vector<std::string> Mpc::takeMessages()
{
  std::vector<std::string> m;
  m.swap(messages_);
  return m;
}

void Mpc::selectVariant()
{
  // Requested (N, dt): integer N, dt rounded to 1 ms (the variant name uses milliseconds).
  const int n = static_cast<int>(std::clamp(std::lround(param("N", pm::N)), 2L, 200L));
  const double dt = std::clamp(std::round(param("dt", pm::DT) * 1000.0) / 1000.0, 0.005, 1.0);
  const bool finished = variants_.poll();  // a background build that just finished
  if (const auto m = variants_.takeMessage(); !m.empty()) {
    messages_.push_back(m);
  }
  if (n == solver_->api.N && std::abs(dt - solver_->api.dt) < 1e-9) {
    return;  // already active
  }
  const auto & def = defaultAcadosApi();
  const AcadosApi * api = (n == def.N && std::abs(dt - def.dt) < 1e-9) ? &def :
    variants_.request(n, dt);
  if (const auto m = variants_.takeMessage(); !m.empty()) {
    messages_.push_back(m);
  }
  if (!api) {
    return;  // being compiled (the current solver keeps running) or unavailable
  }
  solver_ = std::make_unique<Solver>(*api);  // the old instance is freed here
  prev_x_.clear();                            // the old plan has another N/dt
  prev_u_.clear();
  params_dirty_ = true;                       // weights/bounds go into the new solver
  messages_.push_back("MPC now with N=" + std::to_string(api->N) + ", dt=" +
    std::to_string(api->dt).substr(0, 5) + " s (" + api->name + (finished ? ", just compiled)" : ")"));
}

void Mpc::applyWeightsAndBounds()
{
  const int N = solver_->api.N;
  auto * config = solver_->config;
  auto * dims = solver_->dims;
  auto * in = solver_->in;
  auto * out = solver_->out;
  const int yaw_mode = yawMode();

  // --- Cost weights (diagonal) of the residuals y = [p, v, psi, bearing (2), a, r]:
  //   mode 0: no heading terms (the heading is not the MPC's); 1, 2 (LOS heading tracking): psi
  //   against psi_ref; 3, 4 (perception-aware): the bearing residuals (relative heading optimized
  //   by the solver).
  const double q_psi = param("q_psi", pm::Q_PSI);
  std::array<double, pm::NY> wy = {
    param("q_px", pm::Q_P[0]), param("q_py", pm::Q_P[1]), param("q_pz", pm::Q_P[2]),
    param("q_vx", pm::Q_V[0]), param("q_vy", pm::Q_V[1]), param("q_vz", pm::Q_V[2]),
    (yaw_mode == 1 || yaw_mode == 2) ? q_psi : 0.0,
    yaw_mode >= 3 ? q_psi : 0.0, yaw_mode >= 3 ? q_psi : 0.0,
    param("r_ax", pm::R_A[0]), param("r_ay", pm::R_A[1]), param("r_az", pm::R_A[2]),
    param("r_r", pm::R_R)};
  const double q_f = param("q_f", pm::Q_F);
  std::array<double, pm::NY * pm::NY> W{};  // column-major: element (i, j) at [j * NY + i]
  for (int i = 0; i < pm::NY; ++i) {
    W[i * pm::NY + i] = wy[i];
  }
  std::array<double, pm::NY_E * pm::NY_E> W_e{};  // terminal: the state residuals only
  for (int i = 0; i < pm::NY_E; ++i) {
    W_e[i * pm::NY_E + i] = q_f * wy[i];
  }
  for (int k = 0; k < N; ++k) {
    ocp_nlp_cost_model_set(config, dims, in, k, "W", W.data());
  }
  ocp_nlp_cost_model_set(config, dims, in, N, "W", W_e.data());

  // --- Input box: |a_x|, |a_y| <= a_max_xy, |a_z| <= a_max_z, |r| <= r_max.
  const double a_xy = param("a_max_xy", pm::A_MAX[0]);
  const double a_z = param("a_max_z", pm::A_MAX[2]);
  const double r_max = param("r_max", pm::R_MAX);
  std::array<double, pm::NU> ubu = {a_xy, a_xy, a_z, r_max};
  std::array<double, pm::NU> lbu = {-a_xy, -a_xy, -a_z, -r_max};
  for (int k = 0; k < N; ++k) {
    ocp_nlp_constraints_model_set(config, dims, in, out, k, "lbu", lbu.data());
    ocp_nlp_constraints_model_set(config, dims, in, out, k, "ubu", ubu.data());
  }

  // --- Field of view (stages 1..N; stage 0 has none): h(x, c) <= 0 in yaw_mode 2, 4, else open.
  const double half = (param("fov_deg", pm::FOV_DEG) / 2.0 - param("fov_margin_deg", pm::FOV_MARGIN_DEG)) *
    M_PI / 180.0;
  std::array<double, pm::NP> c = {std::tan(std::clamp(half, 0.02, 1.5))};
  for (int k = 0; k <= N; ++k) {
    solver_->api.update_params(solver_->capsule, k, c.data(), pm::NP);
  }
  std::array<double, pm::NH> lh, uh;
  lh.fill(-pm::INFTY);
  uh.fill(fovMode(yaw_mode) ? 0.0 : pm::INFTY);
  const double w = param("w_fov", pm::W_FOV);
  std::array<double, pm::NH> slack_w;
  slack_w.fill(w);
  for (int k = 1; k <= N; ++k) {
    ocp_nlp_constraints_model_set(config, dims, in, out, k, "lh", lh.data());
    ocp_nlp_constraints_model_set(config, dims, in, out, k, "uh", uh.data());
    for (const char * field : {"zl", "zu", "Zl", "Zu"}) {  // L1 and L2 slack penalties
      ocp_nlp_cost_model_set(config, dims, in, k, field, slack_w.data());
    }
  }
  params_dirty_ = false;
}

std::vector<Mpc::StateVec> Mpc::warmStart(const StateVec & x0)
{
  const int N = solver_->api.N;
  const double DT = solver_->api.dt;
  std::vector<StateVec> xs(N + 1);
  std::vector<InputVec> us(N);
  const bool have_plan = static_cast<int>(prev_x_.size()) == N + 1 &&
    static_cast<int>(prev_u_.size()) == N;
  if (have_plan) {
    // Shift one step: guess x[k] = previous x[k+1] (the last one repeated). The heading of the
    // old plan is moved by a multiple of 2 pi next to the measured one (a jump of 2 pi between
    // the old plan and the new measurement would make the solver turn the long way).
    const double shift = kTwoPi * std::round((x0[kPsi] - prev_x_[std::min(1, N)][kPsi]) / kTwoPi);
    for (int k = 0; k <= N; ++k) {
      xs[k] = prev_x_[std::min(k + 1, N)];
      xs[k][kPsi] += shift;
    }
    for (int k = 0; k < N; ++k) {
      us[k] = prev_u_[std::min(k + 1, N - 1)];
    }
  } else {
    // No plan: the relative motion at constant velocity, heading kept, no input.
    for (int k = 0; k <= N; ++k) {
      for (int i = 0; i < 3; ++i) {
        xs[k][i] = x0[i] + x0[3 + i] * k * DT;
        xs[k][3 + i] = x0[3 + i];
      }
      xs[k][kPsi] = x0[kPsi];
    }
    for (auto & u : us) {
      u.fill(0.0);
    }
  }
  xs[0] = x0;
  auto * s = solver_.get();
  for (int k = 0; k <= N; ++k) {
    ocp_nlp_out_set(s->config, s->dims, s->out, s->in, k, "x", xs[k].data());
  }
  for (int k = 0; k < N; ++k) {
    ocp_nlp_out_set(s->config, s->dims, s->out, s->in, k, "u", us[k].data());
  }
  return xs;
}

ControlCommand Mpc::compute(const PursuitState & state, double dt)
{
  selectVariant();
  ControlCommand cmd;  // velocity = acceleration = 0, no yaw
  if (!state.has_position) {
    // APPROACH (no depth, only state.box): no 3D state for the model. Placeholder: hover.
    return cmd;
  }
  if (params_dirty_) {
    applyWeightsAndBounds();
  }
  const int N = solver_->api.N;
  const double DT = solver_->api.dt;
  auto * config = solver_->config;
  auto * dims = solver_->dims;
  auto * in = solver_->in;
  auto * out = solver_->out;
  const bool heading_known = std::isfinite(state.heading);
  const int yaw_mode = heading_known ? yawMode() : 0;

  // 1) Initial state x0 = [p_rel; v_rel; psi]: equality constraint lbx_0 = ubx_0 = x0.
  const StateVec x0 = {
    state.p_rel.x(), state.p_rel.y(), state.p_rel.z(),
    state.v_rel.x(), state.v_rel.y(), state.v_rel.z(),
    heading_known ? state.heading : 0.0};
  ocp_nlp_constraints_model_set(config, dims, in, out, 0, "lbx", const_cast<double *>(x0.data()));
  ocp_nlp_constraints_model_set(config, dims, in, out, 0, "ubx", const_cast<double *>(x0.data()));

  // 2) Initial guess (warm start) = previous plan shifted. Its positions are where the target
  //    is expected to be seen; SQP-RTI also linearizes the field-of-view constraint around it.
  const auto guess = warmStart(x0);

  // 3) Position reference: target `standoff` metres ahead along the current line of sight
  //    (horizontal in ref_horizontal mode), same velocity. Fixed over the horizon.
  Eigen::Vector3d dir = state.p_rel;
  if (param("ref_horizontal", 1.0) > 0.5) {
    dir.z() = 0.0;
  }
  const double range = dir.norm();
  const Eigen::Vector3d los = range > 1e-6 ? Eigen::Vector3d(dir / range) :
    Eigen::Vector3d::UnitX();
  const Eigen::Vector3d p_ref = state.standoff * los;

  // 4) Heading reference psi_ref[k] = atan2 of the guessed p_rel[k] (DATA for the solver: the
  //    problem stays linear), unwrapped along the horizon starting next to the measured heading.
  std::vector<double> psi_ref(N + 1);
  double prev = x0[kPsi];
  for (int k = 0; k <= N; ++k) {
    const double px = guess[k][0], py = guess[k][1];
    const double bearing = std::hypot(px, py) > 0.3 ? std::atan2(py, px) : prev;
    psi_ref[k] = prev + wrapAngle(bearing - prev);   // the nearest copy of bearing (unwrap)
    prev = psi_ref[k];
  }

  // 5) yref per stage: [p_ref; v_rel = 0; psi_ref[k]; bearing = 0 (aligned); a = 0; r = 0],
  //    terminal: the first NY_E entries. psi_ref only matters in modes 1, 2 (weight 0 otherwise).
  for (int k = 0; k <= N; ++k) {
    std::array<double, pm::NY> yref{};
    for (int i = 0; i < 3; ++i) {
      yref[i] = p_ref(i);
    }
    yref[kPsi] = psi_ref[k];
    static_assert(kBearing == pm::NX && kInputs == pm::NX + 2, "residual layout");
    ocp_nlp_cost_model_set(config, dims, in, k, "yref", yref.data());  // NY_E prefix at k = N
  }

  // 6) Solve: one SQP-RTI iteration (the exact QP without the field of view; with it, the QP of
  //    the constraint linearized at the guess).
  status_ = solver_->api.solve(solver_->capsule);
  const Eigen::Vector3d v_chaser = state.v_target - state.v_rel;  // current pursuer velocity
  if (status_ != 0) {
    prev_x_.clear();
    prev_u_.clear();
    cmd.velocity = v_chaser;  // keep the current velocity, no acceleration, heading held
    return cmd;
  }
  prev_x_.resize(N + 1);
  prev_u_.resize(N);
  for (int k = 0; k <= N; ++k) {
    ocp_nlp_out_get(config, dims, out, k, "x", prev_x_[k].data());
  }
  for (int k = 0; k < N; ++k) {
    ocp_nlp_out_get(config, dims, out, k, "u", prev_u_[k].data());
  }

  // 7) Apply the first input: acceleration and the velocity it leads to after one node cycle;
  //    heading (yaw_mode >= 1) where r_0 takes it after one cycle, with r_0 as feedforward.
  const auto & u0 = prev_u_[0];
  cmd.acceleration = Eigen::Vector3d(u0[0], u0[1], u0[2]);
  cmd.velocity = v_chaser + cmd.acceleration * dt;
  if (yaw_mode >= 1) {
    cmd.yaw = wrapAngle(x0[kPsi] + u0[kR] * dt);
    cmd.yaw_rate = u0[kR];
  }

  // 8) Predicted pursuer path relative to now: p_c[k] - p_c[0] = v_target k DT - (p_rel[k] - p_rel[0]).
  cmd.predicted_path.reserve(N);
  for (int k = 1; k <= N; ++k) {
    const Eigen::Vector3d p_rel_k(prev_x_[k][0], prev_x_[k][1], prev_x_[k][2]);
    cmd.predicted_path.push_back(state.v_target * (k * DT) - (p_rel_k - state.p_rel));
  }
  return cmd;
}

}  // namespace aero_pursuit::control
