#pragma once

#include "aero_pursuit/control/acados_variants.hpp"
#include "aero_pursuit/control/pursuit_types.hpp"

#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace aero_pursuit::control
{

/// Pursuit MPC (src/control/mpc.cpp), solved by acados (problem in acados/pursuit_ocp.py).
///
/// Called by PursuitControllerNode::runController() every `dt` (20 Hz) while the supervisor is
/// TRACKING, COASTING or APPROACH. Everything comes in PursuitState and leaves in ControlCommand
/// (pursuit_controller.hpp); nothing here depends on ROS.
///
/// Model (7 states, ENU, discrete step dt of the active solver):
///   x = [p_rel; v_rel; psi],  u = [a_chaser; r],  x[k+1] = A x[k] + B u[k]
///   (target at constant velocity; psi = pursuer heading, decoupled from the translation)
/// Cost (NONLINEAR_LS): position error to the standoff point, relative velocity -> 0, heading
/// error (yaw_mode >= 1: to the predicted line of sight, or the relative heading inside the
/// solver), acceleration and yaw-rate effort.
/// Constraints: |a_i| <= a_max, |r| <= r_max; yaw_mode 2, 4: target inside the camera's
/// horizontal field of view (soft, nonlinear, linearized by SQP-RTI).
///
/// yaw_mode (mpc.params.yaw_mode):
///   0 = no heading control (heading held by the node, no yaw command);
///   1 = LOS heading tracking: psi follows the line-of-sight bearing atan2 of the previous plan
///       (a reference given as data, so the problem stays a QP);
///   2 = 1 + visibility constraint (target inside the horizontal field of view; Allibert 2010);
///   3 = perception-aware (PAMPC, Falanga 2018): the relative heading is optimized inside the
///       solver, cost on cos(psi) - px/rho, sin(psi) - py/rho (the 1 - cos metric of Lee 2010);
///   4 = 3 + visibility constraint.
///
/// GCS numbers (`mpc.params.<name>`, setParameter): q_px q_py q_pz, q_vx q_vy q_vz, r_ax r_ay r_az,
/// q_f, a_max_xy, a_max_z, ref_horizontal, N, dt, yaw_mode, q_psi, r_r, r_max, fov_deg,
/// fov_margin_deg, w_fov. Unset names use the defaults compiled from pursuit_ocp.py.
class Mpc
{
public:
  Mpc();
  ~Mpc();
  Mpc(const Mpc &) = delete;
  Mpc & operator=(const Mpc &) = delete;

  std::string name() const {return "mpc";}
  /// Name + active configuration, shown by the GCS (PursuitStatus::controller).
  std::string label() const;
  /// Switches to the (N, dt) asked by the GCS when its solver is available (a background build
  /// that finished is loaded here). Called every cycle, also while the pursuit is idle.
  void selectVariant();
  /// Events for the node log (a solver compiled / switched), cleared on reading.
  std::vector<std::string> takeMessages();
  /// (Re)engagement: forgets the previous plan.
  void reset();
  /// One control step (every node cycle, period dt).
  ControlCommand compute(const PursuitState & state, double dt);
  /// GCS number `mpc.params.<name>` set / removed.
  void setParameter(const std::string & name, double value);
  void eraseParameter(const std::string & name);

  /// acados status of the last solve (0 = success).
  int lastStatus() const {return status_;}
  /// Horizon of the active solver.
  int horizonSteps() const;
  double horizonStep() const;
  /// Active yaw mode (0..4).
  int yawMode() const;

private:
  struct Solver;  ///< One acados solver instance (capsule + handles) of an AcadosApi (mpc.cpp)
  using StateVec = std::array<double, 7>;
  using InputVec = std::array<double, 4>;

  /// Value of a GCS parameter, or `fallback` if the name was never set.
  double param(const std::string & name, double fallback) const;
  /// Writes the weights (W, W_e), the input bounds and the field-of-view constraint data.
  void applyWeightsAndBounds();
  /// Initial guess = previous plan shifted one step (or a constant-velocity guess); returns the
  /// guessed states x[0..N], whose positions also give the heading references.
  std::vector<StateVec> warmStart(const StateVec & x0);

  std::map<std::string, double> params_;
  bool params_dirty_ = true;  ///< GCS numbers changed: rewrite them before the next solve
  // Declared before solver_: the libraries must outlive the solver that uses their functions.
  AcadosVariants variants_;
  std::unique_ptr<Solver> solver_;
  std::vector<StateVec> prev_x_;  ///< last solution x[0..N] (empty: none / after a switch)
  std::vector<InputVec> prev_u_;  ///< last solution u[0..N-1]
  std::vector<std::string> messages_;
  int status_ = 0;
};

}  // namespace aero_pursuit::control
