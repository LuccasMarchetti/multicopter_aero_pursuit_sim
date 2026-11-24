#pragma once

#include <string>

namespace aero_pursuit::control
{

/// The functions of ONE generated acados solver (one (N, dt) variant of acados/pursuit_ocp.py).
///
/// acados generates C functions whose names carry the solver name, e.g.
/// pursuit_mpc_N20_dt100_acados_solve(). This table holds pointers to them, so the MPC calls any
/// variant the same way, whether it was compiled with the package (defaultAcadosApi()) or
/// generated and loaded at run time (AcadosVariants, dlopen/dlsym). The solver capsule and the
/// acados handles are passed as void * (opaque here; mpc.cpp casts them to the acados types).
struct AcadosApi
{
  std::string name;  ///< C prefix, e.g. "pursuit_mpc_N20_dt100"
  int N = 0;         ///< horizon steps
  double dt = 0.0;   ///< step [s]
  void * (*create_capsule)() = nullptr;
  int (*create)(void * capsule) = nullptr;
  int (*solve)(void * capsule) = nullptr;
  int (*free_solver)(void * capsule) = nullptr;
  int (*free_capsule)(void * capsule) = nullptr;
  void * (*get_config)(void * capsule) = nullptr;
  void * (*get_dims)(void * capsule) = nullptr;
  void * (*get_in)(void * capsule) = nullptr;
  void * (*get_out)(void * capsule) = nullptr;
  /// Stage parameters p (here c = tan(usable half field of view)), stage 0..N.
  int (*update_params)(void * capsule, int stage, double * p, int np) = nullptr;
};

/// The variant compiled with the package (default N/DT of pursuit_ocp.py; generated
/// acados_default_api.cpp in libaero_pursuit_acados).
const AcadosApi & defaultAcadosApi();

}  // namespace aero_pursuit::control
