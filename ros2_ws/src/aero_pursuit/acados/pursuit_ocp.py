"""Optimal control problem (OCP) of the pursuit MPC, in acados.

Relative translational model + pursuer heading, ENU, discrete:

    x = [p_rel (3), v_rel (3), psi]   p_rel = p_target - p_chaser, v_rel idem; psi = pursuer heading
    u = [a_chaser (3), r]             pursuer acceleration [m/s^2] and yaw rate [rad/s]
                                      (the target: constant velocity)

    x[k+1] = A x[k] + B u[k]
        A = [[I, dt I, 0],            B = [[-dt^2/2 I, 0],
             [0,    I, 0],                 [-dt I,     0],
             [0,    0, 1]]                 [0,        dt]]

    Translation and heading are decoupled in the dynamics (the accelerations are in the world
    frame; the PX4 tilts the body for any heading). The heading matters for the camera only.

Cost (NONLINEAR_LS: 1/2 * || y(x, u) - yref ||^2_W at every stage):

    y(x, u) = [ p_rel (3); v_rel (3);                    linear entries
                psi;                                     heading, against psi_ref_k (modes 1, 2)
                cos(psi) - px/rho;  sin(psi) - py/rho;   camera direction minus the unit vector to
                                                         the target (modes 3, 4), reference 0
                a (3); r ]                               (13 entries; terminal: without a, r: 9)
    rho = sqrt(px^2 + py^2 + EPS)

    The two "bearing" entries cost q (2 - 2 cos(delta)), delta = atan2(py, px) - psi: the
    relative heading computed by the solver from the positions it is optimizing (no atan2, no
    wrap). Gauss-Newton linearizes y around the guess every cycle; its Jacobian
    d(p/rho)/dp = (I - n n^T)/rho lets the solver trade moving sideways for turning.

    W = diag(q_p, q_v, w_psi, w_b, w_b, r_a, r_r), chosen at run time by yaw_mode:
      0: w_psi = w_b = 0 (heading held by the node)
      1, 2 (LOS heading tracking): w_psi = q_psi, psi_ref_k = atan2 of the previous plan (DATA), w_b = 0
      3, 4 (perception-aware, PAMPC): w_b = q_psi, w_psi = 0
    W_e = q_f * the state part of W.

Constraints:
    x[0] = measured state                         (lbx_0 = ubx_0, every cycle)
    |a_i| <= a_max per axis, |r| <= r_max          (box on u, run time)
    field of view (yaw_mode 2 and 4), stages 1..N, soft (L1 + L2 slack penalty w_fov):
        front = cos(psi) px + sin(psi) py          (target ahead of the camera)
        lat   = -sin(psi) px + cos(psi) py         (target to the left of the optical axis)
        lat - c front <= 0   and   -lat - c front <= 0,   c = tan(usable half field of view)
      i.e. |lat| <= c front: the target inside the horizontal field of view. Nonlinear (psi
      times p): SQP-RTI linearizes it around the previous plan every cycle. c is a stage
      parameter (np = 1); with yaw_mode 0, 1, 3 the bound is opened (uh = +INFTY).

N and DT are compile-time sizes of the solver. Each (N, dt) pair is a separate solver named
variant_name(N, dt) (its C functions carry that prefix, so several can live in one process):
the default pair below is compiled with the package; others are generated and compiled at run
time by the node when the GCS asks for them (mpc.params.N / mpc.params.dt, control/mpc.cpp).
"""
import math

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp

N = 20                 # default horizon steps
DT = 0.1               # default step [s] (horizon = N * DT = 2 s)
NX, NU = 7, 4
NY, NY_E = NX + 2 + NU, NX + 2   # x, 2 bearing entries, u
EPS = 1e-2             # [m^2] keeps rho > 0 (target straight above/below)
NP = 1                 # stage parameter: c = tan(usable half field of view)
NH = 2                 # field-of-view inequalities

# Initial weights / limits (the GCS overrides them through mpc.params.*, control/mpc.cpp).
Q_P = (10.0, 10.0, 10.0)
Q_V = (1.0, 1.0, 1.0)
R_A = (0.5, 0.5, 0.5)
Q_F = 5.0
A_MAX = (3.0, 3.0, 2.0)
Q_PSI = 5.0            # heading error weight (yaw_mode 1, 2)
R_R = 0.5              # yaw rate weight
R_MAX = 1.2            # |r| bound [rad/s] (PX4 MC_YAWRATE_MAX is 200 deg/s)
FOV_DEG = 69.0         # horizontal field of view of the RGB camera [deg]
FOV_MARGIN_DEG = 8.0   # kept free on each side [deg]
W_FOV = 200.0          # slack penalty of the field-of-view constraint (L1 and L2)
INFTY = 1e10           # acados "no bound" (anything smaller stays in the QP)


def variant_name(n: int, dt: float) -> str:
    """Prefix of the generated C functions, e.g. pursuit_mpc_N20_dt100 (dt in milliseconds)."""
    return f"pursuit_mpc_N{int(n)}_dt{int(round(dt * 1000))}"


def fov_expr(x, c):
    """[lat - c front, -lat - c front]: both <= 0 <=> target inside the field of view."""
    px, py, psi = x[0], x[1], x[6]
    front = ca.cos(psi) * px + ca.sin(psi) * py
    lat = -ca.sin(psi) * px + ca.cos(psi) * py
    return ca.vertcat(lat - c * front, -lat - c * front)


def cost_y(x, u=None):
    """Residuals of the NONLINEAR_LS cost (see the module docstring)."""
    px, py, psi = x[0], x[1], x[6]
    rho = ca.sqrt(px**2 + py**2 + EPS)
    y = ca.vertcat(x, ca.cos(psi) - px / rho, ca.sin(psi) - py / rho)
    return y if u is None else ca.vertcat(y, u)


def model(dt: float, name: str) -> AcadosModel:
    x = ca.SX.sym("x", NX)
    u = ca.SX.sym("u", NU)
    p = ca.SX.sym("p", NP)
    I3, Z3 = np.eye(3), np.zeros((3, 3))
    z31, z13 = np.zeros((3, 1)), np.zeros((1, 3))
    A = np.block([[I3, dt * I3, z31],
                  [Z3, I3, z31],
                  [z13, z13, np.ones((1, 1))]])
    B = np.block([[-0.5 * dt**2 * I3, z31],      # minus: the pursuer acceleration closes the gap
                  [-dt * I3, z31],
                  [z13, dt * np.ones((1, 1))]])
    m = AcadosModel()
    m.name = name
    m.x = x
    m.u = u
    m.p = p
    m.disc_dyn_expr = A @ x + B @ u
    # Field of view at stages 1..N-1 and N (no con_h_expr_0: x[0] is the measurement).
    m.con_h_expr = fov_expr(x, p[0])
    m.con_h_expr_e = fov_expr(x, p[0])
    m.cost_y_expr = cost_y(x, u)
    m.cost_y_expr_e = cost_y(x)
    return m


def ocp(n: int = N, dt: float = DT) -> AcadosOcp:
    name = variant_name(n, dt)
    o = AcadosOcp()
    o.name = name          # else acados names it ocp_<model>_<hash>
    o.model = model(dt, name)
    o.parameter_values = np.array([math.tan(math.radians(FOV_DEG / 2 - FOV_MARGIN_DEG))])

    # Horizon
    o.solver_options.N_horizon = n
    o.solver_options.tf = n * dt

    # Cost: y = cost_y(x, u) (model.cost_y_expr); weights of the heading entries set at run time
    o.cost.cost_type = "NONLINEAR_LS"
    o.cost.cost_type_e = "NONLINEAR_LS"
    o.cost.W = np.diag(Q_P + Q_V + (Q_PSI, 0.0, 0.0) + R_A + (R_R,))
    o.cost.yref = np.zeros(NY)
    o.cost.W_e = Q_F * np.diag(Q_P + Q_V + (Q_PSI, 0.0, 0.0))
    o.cost.yref_e = np.zeros(NY_E)

    # Constraints
    o.constraints.x0 = np.zeros(NX)            # creates lbx_0 = ubx_0 on every state
    o.constraints.idxbu = np.arange(NU)
    o.constraints.lbu = -np.array(A_MAX + (R_MAX,))
    o.constraints.ubu = np.array(A_MAX + (R_MAX,))
    # Field of view: h <= 0, soft; opened (uh = INFTY) unless yaw_mode 2 (set at run time).
    o.constraints.lh = -INFTY * np.ones(NH)
    o.constraints.uh = INFTY * np.ones(NH)
    o.constraints.idxsh = np.arange(NH)
    o.constraints.lh_e = -INFTY * np.ones(NH)
    o.constraints.uh_e = INFTY * np.ones(NH)
    o.constraints.idxsh_e = np.arange(NH)
    # Slack penalties: z (L1, exact penalty) and Z (L2, smooth) for the lower and upper sides.
    for suffix in ("", "_e"):
        for field in ("zl", "zu", "Zl", "Zu"):
            setattr(o.cost, field + suffix, W_FOV * np.ones(NH))

    # Solver
    o.solver_options.integrator_type = "DISCRETE"     # disc_dyn_expr is the model itself
    o.solver_options.nlp_solver_type = "SQP_RTI"      # one QP per call (exact in modes 0, 1)
    o.solver_options.hessian_approx = "GAUSS_NEWTON"  # J^T W J: exact for the linear entries
    o.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    o.solver_options.print_level = 0
    return o
