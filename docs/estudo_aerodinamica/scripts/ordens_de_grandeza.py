#!/usr/bin/env python3
"""Orders of magnitude of the x500 aerodynamic forces (feasibility study).

Compares the current Gazebo model (linear rotor drag only, MulticopterMotorModel)
with a model that adds quadratic parasitic drag of the airframe (flat plate / table).

Usage: python3 ordens_de_grandeza.py            (no dependencies beyond the stdlib)
"""
import math

RHO = 1.225          # kg/m³ (nível do mar, ISA)
G = 9.8              # m/s² (igual ao lawn.sdf)
M = 2.064            # kg, massa total do x500 (docs/INVESTIGACOES.md §1)
KF = 8.54858e-6      # N/(rad/s)², motorConstant
CD_ROTOR = 8.06428e-5  # rotorDragCoefficient (N·s/m por rad/s)
CR_ROTOR = 1e-6      # rollingMomentCoefficient
OMEGA_MAX = 1000.0   # rad/s
N_ROT = 4

# Área de arrasto equivalente do airframe (CdA, m²).
# NASA TM-2018-219758 (Russell et al.), Fig. 30: airframe sem rotores do Iris+/SOLO/Phantom,
# Drag/q ≈ 0,18–0,45 ft² → 0,017–0,042 m²; mínimo perto de α = 0°, ~1,5–2× em |α| = 40°.
CDA_LOW, CDA_MID, CDA_HIGH = 0.017, 0.030, 0.042


def omega_hover():
    return math.sqrt(M * G / (N_ROT * KF))


def rotor_drag_coeff(omega):
    """Equivalent linear D [N·s/m] = sum of the rotors' |ω| c_d."""
    return N_ROT * omega * CD_ROTOR


def cda_of_alpha(alpha_deg, cda0):
    """Placa plana composta: frente (cda0) + topo (~3× a frente) projetados."""
    a = math.radians(alpha_deg)
    return cda0 * abs(math.cos(a)) + 3.0 * cda0 * abs(math.sin(a)) * 0.5


def terminal_speed(tilt_deg, cda):
    """Steady speed with altitude held: T sinθ = D_rotor v + ½ρ CdA v²."""
    th = math.radians(tilt_deg)
    thrust = M * G / math.cos(th)
    if thrust > N_ROT * KF * OMEGA_MAX ** 2:
        return float("nan")
    w = math.sqrt(thrust / (N_ROT * KF))
    d_lin = rotor_drag_coeff(w)
    fx = thrust * math.sin(th)
    a = 0.5 * RHO * cda
    if a == 0.0:
        return fx / d_lin
    return (-d_lin + math.sqrt(d_lin ** 2 + 4 * a * fx)) / (2 * a)


def main():
    wh = omega_hover()
    d = rotor_drag_coeff(wh)
    print(f"ω_hover = {wh:.0f} rad/s  ({wh / OMEGA_MAX * 100:.0f} % de ω_max)")
    print(f"D_rotor (linear, pairar) = {d:.3f} N·s/m  →  τ = m/D = {M / d:.1f} s;"
          f"  EKF2_MCOEF equivalente = D/m = {d / M:.3f} 1/s")
    print()
    print(" v [m/s] | F_rotor [N] | F_corpo CdA=0,017 | 0,030 | 0,042 [N] | razão corpo/rotor (0,030)")
    for v in (2, 3, 5, 8, 10, 12, 15, 20):
        fr = d * v
        fb = [0.5 * RHO * c * v * v for c in (CDA_LOW, CDA_MID, CDA_HIGH)]
        print(f" {v:6.0f}  | {fr:10.2f}  | {fb[0]:16.2f}  | {fb[1]:5.2f} | {fb[2]:9.2f} | {fb[1] / fr:5.2f}")
    v_eq = d / (0.5 * RHO * CDA_MID)
    print(f"\nVelocidade em que o arrasto do corpo iguala o de rotor (CdA=0,030): {v_eq:.1f} m/s")
    print()
    print(" inclinação | v_terminal só rotor (Gazebo atual) | com corpo CdA(α) mín/méd/máx")
    for tilt in (10, 20, 30, 35, 45):
        v0 = terminal_speed(tilt, 0.0)
        vs = [terminal_speed(tilt, cda_of_alpha(tilt, c)) for c in (CDA_LOW, CDA_MID, CDA_HIGH)]
        print(f"   {tilt:4.0f}°    | {v0:10.1f} m/s"
              f"                      | {vs[0]:5.1f} / {vs[1]:5.1f} / {vs[2]:5.1f} m/s")
    # Momento de rolagem do rotor vs momento da força de arrasto do rotor (braço z = 0,06 m)
    v = 10.0
    m_roll = N_ROT * wh * CR_ROTOR * v
    m_arm = d * v * 0.06
    print(f"\nA 10 m/s: momento de rolagem dos rotores = {m_roll * 1e3:.1f} mN·m;"
          f" momento do arrasto aplicado a z=+0,06 m = {m_arm * 1e3:.0f} mN·m")
    # Custo de uma tabela (α, β, V) com 6 coeficientes float32
    na, nb, nv = 73, 37, 12
    print(f"\nTabela α×β×V = {na}×{nb}×{nv}, 6 coeficientes float32: "
          f"{na * nb * nv * 6 * 4 / 1024:.0f} KiB por veículo")


if __name__ == "__main__":
    main()
