#!/usr/bin/env python3
"""Generates the guide figures (numpy/scipy/matplotlib). Usage: python3 gerar_figuras.py

The same computations exist in MATLAB (folder ../matlab); this script only ensures
that the PDF builds without MATLAB.
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import scipy.linalg as la  # noqa: E402
import scipy.signal as sg  # noqa: E402

AQUI = os.path.dirname(os.path.abspath(__file__))
plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": 0.3,
                     "figure.dpi": 150, "savefig.bbox": "tight"})

T = 0.05                      # controller period [s]
WA = 4.0                      # MC_PITCH_P: attitude loop bandwidth [rad/s]
KP_V, KI_V, KD_V = 1.8, 0.4, 0.2   # MPC_XY_VEL_{P,I,D}_ACC


def px4_velocidade(ff):
    """v/v_sp of the PX4 velocity loop (ff=1: with acceleration feedforward)."""
    num = [WA * ff, WA * KP_V, WA * KI_V]
    den = np.polyadd(np.polymul([1, 0, 0], [1, WA]), [WA * KD_V, WA * KP_V, WA * KI_V])
    return num, den


def salvar(nome):
    plt.savefig(os.path.join(AQUI, nome))
    plt.close()


# 1) Step response of the PX4 velocity loop -----------------------------
t = np.linspace(0, 6, 3001)
plt.figure(figsize=(6, 3.2))
for ff, rot in ((0, "sem feedforward de aceleração"), (1, "com feedforward ($a_{ff}=\\dot v_{sp}$)")):
    _, y = sg.step(sg.TransferFunction(*px4_velocidade(ff)), T=t)
    plt.plot(t, y, label=rot)
plt.axhline(1, color="k", lw=0.8, ls="--")
plt.xlabel("tempo [s]")
plt.ylabel("$v / v_{sp}$")
plt.title("Malha de velocidade do PX4 (modelo reduzido)")
plt.legend()
salvar("px4_degrau.png")

# 2) Discrete root locus of the PID (gain kp) with PX4 + delay ------------------
num, den = px4_velocidade(1)
# Plant v_sp -> position: T_ff(s)/s; ZOH with T; 2-sample delay (0.1 s)
numd, dend, _ = sg.cont2discrete((num, np.polymul(den, [1, 0])), T, method="zoh")
numd = np.trim_zeros(np.atleast_1d(np.squeeze(numd)), "f")
dend = np.polymul(dend, [1, 0, 0])            # z^-2
# Controlador PD discreto: C(z) = kp + kd (z-1)/(T z) = kp [1 + (kd/kp)(z-1)/(T z)]
razao = 0.25                                   # fixed kd/kp (yaml PID: 0.2/0.8)
cnum = np.polyadd([1, 0], razao / T * np.array([1, -1]))
cden = [1, 0]
Ln, Ld = np.polymul(cnum, numd), np.polymul(cden, dend)
plt.figure(figsize=(5, 5))
th = np.linspace(0, 2 * np.pi, 400)
plt.plot(np.cos(th), np.sin(th), "k", lw=0.8)
ks = np.concatenate([np.linspace(0, 3, 600), np.linspace(3, 12, 300)])
traj = np.array([np.roots(np.polyadd(Ld, k * np.pad(Ln, (len(Ld) - len(Ln), 0)))) for k in ks])
for i in range(traj.shape[1]):
    plt.plot(traj[:, i].real, traj[:, i].imag, ".", ms=1.2, color="C0")
k_inst = None
for k, r in zip(ks, traj):
    if np.max(np.abs(r)) >= 1 and k > 0:
        k_inst = k
        break
for k, cor in ((0.8, "C2"), (k_inst, "C3")):
    if k is None:
        continue
    r = np.roots(np.polyadd(Ld, k * np.pad(Ln, (len(Ld) - len(Ln), 0))))
    plt.plot(r.real, r.imag, "o", color=cor, mfc="none", label=f"$k_p={k:.2f}$")
plt.plot(np.roots(Ld).real, np.roots(Ld).imag, "kx", label="polos de malha aberta")
plt.plot(np.roots(Ln).real, np.roots(Ln).imag, "ko", mfc="none", ms=4, label="zeros")
plt.axis("equal")
plt.xlim(-1.3, 1.3)
plt.ylim(-1.3, 1.3)
plt.xlabel("Re(z)")
plt.ylabel("Im(z)")
plt.title(f"LGR discreto do PD ($k_d/k_p={razao}$, $T={T}$ s, atraso 0,1 s)")
plt.legend(fontsize=8, loc="lower left")
salvar("lgr_pid_discreto.png")
print("kp crítico (PD, atraso 0,1 s):", k_inst)

# 3) Symmetric root locus of the LQR (varying rho = R) ---------------------------
A = np.array([[1, T], [0, 1]])
B = np.array([[T * T / 2], [T]])
plt.figure(figsize=(5.5, 4))
rhos = np.logspace(-3, 2, 200)
polos = []
for rho in rhos:
    P = la.solve_discrete_are(A, B, np.diag([10, 2]), np.array([[rho]]))
    K = la.solve(rho + B.T @ P @ B, B.T @ P @ A)
    polos.append(np.log(np.linalg.eigvals(A - B @ K).astype(complex)) / T)
polos = np.array(polos)
sc = None
for i in range(2):
    sc = plt.scatter(polos[:, i].real, polos[:, i].imag, c=np.log10(rhos), s=4, cmap="viridis")
P = la.solve_discrete_are(A, B, np.diag([10, 2]), np.array([[0.5]]))
K = la.solve(0.5 + B.T @ P @ B, B.T @ P @ A)
p = np.log(np.linalg.eigvals(A - B @ K).astype(complex)) / T
plt.plot(p.real, p.imag, "r*", ms=10, label="yaml: $Q=\\mathrm{diag}(10,2)$, $R=0{,}5$")
plt.colorbar(sc, label="$\\log_{10} R$")
plt.xlabel("Re(s) [1/s]")
plt.ylabel("Im(s) [rad/s]")
plt.title("Polos de malha fechada do LQR em função de $R$")
plt.legend(fontsize=8)
salvar("lgr_lqr_simetrico.png")


# 4) Phase margin x delay (LQR, MPC, PID) -----------------------------------------
def tff(s):
    return WA * (s * s + KP_V * s + KI_V) / (s * s * (s + WA) + WA * (KD_V * s * s + KP_V * s + KI_V))


def margem(lf):
    w = np.logspace(-2, 2, 20000)
    L = lf(1j * w)
    ic = np.argmax(np.abs(L) < 1)
    return 180 + np.degrees(np.unwrap(np.angle(L))[ic])


atrasos = np.linspace(0, 0.6, 61)
casos = {"LQR ($k_p$=4,09; $k_d$=3,39)": (4.088, 3.394, "acel"),
         "MPC cinemático eq. ($k_p$=2,16; $k_d$=2,06)": (2.155, 2.063, "acel"),
         "PID PBVS ($k_p$=0,8; $k_i$=0,05; $k_d$=0,2)": (0.8, 0.2, "vel")}
plt.figure(figsize=(6, 3.4))
for rot, (kp, kd, tipo) in casos.items():
    pm = []
    for tau in atrasos:
        if tipo == "acel":
            lf = lambda s, kp=kp, kd=kd, tau=tau: kp * s / (s + kd) * tff(s) / s**2 * np.exp(-s * tau)
        else:
            lf = lambda s, kp=kp, kd=kd, tau=tau: (kp + 0.05 / s + kd * s) * tff(s) / s * np.exp(-s * tau)
        pm.append(margem(lf))
    plt.plot(atrasos * 1000, pm, label=rot)
plt.axhline(45, color="k", ls="--", lw=0.8)
plt.text(5, 47, "45° (recomendado)", fontsize=8)
plt.xlabel("atraso total da malha [ms]")
plt.ylabel("margem de fase [graus]")
plt.title("Robustez ao atraso (modelo linear com PX4)")
plt.legend(fontsize=7)
salvar("margem_vs_atraso.png")


# 5) Before x after the KF/MPC fix (closed-loop simulation) -----------
def ler(nome):
    dados = []
    with open(os.path.join(AQUI, "dados", nome)) as f:
        for linha in f:
            c = linha.split(",")
            dados.append([float(x) for x in c[:6]])
    return np.array(dados)


antes, depois = ler("antes.csv"), ler("depois.csv")
fig, ax = plt.subplots(2, 1, figsize=(6.5, 4.6), sharex=True)
ax[0].plot(antes[:, 0], antes[:, 2], lw=0.9, label="antes")
ax[0].plot(depois[:, 0], depois[:, 2], lw=0.9, label="depois")
ax[0].axhline(1.5, color="k", ls="--", lw=0.7)
ax[0].set_ylabel("$v_x$ perseguidor [m/s]")
ax[0].legend(fontsize=8, loc="lower left")
ax[1].plot(antes[:, 0], antes[:, 1], lw=0.9)
ax[1].plot(depois[:, 0], depois[:, 1], lw=0.9)
ax[1].set_ylabel("$|r| - d$ [m]")
ax[1].set_xlabel("tempo [s]")
for a in ax:
    a.axvspan(15, 25, color="gray", alpha=0.12)
    a.axvline(30, color="gray", ls=":")
ax[0].set_title("Alvo: reta 1,5 m/s, curva (cinza), parada em 30 s")
salvar("antes_depois.png")
print("figuras geradas em", AQUI)
