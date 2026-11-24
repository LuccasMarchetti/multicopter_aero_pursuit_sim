%PROJETO_SMC Modos deslizantes: superficie otima, PD equivalente, margens, chattering.
%   Chapter 8 of the guide.
clear; clc; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), 'controladores'));
P = parametros();  M = modelo_px4(P, true);  s = tf('s');  g = P.smc;

%% 1) Superficie otima (Utkin): lambda* = sqrt(qp/qv)
lam_otimo = sqrt(P.lqr.qp/P.lqr.qv);
fprintf('lambda yaml = %.2f (tau = %.2f s) | lambda* = %.2f\n', g.lambda, 1/g.lambda, lam_otimo);

%% 2) Equivalent PD inside the boundary layer and margins with PX4 + delay
fprintf('\n   k_hat   k_eq   kp_eq   kd_eq   PM(tau)\n');
for k = [g.k0 1.5 g.kmax]
    keq = g.ks + k/g.phi;
    kp = g.lambda*keq;  kd = g.lambda + keq;
    L = kp*s/(s + kd)*M.Tv/s^2;  L.InputDelay = P.tau;  [~, pm] = margin(L);
    fprintf('%7.2f %6.2f %7.2f %7.2f %8.1f\n', k, keq, kp, kd, pm);
end

%% 3) Minimum boundary layer against chattering: phi >= k_max (Ts + tau)
fprintf('\nphi minimo sugerido = %.2f m/s (yaml: %.2f)\n', g.kmax*(P.Ts + P.tau), g.phi);

%% 4) Simulation: evolution of the adaptive gain in the stop-and-go scenario
C = cenario('para_anda');
R = sim_perseguicao(@ctrl_smc, g, C, P, struct('grafico',true));
g2 = g;  g2.lambda = lam_otimo;
R2 = sim_perseguicao(@ctrl_smc, g2, C, P);
fprintf('RMSE pos: lambda yaml %.3f | lambda* %.3f\n', R.metricas.rmse_pos, R2.metricas.rmse_pos);

%% 5) Comparison with small phi (chattering) using the acceleration spectrum
g3 = g;  g3.phi = 0.05;
R3 = sim_perseguicao(@ctrl_smc, g3, C, P);
figure('Name','Chattering');
plot(R.t, vecnorm(R.a_cmd), R3.t, vecnorm(R3.a_cmd)); grid on;
legend(sprintf('\\phi = %.2f', g.phi), '\phi = 0,05'); xlabel('t [s]'); ylabel('|a| [m/s^2]');
