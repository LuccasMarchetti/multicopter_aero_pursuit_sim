%PROJETO_GUIAMENTO PN/PP guidance: radial loop (PD), navigation constant, alpha.
%   Chapter 12 of the guide.
clear; clc; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), 'controladores'));
P = parametros();  M = modelo_px4(P, true);  s = tf('s');  g = P.pn;

%% 1) Malha radial: s^2 + krd s + kr
wn = sqrt(g.kr);  zeta = g.krd/(2*wn);
fprintf('Radial: wn=%.2f rad/s zeta=%.2f ts=%.1f s\n', wn, zeta, 4/(zeta*wn));
% With PX4 + delay: rdot comes from the KF (measured), so the delay hits P and D
L = (g.kr + g.krd*s)*M.Tv/s^2;  L.InputDelay = P.tau;
figure('Name','Radial'); margin(L); grid on;

%% 2) Radial loop design by specification
ts = 3;  z = 0.8;  w = 4/(z*ts);
fprintf('Para ts=%.1f s e zeta=%.1f: kr=%.2f krd=%.2f\n', ts, z, w^2, 2*z*w);
rlocus((1 + (2*z/w)*s)*M.Tv/s^2); sgrid(z, w); title('LGR radial (kr)');

%% 3) Sweep of N and alpha in the zig-zag scenario
fprintf('\n   N  alpha   RMSE pos   esforco\n');
for N = [2 3 4 5]
    for a = [0 0.5 1]
        gg = g;  gg.N = N;  gg.alpha = a;
        R = sim_perseguicao(@ctrl_guiamento, gg, cenario('zigue'), P);
        fprintf('%4.0f %6.1f %10.3f %9.1f\n', N, a, R.metricas.rmse_pos, R.metricas.effort_l2);
    end
end
