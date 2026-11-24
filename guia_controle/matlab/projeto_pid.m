%PROJETO_PID PBVS PID design: pole placement, discrete root locus, margins, optimal PID.
%   Chapter 5 of the guide.  Requires the Control System Toolbox.
clear; clc; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), 'controladores'));
P = parametros();  M = modelo_px4(P, true);  Ts = P.Ts;
z = tf('z', Ts);  s = tf('s');

%% 1) Pole placement on the ideal model (perfect velocity tracking)
%    (1+kd) s^2 + kp s + ki = 0
tau_e = 0.8;  zeta = 1.0;  kd = 0.2;
wn = 1/tau_e;
kp = 2*zeta*wn*(1+kd);   ki = (1+kd)*wn^2;
fprintf('Alocacao: kp=%.3f ki=%.3f kd=%.3f\n', kp, ki, kd);
fprintf('Polos do yaml: '); disp(roots([1+P.pid.kd, P.pid.kp, P.pid.ki]).');

%% 2) Discrete root locus with PX4 + delay (gain kp; fixed zeros)
Pz = M.Pv_z;                                    % v_sp -> posicao, ZOH + z^-n
razao_d = P.pid.kd/P.pid.kp;   razao_i = P.pid.ki/P.pid.kp;
C0 = 1 + razao_i*Ts*z/(z-1) + razao_d*(z-1)/(Ts*z);   % Discrete PID with kp = 1
figure('Name','LGR PID'); rlocus(C0*Pz); zgrid(0.7, 2.3*Ts/pi);   % wn normalized by pi/Ts axis([-1.2 1.2 -1.2 1.2]);
title('LGR discreto do PID PBVS (PX4 + atraso)');
[Gm, Pm, Wcg, Wcp] = margin(P.pid.kp*C0*Pz);
fprintf('yaml: margem de ganho %.1f (kp critico = %.2f), PM %.1f graus em %.2f rad/s\n', ...
        Gm, Gm*P.pid.kp, Pm, Wcp);
% Escolha interativa:  [k, polos] = rlocfind(C0*Pz)

%% 3) Ziegler-Nichols from the critical gain (aggressive starting point)
Ku = Gm*P.pid.kp;  Pu = 2*pi/Wcg;
fprintf('ZN: Ku=%.2f Pu=%.2f s -> kp=%.2f Ti=%.2f Td=%.3f\n', Ku, Pu, 0.6*Ku, Pu/2, Pu/8);

%% 4) Optimal LQ PID (ideal model): state [int e; e], e' = -v (stationary target)
A = [0 1; 0 0];  B = [0; -1];
Kpi = lqr(A, B, diag([0.5 4]), 1);      % pesos: q_int, q_e, r
fprintf('PI otimo LQ: kp=%.3f ki=%.3f\n', -Kpi(2), -Kpi(1));

%% 5) Validation in simulation (line scenario) and comparison with the yaml
g = P.pid;
R1 = sim_perseguicao(@ctrl_pid, g, cenario('reta'), P, struct('grafico',true));
g.kp = kp; g.ki = ki; g.kd = kd;
R2 = sim_perseguicao(@ctrl_pid, g, cenario('reta'), P, struct('grafico',true));
fprintf('RMSE pos: yaml %.3f m | alocacao %.3f m\n', R1.metricas.rmse_pos, R2.metricas.rmse_pos);
