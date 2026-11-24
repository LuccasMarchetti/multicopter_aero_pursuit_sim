%RODAR_SIMULINK Creates (if needed) and simulates perseguicao_1eixo.slx for PID, LQR, LQI and SMC.
%   One axis along the LOS; the target starts at rest and accelerates to vt_alvo at t = 2 s.
clear; clc; close all;
aqui = fileparts(mfilename('fullpath'));
addpath(fullfile(aqui, '..'), fullfile(aqui, '..', 'controladores'));
P = parametros();

% ---- Variables read by the model (base workspace)
Ts = P.Ts;  tau = P.tau;  d = P.d;  vt_alvo = 1.5;  r0 = 8;
Kp_v = P.px4.Kp_v;  Ki_v = P.px4.Ki_v;  Kd_v = P.px4.Kd_v;  wa = P.px4.wa;
K_lqr = dare_ganho([1 Ts;0 1], [Ts^2/2;Ts], diag([P.lqr.qp P.lqr.qv]), P.lqr.r);
K_lqi = dare_ganho([1 Ts 0;0 1 0;Ts 0 1], [Ts^2/2;Ts;0], ...
                   diag([P.lqi.qp P.lqi.qv P.lqi.qi]), P.lqi.r);
s = P.smc;
g_ctrl = [P.pid.kp P.pid.ki P.pid.kd P.pid.ff, K_lqr, K_lqi, ...
          s.lambda s.ks s.k0 s.kmax s.gamma s.sigma s.phi, P.vmax P.amax];

mdl = 'perseguicao_1eixo';
if ~exist(fullfile(aqui, [mdl '.slx']), 'file'), criar_modelo_simulink(); end
load_system(fullfile(aqui, [mdl '.slx']));

nomes = {'PID PBVS', 'LQR', 'LQI', 'ASMC'};
figure('Name', 'Simulink: comparacao 1D');
for tipo = 1:4
    out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
    e = out.get('erro_dist');  v = out.get('v_chaser');
    subplot(2,1,1); hold on; plot(e.Time, e.Data, 'DisplayName', nomes{tipo});
    subplot(2,1,2); hold on; plot(v.Time, v.Data, 'DisplayName', nomes{tipo});
    m = e.Time > 10;
    fprintf('%-9s RMS erro (t>10 s) = %.3f m\n', nomes{tipo}, sqrt(mean(e.Data(m).^2)));
end
subplot(2,1,1); grid on; ylabel('|r| - d [m]'); legend show;
subplot(2,1,2); grid on; ylabel('v_c [m/s]'); xlabel('t [s]');

% Exercicios sugeridos:
%  1) tau = 0.2 and 0.3: note the loss of damping (compare with analise_margens.m)
%  2) Ts = 0.1: effect of sampling
%  3) Kp_v = 1.0 (slower PX4) or wa = 2
%  4) replace the target Step with a Sine Wave (oscillating target)
