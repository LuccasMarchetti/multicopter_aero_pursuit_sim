%PROJETO_LQI LQR with integral action: gains, choice of q_int, disturbance rejection.
%   Chapter 7 of the guide.
clear; clc; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), 'controladores'));
P = parametros();  M = modelo_px4(P, true);  Ts = P.Ts;  s = tf('s');
Aa = [1 Ts 0; 0 1 0; Ts 0 1];  Ba = [Ts^2/2; Ts; 0];

%% 1) yaml gain
[K, ~, pz] = dlqr(Aa, Ba, diag([P.lqi.qp P.lqi.qv P.lqi.qi]), P.lqi.r);
fprintf('K = [%.3f %.3f %.3f]\n', K);  disp('polos (s):'); disp(log(pz).'/Ts);

%% 2) Sweep of q_int: integrator pole x PM with PX4 + delay
fprintf('\n  q_int   polo_int   par dominante      PM\n');
for qi = [0.2 0.5 1 2 5 10 20]
    [K, ~, pz] = dlqr(Aa, Ba, diag([P.lqi.qp P.lqi.qv qi]), P.lqi.r);
    ps = log(pz)/Ts;
    preal = ps(abs(imag(ps)) < 1e-6);  pcpx = ps(imag(ps) > 1e-6);
    if isempty(preal), preal = NaN; end
    if isempty(pcpx), pcpx = NaN; end
    % u = -(kp ep + kd ev + ki xi), ev = u/s (referencia), xi = ep/s
    C = (K(1) + K(3)/s)*s/(s + K(2));
    L = C*M.Tv/s^2;  L.InputDelay = P.tau;  [~, pm] = margin(L);
    fprintf('%6.1f %10.3f   %6.2f%+6.2fj %8.1f\n', qi, min(abs(preal))*-1, ...
            real(pcpx(1)), imag(pcpx(1)), pm);
end

%% 3) Constant disturbance rejection (wind 0.5 m/s^2): LQR x LQI
C = cenario('regulacao');  C.vento = [0.5; 0; 0];  C.T = 25;
Rl = sim_perseguicao(@ctrl_lqr, P.lqr, C, P);
Ri = sim_perseguicao(@ctrl_lqi, P.lqi, C, P);
figure('Name','LQR x LQI com vento');
plot(Rl.t, Rl.e_range, Ri.t, Ri.e_range); grid on; legend('LQR','LQI');
xlabel('t [s]'); ylabel('|r|-d [m]'); title('Perturbacao constante de 0,5 m/s^2');
Kl = dlqr([1 Ts;0 1], [Ts^2/2;Ts], diag([P.lqr.qp P.lqr.qv]), P.lqr.r);
fprintf('Erro de regime previsto do LQR: a/kp = %.3f m\n', 0.5/Kl(1));
