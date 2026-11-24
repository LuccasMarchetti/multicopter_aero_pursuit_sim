%ANALISE_MARGENS Reproduces the margins table and the margin x delay figure of the guide.
%   Outer loop per axis, stationary target, PX4 with feedforward.
%   Acceleration controllers: damping acts on v_ref = u/s, so
%       C_eq(s) = kp*s/(s+kd)     e     L = C_eq * Tv/s^2 * e^{-s tau}
%   PBVS PID (velocity output):  L = (kp + ki/s + kd*s) * Tv/s * e^{-s tau}
clear; clc;
addpath(fullfile(fileparts(mfilename('fullpath')), 'controladores'));
P = parametros();
M = modelo_px4(P, true);
s = tf('s');
Tv = M.Tv;

% Equivalent gains: LQR (DARE), MPC (first row of H^-1 F), SMC (boundary layer)
K_lqr = dare_ganho([1 P.Ts; 0 1], [P.Ts^2/2; P.Ts], diag([P.lqr.qp P.lqr.qv]), P.lqr.r);
K_mpc = mpc_ganho_equivalente(P);
keq   = P.smc.ks + P.smc.kmax/P.smc.phi;

casos = { ...
  'LQR',            K_lqr(1)*s/(s+K_lqr(2)) * Tv/s^2;
  'MPC cinematico', K_mpc(1)*s/(s+K_mpc(2)) * Tv/s^2;
  'SMC (k=kmax)',   P.smc.lambda*keq*s/(s+P.smc.lambda+keq) * Tv/s^2;
  'PID PBVS',       (P.pid.kp + P.pid.ki/s + P.pid.kd*s) * Tv/s };

fprintf('%-16s %8s %8s %10s %12s\n','controlador','wc','PM(0)','PM(tau)','marg.atraso');
for i = 1:size(casos,1)
    L0 = casos{i,2};
    L1 = L0;  L1.InputDelay = P.tau;
    m0 = allmargin(L0);  m1 = allmargin(L1);
    [~,Pm0,~,wc] = margin(L0);  [~,Pm1] = margin(L1);
    fprintf('%-16s %8.2f %8.1f %10.1f %10.0f ms\n', casos{i,1}, wc, Pm0, Pm1, ...
            1000*min(m0.DelayMargin));
end

% Phase margin as a function of delay
atrasos = 0:0.01:0.6;
figure('Name','Margem x atraso'); hold on; grid on;
for i = 1:size(casos,1)
    pm = zeros(size(atrasos));
    for j = 1:numel(atrasos)
        L = casos{i,2};  L.InputDelay = atrasos(j);
        [~, pm(j)] = margin(L);
    end
    plot(1000*atrasos, pm, 'DisplayName', casos{i,1});
end
yline(45,'--k','45 graus');
xlabel('atraso total [ms]'); ylabel('margem de fase [graus]'); legend show;

% Step response of the inner loop (figure px4_degrau)
figure('Name','PX4 degrau'); step(modelo_px4(P,false).Tv, Tv, 6); grid on;
legend('sem feedforward','com feedforward');
stepinfo(Tv)
