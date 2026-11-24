%PROJETO_LQR LQR design: analytic formulas, Bryson, symmetric root locus, PX4, predictor.
%   Chapter 6 of the guide.  Requires the Control System Toolbox (dlqr, margin).
clear; clc; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), 'controladores'));
P = parametros();  M = modelo_px4(P, true);  Ts = P.Ts;  s = tf('s');
Ac = [0 1; 0 0];  Bc = [0; 1];
Ad = [1 Ts; 0 1];  Bd = [Ts^2/2; Ts];
qp = P.lqr.qp;  qv = P.lqr.qv;  r = P.lqr.r;

%% 1) Analytic solution (continuous) x DARE (discrete, same as the C++)
kp = sqrt(qp/r);  kd = sqrt(qv/r + 2*sqrt(qp/r));
fprintf('Continuo analitico: kp=%.3f kd=%.3f  wn=%.2f zeta=%.2f\n', kp, kd, ...
        (qp/r)^0.25, kd/(2*(qp/r)^0.25));
Kc = lqr(Ac, Bc, diag([qp qv]), r);
[Kd, ~, polos_z] = dlqr(Ad, Bd, diag([qp qv]), r);
fprintf('lqr: [%.3f %.3f] | dlqr: [%.3f %.3f] | dare_ganho: [%.3f %.3f]\n', Kc, Kd, ...
        dare_ganho(Ad, Bd, diag([qp qv]), r));
disp('Polos discretos (equivalente continuo):'); disp(log(polos_z).'/Ts);

%% 2) Projeto inverso: especificacao -> pesos (r = 1)
ts = 2.0;  zeta = 0.8;  wn = 4/(zeta*ts);
qp_i = wn^4;  qv_i = 2*wn^2*(2*zeta^2 - 1);
fprintf('Para ts=%.1f s, zeta=%.2f: qp=%.2f qv=%.2f r=1\n', ts, zeta, qp_i, qv_i);

%% 3) Bryson's rule
Qb = diag([1/0.5^2, 1/1^2]);  Rb = 1/P.amax^2;
fprintf('Bryson: K = [%.2f %.2f]\n', dlqr(Ad, Bd, Qb, Rb));

%% 4) Symmetric root locus: poles as a function of rho (R = rho)
rhos = logspace(-3, 2, 150);  pol = zeros(2, numel(rhos));
for i = 1:numel(rhos)
    [~, ~, pz] = dlqr(Ad, Bd, diag([qp qv]), rhos(i));
    pol(:, i) = log(pz)/Ts;
end
cor = repmat(log10(rhos), 2, 1);
figure('Name','LGR simetrico'); scatter(real(pol(:)), imag(pol(:)), 6, cor(:)); colorbar; grid on;
xlabel('Re(s)'); ylabel('Im(s)'); title('Polos do LQR x log_{10}(R)');

%% 5) Margins with PX4 + delay for each rho  (C_eq = kp s/(s+kd), Ch. 4)
fprintf('\n   rho     kp     kd    PM(tau)  margem atraso\n');
for rho = [0.05 0.1 0.25 0.5 1 2]
    K = dlqr(Ad, Bd, diag([qp qv]), rho);
    L = K(1)*s/(s + K(2)) * M.Tv/s^2;  L.InputDelay = P.tau;
    am = allmargin(L);  [~, pm] = margin(L);
    fprintf('%6.2f %6.2f %6.2f %8.1f %10.0f ms\n', rho, K, pm, 1000*min(am.DelayMargin));
end

%% 6) Extension: LQR with the PX4 acceleration dynamics (x = [ep ev a])
wa = P.px4.wa;
A3 = [0 1 0; 0 0 1; 0 0 -wa];  B3 = [0; 0; wa];
sys3 = c2d(ss(A3, B3, eye(3), 0), Ts, 'zoh');
K3 = dlqr(sys3.A, sys3.B, diag([qp qv 0.1]), r);
fprintf('\nLQR com modelo da PX4: K = [%.3f %.3f %.3f]\n', K3);

%% 7) State predictor (measurement delay compensation)
%    x(t) = e^{A tau} x(t-tau) + int_0^tau e^{A s} B u ds  (u ~ constant over the delay)
Phi = expm(Ac*P.tau);  Gam = integral(@(x) expm(Ac*x)*Bc, 0, P.tau, 'ArrayValued', true);
fprintf('Preditor: Phi = [%s], Gamma = [%s]\n', num2str(Phi(:)'), num2str(Gam'));

%% 8) Simulacao
R = sim_perseguicao(@ctrl_lqr, P.lqr, cenario('completo'), P, struct('grafico',true));
disp(R.metricas);
