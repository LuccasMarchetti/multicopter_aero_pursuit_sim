function R = sim_perseguicao(ctrl, ganhos, C, P, opt)
%SIM_PERSEGUICAO 3D closed-loop simulation (equivalent to the C++ harness of Ch. 14).
%   R = sim_perseguicao(@ctrl_mpc, P.mpc, cenario('curva'), parametros())
%   R = sim_perseguicao(..., struct('semente',3,'grafico',true))
%
%   Blocos simulados:
%     target (C.vt) -> camera 30 Hz + delay P.tau + noise P.ruido
%     -> relative KF (with/without pursuer velocity compensation)
%     -> controlador a 1/Ts Hz -> aplicar_limites -> (v_sp, a_ff)
%     -> PX4: a_sp = a_ff + Kp(v_sp-v) + Ki*int - Kd*a ; first-order attitude (wa) -> drone
%
%   R: campos t, e_range, e_pos, v_c, a_cmd, metricas (struct), resets
if nargin < 5, opt = struct(); end
semente = getfield_def(opt, 'semente', 1);
grafico = getfield_def(opt, 'grafico', false);
rng(semente);
addpath(fullfile(fileparts(mfilename('fullpath')), 'controladores'));

h = P.h;  Ts = P.Ts;  nT = round(C.T/h);
nc = round(Ts/h);  nm = round(1/P.fcam/h);  nd = round(P.tau/h);
amax_px4 = P.g*tan(P.px4.tilt_max);

% Plant state
pc = [0;0;5];  vc = zeros(3,1);  ac = zeros(3,1);  Iv = zeros(3,1);
pt = pc + C.p0;
v_sp = zeros(3,1);  a_ff = zeros(3,1);  v_ref = zeros(3,1);

% Histories (for the delay and for the KF known input)
VH = zeros(3, nT+1);  ZH = nan(3, nT+1);
kf = kf_init(P);  kf_k = -1;  rej = 0;  resets = 0;
est = [];  mem = [];

nlog = floor(nT/nc) + 1;
L.t = nan(1,nlog);  L.er = nan(1,nlog);  L.ep = nan(3,nlog);
L.ev = nan(3,nlog); L.vc = nan(3,nlog); L.a = nan(3,nlog);  il = 0;

for i = 1:nT
    t = (i-1)*h;
    % ---- target
    vt = C.vt(t);  pt = pt + vt*h;
    % ---- PX4 (velocity loop + first-order attitude), Euler integration
    ev_px4 = v_sp - vc;
    Iv = Iv + P.px4.Ki_v*ev_px4*h;
    a_sp = a_ff + P.px4.Kp_v*ev_px4 + Iv - P.px4.Kd_v*ac;
    a_sp = clampn(a_sp, amax_px4);
    ac = ac + P.px4.wa*(a_sp - ac)*h;
    vc = vc + (ac + C.vento)*h;
    pc = pc + vc*h;
    VH(:,i) = vc;
    % ---- camera: sample now, deliver with delay nd
    if mod(i-1, nm) == 0
        ZH(:,i) = (pt - pc) + P.ruido*randn(3,1);
    end
    j = i - nd;                               % measurement arriving now
    if j >= 1 && ~isnan(ZH(1,j))
        z = ZH(:,j);
        if kf_k < 0
            kf = kf_reset(kf, z, P);  resets = resets + 1;
        else
            dt = (j - kf_k)*h;
            dv = zeros(3,1);
            if P.kf_compensar, dv = VH(:,j) - VH(:,kf_k); end
            kf = kf_predict(kf, dt, dv, P);
            [kf, ok] = kf_update(kf, z, P);
            if ~ok
                rej = rej + 1;
                if rej >= P.kf_max_rej
                    kf = kf_reset(kf, z, P);  rej = 0;  resets = resets + 1;
                end
            else
                rej = 0;
            end
        end
        kf_k = j;
        est.p = kf.x(1:3);  est.v = kf.x(4:6);  est.k = j;
    end
    % ---- controlador
    if mod(i-1, nc) == 0 && ~isempty(est)
        age = (i - est.k)*h;
        st.p_rel = est.p + est.v*age;
        st.v_rel = est.v;
        st.v_chaser = v_ref;
        st.v_target = vc + est.v;            % v_meas + v_rel (as in the node)
        st.dt = Ts;
        [cmd, mem] = ctrl(st, mem, ganhos, P);
        cmd = aplicar_limites(cmd, v_ref, P.vmax, P.amax, Ts);
        v_ref = cmd.v;  v_sp = cmd.v;  a_ff = cmd.a;
        % log with the ground truth (fair evaluation)
        il = il + 1;  rel = pt - pc;
        L.t(il) = t;  L.er(il) = norm(rel) - P.d;
        L.ep(:,il) = standoff_point(rel, P.d);  L.ev(:,il) = vc - vt;
        L.vc(:,il) = vc;  L.a(:,il) = cmd.a;
    end
end
f = @(x) x(:, 1:il);
R.t = L.t(1:il);  R.e_range = L.er(1:il);  R.e_pos = f(L.ep);  R.e_vel = f(L.ev);
R.v_c = f(L.vc);  R.a_cmd = f(L.a);  R.resets = resets - 1;   % the first reset is the initialization
R.metricas = metricas(R, Ts);
if grafico, plotar(R, C); end
end

% =========================================================================
function m = metricas(R, Ts)
% Same definitions as performance_metrics.cpp (time integrals)
T = numel(R.t)*Ts;
ep2 = sum(R.e_pos.^2, 1);  ev2 = sum(R.e_vel.^2, 1);  a = sqrt(sum(R.a_cmd.^2, 1));
m.rmse_pos = sqrt(sum(ep2)*Ts/T);
m.rmse_vel = sqrt(sum(ev2)*Ts/T);
m.max_pos  = max(sqrt(ep2));
m.effort_l2 = sum(a.^2)*Ts;
m.effort_l1 = sum(a)*Ts;
k = find(sqrt(ep2) < 0.5, 1);
if isempty(k), m.t_05 = inf; else, m.t_05 = R.t(k); end
m.std_vx_regime = std(R.v_c(1, R.t > 0.5*R.t(end)));
end

function kf = kf_init(P)
kf.x = zeros(6,1);  kf.P = eye(6);
end

function kf = kf_reset(kf, z, P)
kf.x = [z; 0; 0; 0];
kf.P = blkdiag(P.kf_sigma_z^2*eye(3), P.kf_sigma_v0^2*eye(3));
end

function kf = kf_predict(kf, dt, dv, P)
% Known input: change of the pursuer velocity (Ch. 14)
F = [eye(3) dt*eye(3); zeros(3) eye(3)];
q = P.kf_sigma_a^2;
Q = [dt^4/4*eye(3) dt^3/2*eye(3); dt^3/2*eye(3) dt^2*eye(3)]*q;
kf.x = F*kf.x - [0.5*dt*dv; dv];
kf.P = F*kf.P*F' + Q;
end

function [kf, ok] = kf_update(kf, z, P)
H = [eye(3) zeros(3)];
y = z - H*kf.x;
S = H*kf.P*H' + P.kf_sigma_z^2*eye(3);
ok = ~(P.kf_gate > 0 && y'*(S\y) > P.kf_gate);
if ~ok, return; end
K = kf.P*H'/S;
kf.x = kf.x + K*y;
kf.P = (eye(6) - K*H)*kf.P;
kf.P = 0.5*(kf.P + kf.P');
end

function v = clampn(v, m)
n = norm(v);
if n > m, v = v*m/n; end
end

function v = getfield_def(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end

function plotar(R, C)
figure('Name', ['Perseguicao - ' C.nome]);
subplot(3,1,1); plot(R.t, R.e_range); grid on; ylabel('|r| - d [m]');
title(sprintf('%s | RMSE pos %.3f m | resets KF %d', C.nome, R.metricas.rmse_pos, R.resets));
subplot(3,1,2); plot(R.t, R.v_c'); grid on; ylabel('v_c [m/s]'); legend('x','y','z');
subplot(3,1,3); plot(R.t, sqrt(sum(R.a_cmd.^2,1))); grid on; ylabel('|a| [m/s^2]'); xlabel('t [s]');
end
