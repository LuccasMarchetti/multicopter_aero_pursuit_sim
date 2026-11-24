%PROJETO_MPC Kinematic MPC: equivalent gain, horizon, terminal weight, quadprog, toolbox.
%   Chapter 9 of the guide.
clear; clc; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), 'controladores'));
P = parametros();  M = modelo_px4(P, true);  Ts = P.Ts;  s = tf('s');
g = P.mpc;
Ad = [1 Ts; 0 1];  Bd = [Ts^2/2; Ts];

%% 1) Equivalent gain (unconstrained) x LQR
K_lqr = dlqr(Ad, Bd, diag([g.qp g.qv]), g.r);
K_mpc = mpc_ganho_equivalente(P);
Mm = mpc_matrizes(g, Ts);
fprintf('Horizonte T_H = %.3f s | K_MPC = [%.3f %.3f] | K_LQR = [%.3f %.3f]\n', ...
        Mm.t(end), K_mpc, K_lqr);

%% 2) Effect of the horizon (N and growth) on the equivalent gain
fprintf('\n   N  growth   T_H     kp     kd    PM(tau)\n');
for N = [5 10 15 20 40]
    for gr = [1.0 1.15]
        gg = g;  gg.N = N;  gg.growth = gr;
        K = mpc_ganho_equivalente(P, gg);  Mg = mpc_matrizes(gg, Ts);
        L = K(1)*s/(s + K(2))*M.Tv/s^2;  L.InputDelay = P.tau;  [~, pm] = margin(L);
        fprintf('%4d %6.2f %6.2f %6.2f %6.2f %8.1f\n', N, gr, Mg.t(end), K, pm);
    end
end

%% 3) Terminal weight = DARE P: unconstrained MPC == LQR for any N
%     (grade uniforme, growth = 1)
[~, Pdare] = dare_ganho(Ad, Bd, diag([g.qp g.qv]), g.r);
for N = [3 10]
    Mu = mpc_matrizes(setfield(setfield(g, 'N', N), 'growth', 1.0), Ts); %#ok<SFLD>
    Qb = Mu.Qb;  Qb(end-1:end, end-1:end) = Pdare;
    H = 2*(Mu.Su'*Qb*Mu.Su + g.r*eye(N));  F = 2*Mu.Su'*Qb*Mu.Sx;
    U = -(H\F);
    fprintf('N=%2d com P terminal: K = [%.4f %.4f]  (LQR [%.4f %.4f])\n', N, -U(1,:), K_lqr);
end
% Note: the C++ cost includes the states x_1..x_N and u_0..u_{N-1}; the exact identity with the
% LQR holds when the last weight is P (the first state x_0 is not in the cost).

%% 4) Check of FISTA against quadprog (Optimization Toolbox), one axis
st.p_rel = [9; 0; 0];  st.v_rel = zeros(3,1);  st.v_chaser = zeros(3,1);
st.v_target = zeros(3,1);  st.dt = Ts;
[cmd, mem] = ctrl_mpc(st, [], g, P);
if exist('quadprog', 'file')
    S0 = [0; 0];  Xref = repmat([st.p_rel(1) - P.d; 0], g.N, 1);
    f = 2*Mm.Su'*Mm.Qb*(Mm.Sx*S0 - Xref);
    U = quadprog(Mm.H, f, [], [], [], [], -P.amax*ones(g.N,1), P.amax*ones(g.N,1), [], ...
                 optimoptions('quadprog', 'Display', 'off'));
    fprintf('\nu0: FISTA %.4f | quadprog (sem restr. de vel.) %.4f\n', cmd.a(1), U(1));
end

%% 5) MPC Toolbox (optional): same problem per axis
if exist('mpc', 'file')
    plant = ss(Ad, Bd, eye(2), 0, Ts);
    obj = mpc(plant, Ts, g.N, g.N);
    obj.Weights.OutputVariables = [sqrt(g.qp) sqrt(g.qv)];
    obj.Weights.ManipulatedVariables = sqrt(g.r);
    obj.Weights.ManipulatedVariablesRate = 0;
    obj.MV.Min = -P.amax;  obj.MV.Max = P.amax;
    obj.OV(2).Min = -P.vmax;  obj.OV(2).Max = P.vmax;
    disp(obj);
    % Uso: x = mpcstate(obj); u = mpcmove(obj, x, y_medido, [0 0]);
end

%% 6) Simulations: comparison scenarios
for nome = {'reta', 'curva', 'completo'}
    R = sim_perseguicao(@ctrl_mpc, g, cenario(nome{1}), P, struct('grafico', true));
    fprintf('%-10s RMSE pos %.3f m | resets KF %d\n', nome{1}, R.metricas.rmse_pos, R.resets);
end
