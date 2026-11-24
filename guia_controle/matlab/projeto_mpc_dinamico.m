%PROJETO_MPC_DINAMICO Rigid-body model of the x500 (horizontal axis), ZOH and LQR.
%   Chapter 10 of the guide. Compares the C++ model (torque as input) with a model
%   that includes the PX4 attitude loop (attitude as input).
%   The LQR of each model is the N -> infinity limit of the corresponding unconstrained MPC.
clear; clc; close all;
P = parametros();  Ts = P.Ts;
tau_max = 0.5*P.tau_max;          % torque_authority = 0,5

%% (a) Code model: x = [p v theta omega], u = tau/tau_max
Aa = [0 1 0 0; 0 -P.D/P.m P.g 0; 0 0 0 1; 0 0 0 0];
Ba = [0; 0; 0; tau_max/P.Iyy];
fprintf('tau_max/I = %.1f rad/s^2 | D/m = %.3f 1/s\n', tau_max/P.Iyy, P.D/P.m);

%% (b) Model with the PX4 attitude loop: theta' = wa (theta_sp - theta), u = theta_sp
wa = P.px4.wa;
Ab = [0 1 0; 0 -P.D/P.m P.g; 0 0 -wa];
Bb = [0; 0; wa];

%% ZOH discretization (equivalent to discretizeZoh in the C++)
da = c2d(ss(Aa, Ba, eye(4), 0), Ts, 'zoh');
db = c2d(ss(Ab, Bb, eye(3), 0), Ts, 'zoh');

%% LQR with the yaml weights (weight_pos, weight_vel, weight_tilt, weight_rate, weight_torque)
Ka = dlqr(da.A, da.B, diag([10 2 1 0.05]), 0.1);
Kb = dlqr(db.A, db.B, diag([10 2 1]), 5);      % R on theta_sp [rad]
fprintf('K (modelo torque)   = [%s]\n', num2str(Ka, '%.3f '));
fprintf('K (modelo atitude)  = [%s]\n', num2str(Kb, '%.3f '));

%% 1 m position step in closed loop
ca = ss(da.A - da.B*Ka, da.B*Ka(1), [1 0 0 0], 0, Ts);
cb = ss(db.A - db.B*Kb, db.B*Kb(1), [1 0 0], 0, Ts);
figure('Name','MPC dinamico: degrau'); step(ca, cb, 6); grid on;
legend('modelo torque (C++)', 'modelo atitude (PX4)');
stepinfo(ca), stepinfo(cb)

%% Closed-loop poles (continuous equivalent)
disp('Polos (modelo torque):');  disp(log(eig(da.A - da.B*Ka)).'/Ts);
disp('Polos (modelo atitude):'); disp(log(eig(db.A - db.B*Kb)).'/Ts);
