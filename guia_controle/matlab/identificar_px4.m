%IDENTIFICAR_PX4 Identifies the PX4 velocity loop from a step test.
%   Chapter 4 of the guide.
%
%   How to produce the CSV (in the container/ROS):
%     ros2 bag record /offboard/trajectory_setpoint /fmu/out/vehicle_local_position_v1
%     (apply velocity steps in x with the drone in offboard)
%     ros2 bag ... -> export to CSV with columns:  t, vx_sp, ax_ff, vx
%     (for example with Foxglove "Export" or a rosbag2_py script)
%   Without a CSV, the script generates synthetic data from modelo_px4 for demonstration.
clear; clc; close all;
P = parametros();
arquivo = 'ensaio_degrau.csv';

if exist(arquivo, 'file')
    D = readmatrix(arquivo);
    t = D(:,1) - D(1,1);  u = D(:,2);  y = D(:,4);
    Tsd = median(diff(t));
    ti = (0:Tsd:t(end))';                       % reamostragem uniforme
    u = interp1(t, u, ti, 'previous');  y = interp1(t, y, ti);  t = ti;
else
    warning('Sem %s: usando dados sinteticos do modelo_px4 (sem feedforward).', arquivo);
    M = modelo_px4(P, false);
    Tsd = 0.02;  t = (0:Tsd:12)';
    u = 1.0*(t > 1) - 1.0*(t > 6);
    y = lsim(M.Tv, u, t) + 0.01*randn(size(t));
end

%% 1) First-order model with delay by least squares (no extra toolbox)
modelo = @(th) lsim(tf(th(1), [th(2) 1], 'InputDelay', th(3)), u, t);
J = @(th) sum((y - modelo(abs(th))).^2);
th = abs(fminsearch(J, [1 0.5 0.05]));
fprintf('FOPDT: K=%.3f  tau=%.3f s  atraso=%.3f s\n', th);

%% 2) Second-order model (System Identification Toolbox, if available)
if exist('tfest', 'file')
    dados = iddata(y, u, Tsd);
    G2 = tfest(dados, 2, 1);
    disp(G2);
    figure('Name','Identificacao'); compare(dados, G2, tf(th(1), [th(2) 1], 'InputDelay', th(3)));
else
    figure('Name','Identificacao'); plot(t, y, t, modelo(th), '--'); grid on;
    legend('medido','FOPDT'); xlabel('t [s]'); ylabel('v_x [m/s]');
end
fprintf('Use o modelo identificado no lugar de M.Tv em modelo_px4.m.\n');
