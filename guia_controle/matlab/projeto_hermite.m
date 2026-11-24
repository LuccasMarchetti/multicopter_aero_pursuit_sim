%PROJETO_HERMITE Quintic planning: optimal duration, equivalent gain, weights.
%   Chapter 11 of the guide.
clear; clc; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), 'controladores'));
P = parametros();  g = P.her;

%% 1) Theoretical optimal duration (rest-to-rest, no limits): T* = (3600 wj D^2 / wT)^(1/6)
D = 0.5:0.5:6;
Tstar = (3600*g.wj*D.^2/g.wT).^(1/6);
figure('Name','T otimo'); plot(D, Tstar, '-o'); grid on;
xlabel('\Delta [m]'); ylabel('T^* [s]'); title('Duracao otima do segmento');

%% 2) Equivalent gain by numerical perturbation (depends on the error size!)
st.v_rel = zeros(3,1);  st.v_chaser = zeros(3,1);  st.v_target = zeros(3,1);  st.dt = P.Ts;
fprintf('   erro[m]   a/e_p [1/s^2]\n');
for e = [0.1 0.5 1 2 4]
    st.p_rel = [P.d + e; 0; 0];
    cmd = ctrl_hermite(st, [], g, P);
    fprintf('%9.2f %12.3f\n', e, cmd.a(1)/e);
end
fprintf('(ganho que cai com o erro = controlador nao linear: suave longe, firme perto)\n');

%% 3) Effect of wT and wj in the simulation
for wT = [0.5 1 2]
    gg = g;  gg.wT = wT;
    R = sim_perseguicao(@ctrl_hermite, gg, cenario('curva'), P);
    fprintf('wT=%.1f: RMSE %.3f m, esforco L2 %.1f\n', wT, R.metricas.rmse_pos, R.metricas.effort_l2);
end
