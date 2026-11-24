%COMPARAR_CONTROLADORES Monte Carlo of every controller in every scenario.
%   Chapter 13 of the guide. Produces a table (mean +- std), boxplots, Pareto and tests
%   statistical tests. To use optimized gains, run first:
%       G.pid = otimizar_ganhos('pid'); ... ; save ganhos_otimos.mat G
clear; clc; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), 'controladores'));
P = parametros();

ctrls = { 'PID PBVS', @ctrl_pid,       P.pid;
          'LQR',      @ctrl_lqr,       P.lqr;
          'LQI',      @ctrl_lqi,       P.lqi;
          'ASMC',     @ctrl_smc,       P.smc;
          'MPC',      @ctrl_mpc,       P.mpc;
          'Hermite',  @ctrl_hermite,   P.her;
          'PN/PP',    @ctrl_guiamento, P.pn };
if exist('ganhos_otimos.mat', 'file')       % replaces with the optimized gains
    S = load('ganhos_otimos.mat');  mapa = {'pid','lqr','lqi','smc','mpc','hermite','pn'};
    for i = 1:numel(mapa)
        if isfield(S.G, mapa{i}), ctrls{i,3} = S.G.(mapa{i}); end
    end
    disp('Usando ganhos de ganhos_otimos.mat');
end
cenarios = {'regulacao', 'reta', 'curva', 'para_anda', 'zigue'};
n_mc = 10;                                  % seeds per scenario (30 for the thesis)
metr = {'rmse_pos', 'rmse_vel', 'max_pos', 'effort_l2', 't_05'};

nC = size(ctrls, 1);  nS = numel(cenarios);
Res = nan(nC, nS, n_mc, numel(metr));
Resets = zeros(nC, nS, n_mc);
for i = 1:nC
    for j = 1:nS
        for k = 1:n_mc
            R = sim_perseguicao(ctrls{i,2}, ctrls{i,3}, cenario(cenarios{j}), P, ...
                                struct('semente', k));
            for m = 1:numel(metr), Res(i,j,k,m) = R.metricas.(metr{m}); end
            Resets(i,j,k) = R.resets;
        end
        fprintf('%-9s %-10s RMSE %.3f +- %.3f m\n', ctrls{i,1}, cenarios{j}, ...
                mean(Res(i,j,:,1)), std(Res(i,j,:,1)));
    end
end
save resultados_comparacao.mat Res Resets ctrls cenarios metr

%% Table (mean over scenarios and seeds)
fprintf('\n%-9s %10s %10s %10s %10s %8s\n', 'ctrl', 'RMSEpos', 'RMSEvel', 'max', 'L2', 'resets');
for i = 1:nC
    x = squeeze(mean(mean(Res(i,:,:,:), 2), 3));
    fprintf('%-9s %10.3f %10.3f %10.3f %10.1f %8.2f\n', ctrls{i,1}, x(1:4), ...
            mean(Resets(i,:,:), 'all'));
end

%% Boxplot of the position RMSE per scenario
figure('Name','RMSE por cenario');
for j = 1:nS
    subplot(1, nS, j);
    boxplot(squeeze(Res(:,j,:,1))', ctrls(:,1)); title(cenarios{j});
    ylabel('RMSE pos [m]'); xtickangle(45); grid on;
end

%% Pareto: position RMSE x effort
figure('Name','Pareto'); hold on; grid on;
for i = 1:nC
    x = squeeze(mean(Res(i,:,:,4), 3));  y = squeeze(mean(Res(i,:,:,1), 3));
    scatter(mean(x), mean(y), 60, 'filled', 'DisplayName', ctrls{i,1});
end
xlabel('esforco L2 [m^2/s^3]'); ylabel('RMSE pos [m]'); legend show;

%% Estatistica (Statistics Toolbox): Kruskal-Wallis e Wilcoxon pareado vs LQR
if exist('kruskalwallis', 'file')
    for j = 1:nS
        X = squeeze(Res(:,j,:,1))';
        p = kruskalwallis(X, ctrls(:,1), 'off');
        fprintf('%-10s Kruskal-Wallis p = %.2e\n', cenarios{j}, p);
        for i = [1 3:nC]
            pw = signrank(X(:,i), X(:,2));
            fprintf('   %-9s vs LQR: Wilcoxon p = %.3f\n', ctrls{i,1}, pw);
        end
    end
end
