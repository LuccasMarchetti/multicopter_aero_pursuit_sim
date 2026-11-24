function [g_otimo, J_otimo, hist] = otimizar_ganhos(nome, opt)
%OTIMIZAR_GANHOS Optimal tuning of a controller's parameters by the common cost (Ch. 13).
%   [g, J] = otimizar_ganhos('pid')       % 'pid'|'lqr'|'lqi'|'smc'|'mpc'|'hermite'|'pn'
%   opt.cenarios  : training scenarios   (default {'regulacao','curva','para_anda'})
%   opt.atrasos   : atrasos avaliados    (padrao [P.tau, P.tau+0.05] -> robustez)
%   opt.max_iter  : fminsearch iterations (default 60)
%
%   J = mean_scenarios[(1/T) int(|e_p|^2 + wv |e_v|^2 + rho |a|^2) dt] + reset penalty
%   The search is done in log(theta) to keep the parameters positive.
if nargin < 2, opt = struct(); end
P = parametros();
addpath(fullfile(fileparts(mfilename('fullpath')), 'controladores'));
cen = def(opt, 'cenarios', {'regulacao', 'curva', 'para_anda'});
atr = def(opt, 'atrasos', [P.tau, P.tau + 0.05]);
it  = def(opt, 'max_iter', 60);
wv = 0.2;  rho = 0.05;

[ctrl, g0, campos] = espaco(nome, P);
theta0 = log(cellfun(@(c) g0.(c), campos));
hist = [];

    function J = custo(lt)
        g = g0;
        for k = 1:numel(campos), g.(campos{k}) = exp(lt(k)); end
        J = 0;
        for a = atr
            Pa = P;  Pa.tau = a;
            for c = 1:numel(cen)
                R = sim_perseguicao(ctrl, g, cenario(cen{c}), Pa);
                ep2 = sum(R.e_pos.^2, 1);  ev2 = sum(R.e_vel.^2, 1);  a2 = sum(R.a_cmd.^2, 1);
                J = J + mean(ep2 + wv*ev2 + rho*a2) + 1.0*R.resets;
            end
        end
        J = J/(numel(atr)*numel(cen));
        hist(end+1) = J; %#ok<AGROW>
    end

o = optimset('MaxIter', it, 'Display', 'iter', 'TolX', 1e-3, 'TolFun', 1e-4);
[lt, J_otimo] = fminsearch(@custo, theta0, o);
g_otimo = g0;
for k = 1:numel(campos), g_otimo.(campos{k}) = exp(lt(k)); end
fprintf('\n%s: J inicial %.4f -> J otimo %.4f\n', nome, hist(1), J_otimo);
for k = 1:numel(campos)
    fprintf('  %-8s %8.4f -> %8.4f\n', campos{k}, g0.(campos{k}), g_otimo.(campos{k}));
end
end

% -------------------------------------------------------------------------
function [ctrl, g, campos] = espaco(nome, P)
switch nome
    case 'pid',     ctrl = @ctrl_pid;       g = P.pid; campos = {'kp','ki','kd'};
    case 'lqr',     ctrl = @ctrl_lqr;       g = P.lqr; campos = {'qp','qv'};      % r fixo
    case 'lqi',     ctrl = @ctrl_lqi;       g = P.lqi; campos = {'qp','qv','qi'};
    case 'smc',     ctrl = @ctrl_smc;       g = P.smc; campos = {'lambda','ks','phi'};
    case 'mpc',     ctrl = @ctrl_mpc;       g = P.mpc; campos = {'qp','qv'};
    case 'hermite', ctrl = @ctrl_hermite;   g = P.her; campos = {'wT','wj'};
    case 'pn',      ctrl = @ctrl_guiamento; g = P.pn;  campos = {'kr','krd','klat'};
    otherwise, error('controlador desconhecido: %s', nome);
end
end

function v = def(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end
