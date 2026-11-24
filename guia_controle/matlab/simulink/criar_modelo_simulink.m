function mdl = criar_modelo_simulink()
%CRIAR_MODELO_SIMULINK Generates perseguicao_1eixo.slx (one axis, along the LOS).
%
%   Diagrama:
%     target: Step(vt) -> Integrator -> p_t
%     PX4 : a_sp = a_ff + Kp (v_sp - v) + Ki int(v_sp - v) - Kd a ;  a = wa/(s+wa) a_sp
%           v = int a ; p_c = int v
%     medida: r = p_t - p_c e rdot = v_t - v_c  -> Transport Delay(tau) -> ZOH(Ts)
%     controlador (MATLAB Function, discreto via ZOH): tipo 1=PID 2=LQR 3=LQI 4=SMC
%
%   Variables read from the base workspace (defined by rodar_simulink.m):
%     Ts tau d vt_alvo r0 Kp_v Ki_v Kd_v wa tipo g_ctrl
mdl = 'perseguicao_1eixo';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
set_param(mdl, 'StopTime', '30', 'Solver', 'ode45', 'MaxStep', '0.005');

B = @(nome) [mdl '/' nome];
add = @(lib, nome, pos, varargin) add_block(lib, B(nome), 'Position', pos, varargin{:});

% ---------------- Alvo
add('simulink/Sources/Step', 'vt', [30 40 60 70], 'Time', '2', 'Before', '0', 'After', 'vt_alvo');
add('simulink/Continuous/Integrator', 'pt', [110 40 140 70], 'InitialCondition', 'r0');

% ---------------- PX4 (velocity loop + attitude)
add('simulink/Math Operations/Sum', 'ev_px4', [520 200 540 220], 'Inputs', '+-', 'IconShape', 'round');
add('simulink/Math Operations/Gain', 'Kp', [580 190 620 230], 'Gain', 'Kp_v');
add('simulink/Math Operations/Gain', 'Ki', [580 250 620 290], 'Gain', 'Ki_v');
add('simulink/Continuous/Integrator', 'Iv', [650 250 680 290]);
add('simulink/Math Operations/Sum', 'a_sp', [720 190 740 300], 'Inputs', '++++');
add('simulink/Math Operations/Gain', 'Kd', [720 340 760 370], 'Gain', '-Kd_v', 'Orientation', 'left');
add('simulink/Continuous/Transfer Fcn', 'atitude', [780 225 860 265], ...
    'Numerator', '[wa]', 'Denominator', '[1 wa]');
add('simulink/Continuous/Integrator', 'vc', [900 230 930 260]);
add('simulink/Continuous/Integrator', 'pc', [970 230 1000 260]);

% ---------------- Medida (relativa, atrasada, amostrada)
add('simulink/Math Operations/Sum', 'r', [180 40 200 60], 'Inputs', '+-', 'IconShape', 'round');
add('simulink/Math Operations/Sum', 'rdot', [180 100 200 120], 'Inputs', '+-', 'IconShape', 'round');
add('simulink/Continuous/Transport Delay', 'atraso_r', [230 35 280 65], 'DelayTime', 'tau');
add('simulink/Continuous/Transport Delay', 'atraso_rd', [230 95 280 125], 'DelayTime', 'tau');
add('simulink/Discrete/Zero-Order Hold', 'zoh_r', [300 35 330 65], 'SampleTime', 'Ts');
add('simulink/Discrete/Zero-Order Hold', 'zoh_rd', [300 95 330 125], 'SampleTime', 'Ts');
add('simulink/Discrete/Zero-Order Hold', 'zoh_vt', [300 155 330 185], 'SampleTime', 'Ts');

% ---------------- Controlador
add('simulink/Sources/Constant', 'tipo', [300 210 330 230], 'Value', 'tipo');
add('simulink/Sources/Constant', 'Ts_c', [300 245 330 265], 'Value', 'Ts');
add('simulink/Sources/Constant', 'd', [300 280 330 300], 'Value', 'd');
add('simulink/Sources/Constant', 'g', [300 315 330 335], 'Value', 'g_ctrl');
add('simulink/User-Defined Functions/MATLAB Function', 'controlador', [380 30 470 340]);
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', B('controlador'));
ch.Script = codigo_controlador();

% ---------------- Saidas
add('simulink/Math Operations/Sum', 'erro', [1050 40 1070 60], 'Inputs', '+-', 'IconShape', 'round');
add('simulink/Sinks/To Workspace', 'log_erro', [1100 35 1160 65], ...
    'VariableName', 'erro_dist', 'SaveFormat', 'Timeseries');
add('simulink/Sinks/To Workspace', 'log_v', [1100 95 1160 125], ...
    'VariableName', 'v_chaser', 'SaveFormat', 'Timeseries');
add('simulink/Sinks/To Workspace', 'log_a', [1100 155 1160 185], ...
    'VariableName', 'a_cmd', 'SaveFormat', 'Timeseries');
add('simulink/Sinks/Scope', 'Scope', [1100 220 1130 250], 'NumInputPorts', '2');

% ---------------- Ligacoes
L = @(a, b) add_line(mdl, a, b, 'autorouting', 'on');
L('vt/1', 'pt/1');
L('pt/1', 'r/1');          L('pc/1', 'r/2');
L('vt/1', 'rdot/1');       L('vc/1', 'rdot/2');
L('r/1', 'atraso_r/1');    L('atraso_r/1', 'zoh_r/1');
L('rdot/1', 'atraso_rd/1'); L('atraso_rd/1', 'zoh_rd/1');
L('vt/1', 'zoh_vt/1');
L('zoh_r/1', 'controlador/1');  L('zoh_rd/1', 'controlador/2');  L('zoh_vt/1', 'controlador/3');
L('tipo/1', 'controlador/4');   L('Ts_c/1', 'controlador/5');
L('d/1', 'controlador/6');      L('g/1', 'controlador/7');
L('controlador/1', 'ev_px4/1'); L('vc/1', 'ev_px4/2');
L('ev_px4/1', 'Kp/1');          L('ev_px4/1', 'Ki/1');   L('Ki/1', 'Iv/1');
L('controlador/2', 'a_sp/1');   L('Kp/1', 'a_sp/2');     L('Iv/1', 'a_sp/3');
L('Kd/1', 'a_sp/4');
L('a_sp/1', 'atitude/1');       L('atitude/1', 'vc/1');  L('atitude/1', 'Kd/1');
L('vc/1', 'pc/1');
L('r/1', 'erro/1');             L('d/1', 'erro/2');
L('erro/1', 'log_erro/1');      L('vc/1', 'log_v/1');    L('controlador/2', 'log_a/1');
L('erro/1', 'Scope/1');         L('vc/1', 'Scope/2');

save_system(mdl, fullfile(fileparts(mfilename('fullpath')), [mdl '.slx']));
fprintf('Modelo salvo: %s.slx\n', mdl);
end

% -------------------------------------------------------------------------
function c = codigo_controlador()
c = strjoin({
'function [v_sp, a_ff] = controlador(r, rdot, vt, tipo, Ts, d, g)'
'% Leis de controle 1D ao longo da LOS (mesmas equacoes do C++).'
'% g = [kp ki kd ff | K_lqr(2) | K_lqi(3) | lambda ks k0 kmax gamma sigma phi | vmax amax]'
'persistent vref xi khat'
'if isempty(vref), vref = 0; xi = 0; khat = g(12); end'
'ep = d - r;          % perseguidor - ponto desejado'
'ev = vref - vt;      % velocidade de referencia - alvo'
'switch tipo'
'    case 1  % PID PBVS (saida em velocidade)'
'        e = r - d;  xi = xi + e*Ts;'
'        v = g(4)*vt + g(1)*e + g(2)*xi + g(3)*rdot;'
'    case 2  % LQR'
'        v = vref + (-(g(5)*ep + g(6)*ev))*Ts;'
'    case 3  % LQI'
'        xi = xi + ep*Ts;'
'        v = vref + (-(g(7)*ep + g(8)*ev + g(9)*xi))*Ts;'
'    otherwise  % ASMC'
'        s = ev + g(10)*ep;'
'        if abs(s) > g(16)'
'            khat = min(max(khat + g(14)*(abs(s) - g(15)*khat)*Ts, 0), g(13));'
'        end'
'        a = -g(10)*ev - g(11)*s - khat*s/max(abs(s), g(16));'
'        v = vref + a*Ts;'
'end'
'% applyLimits: |a| <= amax, |v| <= vmax'
'a = min(max((v - vref)/Ts, -g(18)), g(18));'
'v = min(max(vref + a*Ts, -g(17)), g(17));'
'vref = v;  v_sp = v;  a_ff = a;'
'end'}, newline);
end
