function C = cenario(nome)
%CENARIO Target trajectories used in the comparison (Ch. 13).
%   C.vt(t) -> target velocity (3x1);  C.T duration;  C.p0 initial relative position
%   Nomes: 'regulacao' | 'reta' | 'curva' | 'para_anda' | 'zigue' | 'rapido' | 'completo'
C.nome = nome;
C.p0 = [8; 0; 0];          % target 8 m ahead (3 m of error with d = 5 m)
C.T = 30;
C.vento = [0; 0; 0];       % disturbance acceleration on the pursuer [m/s^2]
switch nome
    case 'regulacao'
        C.vt = @(t) [0; 0; 0];  C.T = 15;
    case 'reta'
        C.vt = @(t) [1.5; 0; 0];
    case 'curva'
        C.vt = @(t) vel_curva(t, 1.5, 0.3, 5, 15);
    case 'para_anda'
        C.vt = @(t) [1.5*(mod(t, 8) < 4); 0; 0];
    case 'zigue'
        C.vt = @(t) [1.2; 1.0*sign(sin(2*pi*t/6)); 0];
    case 'rapido'
        C.vt = @(t) [2.5; 0; 0];   % above vmax: saturation
    case 'completo'                % same as the simulation of Ch. 14
        C.vt = @(t) vel_completo(t);  C.T = 40;
    otherwise
        error('cenario desconhecido: %s', nome);
end
end

function v = vel_curva(t, V, w, t0, t1)
if t < t0
    v = [V; 0; 0];
elseif t < t1
    v = [V*cos(w*(t - t0)); V*sin(w*(t - t0)); 0];
else
    v = [V*cos(w*(t1 - t0)); V*sin(w*(t1 - t0)); 0];
end
end

function v = vel_completo(t)
if t >= 30
    v = [0; 0; 0];
else
    v = vel_curva(t, 1.5, 0.3, 15, 25);
end
end
