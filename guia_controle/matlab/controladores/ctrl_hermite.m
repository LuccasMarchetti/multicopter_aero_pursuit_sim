function [cmd, mem] = ctrl_hermite(st, mem, g, P)
%CTRL_HERMITE Replanejamento quintico + planeza (hermite_flatness_controller.cpp).
%   One segment per axis; duration T optimized by golden-section search.
%   g: wT, wj, wlim, Tmin, Tmax, M
if isempty(mem), mem.a_prev = zeros(3,1); end
custo = @(T) custo_plano(plano(st, mem.a_prev, T, P), g, P);
r = 0.6180339887498949;
lo = g.Tmin;  hi = g.Tmax;
t1 = hi - r*(hi - lo);  t2 = lo + r*(hi - lo);
c1 = custo(t1);  c2 = custo(t2);
for it = 1:30
    if hi - lo < 1e-3, break; end
    if c1 < c2
        hi = t2; t2 = t1; c2 = c1; t1 = hi - r*(hi - lo); c1 = custo(t1);
    else
        lo = t1; t1 = t2; c1 = c2; t2 = lo + r*(hi - lo); c2 = custo(t2);
    end
end
C = plano(st, mem.a_prev, 0.5*(lo + hi), P);
[~, v, a, j] = avaliar(C, st.dt);
cmd.v = v;  cmd.a = a;  cmd.j = j;
mem.a_prev = a;
end

% -------------------------------------------------------------------------
function C = plano(st, a0, T, P)
% Quintic coefficients (6 x 3) with p0=0, v0=v_ref, a0; p1=standoff(target at T), v1=v_t, a1=0
p0 = zeros(3,1);  v0 = st.v_chaser;
p1 = standoff_point(st.p_rel + st.v_target*T, P.d);  v1 = st.v_target;  a1 = zeros(3,1);
c0 = p0;  c1 = v0;  c2 = a0/2;
c3 = (20*(p1-p0) - (8*v1 + 12*v0)*T - (3*a0 - a1)*T^2)/(2*T^3);
c4 = (30*(p0-p1) + (14*v1 + 16*v0)*T + (3*a0 - 2*a1)*T^2)/(2*T^4);
c5 = (12*(p1-p0) - 6*(v1 + v0)*T - (a0 - a1)*T^2)/(2*T^5);
C.c = [c0 c1 c2 c3 c4 c5]';   % 6 x 3
C.T = T;
end

function [p, v, a, j] = avaliar(C, t)
c = C.c;
p = (c(1,:) + c(2,:)*t + c(3,:)*t^2 + c(4,:)*t^3 + c(5,:)*t^4 + c(6,:)*t^5)';
v = (c(2,:) + 2*c(3,:)*t + 3*c(4,:)*t^2 + 4*c(5,:)*t^3 + 5*c(6,:)*t^4)';
a = (2*c(3,:) + 6*c(4,:)*t + 12*c(5,:)*t^2 + 20*c(6,:)*t^3)';
j = (6*c(4,:) + 24*c(5,:)*t + 60*c(6,:)*t^2)';
end

function J = custo_plano(C, g, P)
T = C.T;  c = C.c;
% Integral of |jerk|^2: j = A + B t + D t^2
A = 6*c(4,:);  B = 24*c(5,:);  D = 60*c(6,:);
Ij = sum(A.^2*T + A.*B*T^2 + (B.^2 + 2*A.*D)*T^3/3 + B.*D*T^4/2 + D.^2*T^5/5);
viol = 0;
for i = 1:g.M
    [~, v, a] = avaliar(C, T*i/g.M);
    viol = viol + max(0, v'*v - P.vmax^2)^2 + max(0, a'*a - P.amax^2)^2;
end
J = g.wT*T + g.wj*Ij + g.wlim*viol/g.M;
end
