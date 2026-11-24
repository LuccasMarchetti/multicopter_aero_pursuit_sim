function [cmd, mem] = ctrl_mpc(st, mem, g, P)
%CTRL_MPC Kinematic LTV MPC with constraints (replicates mpc_controller.cpp, online mode).
%   - grade crescente dt_k = Ts*growth^k
%   - reference on the predicted LOS (the MPC's own previous trajectory)
%   - v_ref only with the LOS rotation induced by the target (fix of Ch. 14)
%   - |u_k| <= amax (projecao) e |v_k| <= vmax (Lagrangiano aumentado) via FISTA
%   g: N, qp, qv, r, growth, wN, iter, rho, outer, predict_los, vel_con
if isempty(mem)
    mem.M = mpc_matrizes(g, P.Ts);
    mem.U = zeros(g.N, 3);
    mem.lam = zeros(g.N, 1);
    mem.path = [];
end
M = mem.M;  N = M.N;  d = P.d;
los0 = unit_or(st.p_rel, [1;0;0]);

% ---- Time-varying reference
Xref = zeros(2*N, 3);
have_prev = g.predict_los && ~isempty(mem.path);
for k = 1:N
    pt = st.p_rel + st.v_target*M.t(k);
    los = los0;
    vr = st.v_target;
    if g.predict_los
        if have_prev
            c = prev_chaser(mem, M.t(k), st.dt);
        else
            c = st.v_chaser*M.t(k);
        end
        line = pt - c;
        los = unit_or(line, los0);
        ratio = d/max(norm(line), d);
        vr = st.v_target - ratio*(st.v_target - los*(los'*st.v_target));
    end
    Xref(2*k-1, :) = (pt - d*los)';
    Xref(2*k,   :) = vr';
end

% ---- Linear term of the QP (per axis)
S0 = [zeros(1,3); st.v_chaser'];            % 2 x 3: [p0; v0] por eixo
G  = 2*M.Su'*M.Qb*(M.Sx*S0 - Xref);         % N x 3
Vfree = M.Sv_x*S0;                          % N x 3

% ---- Solucao (FISTA + Lagrangiano aumentado)
U = [mem.U(2:end,:); mem.U(end,:)];         % warm start deslocado
lam = [mem.lam(2:end); mem.lam(end)];
n_outer = 1;
if g.vel_con, n_outer = max(1, g.outer); end
for o = 1:n_outer
    U = fista(U, G, Vfree, lam, M, g, P);
    if g.vel_con
        V = Vfree + M.Sv_u*U;
        lam = max(0, lam + g.rho*(sqrt(sum(V.^2, 2)) - P.vmax));
    end
end
mem.U = U;  mem.lam = lam;

cmd.a = U(1,:)';
cmd.v = st.v_chaser + cmd.a*st.dt;
X = M.Sx*S0 + M.Su*U;
mem.path = X(1:2:end, :);                    % N x 3 posicoes previstas
end

% -------------------------------------------------------------------------
function U = fista(U, G, Vfree, lam, M, g, P)
step = 1/M.L;  y = U;  Uprev = U;  t = 1;
for it = 1:g.iter
    grad = M.H*y + G;
    if g.vel_con
        V = Vfree + M.Sv_u*y;
        n = sqrt(sum(V.^2, 2));
        act = n - P.vmax + lam/g.rho;
        h = zeros(size(V));
        idx = act > 0 & n > 1e-9;
        h(idx,:) = (g.rho*act(idx)./n(idx)).*V(idx,:);
        grad = grad + M.Sv_u'*h;
    end
    U = proj(y - step*grad, P.amax);
    tn = 0.5*(1 + sqrt(1 + 4*t^2));
    y = U + ((t - 1)/tn)*(U - Uprev);
    if sum((U(:) - Uprev(:)).^2) < 1e-12, break; end
    Uprev = U;  t = tn;
end
end

function U = proj(U, amax)
n = sqrt(sum(U.^2, 2));
f = min(1, amax./max(n, 1e-12));
U = U.*f;
end

function p = prev_chaser(mem, t, dt)
% Previous trajectory re-expressed from the current position: p(t) = p_prev(t+dt) - p_prev(dt)
tt = [0, mem.M.t];  pp = [zeros(1,3); mem.path];
p = (interp1(tt, pp, t + dt, 'linear', 'extrap') - interp1(tt, pp, dt, 'linear', 'extrap'))';
end

function u = unit_or(v, fb)
n = norm(v);
if n > 1e-3, u = v/n; else, u = fb; end
end
