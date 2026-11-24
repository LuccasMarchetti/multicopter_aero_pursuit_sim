function M = mpc_matrizes(g, Ts)
%MPC_MATRIZES Condensed per-axis prediction of the LTV MPC (mpc_controller.cpp).
%   Grid: dt_k = Ts*growth^k, k = 0..N-1.  State per axis x = [p; v].
%   X = Sx*x0 + Su*U  (X: 2N x 1 empilhado [p1 v1 p2 v2 ...], U: N x 1)
%   Custo: (X-Xref)'Qb(X-Xref) + r U'U  ->  H = 2(Su'Qb Su + r I)
N = g.N;
dts = Ts*g.growth.^(0:N-1);
Sx = zeros(2*N, 2);  Su = zeros(2*N, N);  phi = eye(2);
for k = 1:N
    A = [1 dts(k); 0 1];  B = [dts(k)^2/2; dts(k)];
    phi = A*phi;
    Sx(2*k-1:2*k, :) = phi;
    for j = 1:k-1
        Su(2*k-1:2*k, j) = A*Su(2*k-3:2*k-2, j);
    end
    Su(2*k-1:2*k, k) = B;
end
q = repmat([g.qp; g.qv], N, 1);
q(end-1:end) = q(end-1:end)*g.wN;
Qb = diag(q);
M.N = N;  M.dts = dts;  M.t = cumsum(dts);
M.Sx = Sx;  M.Su = Su;  M.Qb = Qb;
M.H = 2*(Su'*Qb*Su + g.r*eye(N));
M.Sv_x = Sx(2:2:end, :);  M.Sv_u = Su(2:2:end, :);   % velocity rows
M.L = max(eig(M.H));
if isfield(g,'vel_con') && g.vel_con
    M.L = M.L + g.rho*max(eig(M.Sv_u'*M.Sv_u));
end
end
