function K = mpc_ganho_equivalente(P, g)
%MPC_GANHO_EQUIVALENTE Linear gain u0 = -K x0 of the unconstrained MPC with frozen LOS.
%   Zero reference (standoff already reached over the horizon), error state x0 = [e_p; e_v].
if nargin < 2, g = P.mpc; end
M = mpc_matrizes(g, P.Ts);
F = 2*M.Su'*M.Qb*M.Sx;        % g = F*x0
U = -(M.H \ F);               % N x 2
K = -U(1, :);
end
