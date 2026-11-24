function [K, P] = dare_ganho(A, B, Q, R)
%DARE_GANHO Discrete LQR gain by Riccati iteration (same as the C++).
%   [K, P] = dare_ganho(A, B, Q, R)  ->  u = -K x
%   Needs no toolbox. With the Control System Toolbox: K = dlqr(A,B,Q,R).
P = Q;
for it = 1:20000
    S = R + B'*P*B;
    K = S \ (B'*P*A);
    Pn = Q + A'*P*(A - B*K);
    Pn = 0.5*(Pn + Pn');
    if max(abs(Pn(:) - P(:))) < 1e-10
        P = Pn;
        break;
    end
    P = Pn;
end
K = (R + B'*P*B) \ (B'*P*A);
end
