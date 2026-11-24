function M = modelo_px4(P, com_ff)
%MODELO_PX4 Linear models of the PX4 inner loop and of the outer-loop plants.
%   M = modelo_px4(P, com_ff)
%     com_ff = true  -> velocity setpoint with acceleration feedforward (default)
%   Campos:
%     M.Tv   : v/v_sp (continuous)                        eq. (4.x) of the guide
%     M.Pa   : position / commanded acceleration  = Tv/s^2  (acceleration controllers)
%     M.Pv   : position / commanded velocity  = Tv/s    (PBVS PID)
%     M.Pa_z, M.Pv_z : discrete (ZOH, Ts) with measurement delay in samples
%   Requer Control System Toolbox.
if nargin < 2, com_ff = true; end
s  = tf('s');
wa = P.px4.wa;  Kp = P.px4.Kp_v;  Ki = P.px4.Ki_v;  Kd = P.px4.Kd_v;
kappa = double(com_ff);

num = wa*(kappa*s^2 + Kp*s + Ki);
den = s^2*(s + wa) + wa*(Kd*s^2 + Kp*s + Ki);
M.Tv = minreal(num/den);

M.Pa = M.Tv/s^2;
M.Pv = M.Tv/s;
M.Pa.InputDelay = P.tau;
M.Pv.InputDelay = P.tau;

% Discretization: ZOH of the plant + integer delay in samples
n = round(P.tau/P.Ts);
M.n_atraso = n;
M.Pa_z = c2d(M.Tv/s^2, P.Ts, 'zoh');  M.Pa_z.InputDelay = n;
M.Pv_z = c2d(M.Tv/s,   P.Ts, 'zoh');  M.Pv_z.InputDelay = n;
end
