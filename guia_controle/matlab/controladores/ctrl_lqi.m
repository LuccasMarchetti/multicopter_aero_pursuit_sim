function [cmd, mem] = ctrl_lqi(st, mem, g, P)
%CTRL_LQI LQR with integral action (lqi_controller.cpp).
%   g: qp, qv, qi, r, ilim, izona
if isempty(mem)
    T = P.Ts;
    A = [1 T 0; 0 1 0; T 0 1];  B = [T^2/2; T; 0];
    mem.K  = dare_ganho(A, B, diag([g.qp g.qv g.qi]), g.r);
    mem.xi = zeros(3,1);
end
ep = -standoff_point(st.p_rel, P.d);
ev = st.v_chaser - st.v_target;
if g.izona <= 0 || norm(ep) < g.izona
    mem.xi = mem.xi + ep*st.dt;
    if norm(mem.xi) > g.ilim, mem.xi = mem.xi*(g.ilim/norm(mem.xi)); end
end
cmd.a = -(mem.K(1)*ep + mem.K(2)*ev + mem.K(3)*mem.xi);
cmd.v = st.v_chaser + cmd.a*st.dt;
end
