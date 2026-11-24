function [cmd, mem] = ctrl_lqr(st, mem, g, P)
%CTRL_LQR Discrete LQR on the error double integrator (lqr_controller.cpp).
%   g: qp, qv, r   (K computed on the first call)
if isempty(mem)
    A = [1 P.Ts; 0 1];  B = [P.Ts^2/2; P.Ts];
    mem.K = dare_ganho(A, B, diag([g.qp g.qv]), g.r);
end
ep = -standoff_point(st.p_rel, P.d);
ev = st.v_chaser - st.v_target;
cmd.a = -(mem.K(1)*ep + mem.K(2)*ev);
cmd.v = st.v_chaser + cmd.a*st.dt;
end
