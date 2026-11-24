function [cmd, mem] = ctrl_pid(st, mem, g, P)
%CTRL_PID PBVS + PID with feedforward (pbvs_pid_controller.cpp).
%   st: p_rel, v_rel, v_chaser (v_ref), v_target, dt
%   g : kp, ki, kd, ff, ilim, izona
if isempty(mem), mem.I = zeros(3,1); end
d = P.d;
e = standoff_point(st.p_rel, d);
edot = st.v_rel;
r = norm(st.p_rel);
if r > 1e-3
    rh = st.p_rel/r;
    vperp = st.v_rel - (st.v_rel'*rh)*rh;
    edot = edot - (d/r)*vperp;
end
if g.ki > 0 && (g.izona <= 0 || norm(e) < g.izona)
    mem.I = mem.I + e*st.dt;
    imax = g.ilim/g.ki;
    if norm(mem.I) > imax, mem.I = mem.I*(imax/norm(mem.I)); end
end
cmd.v = g.ff*st.v_target + g.kp*e + g.ki*mem.I + g.kd*edot;
cmd.a = (cmd.v - st.v_chaser)/st.dt;
end
