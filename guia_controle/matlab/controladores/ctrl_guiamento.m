function [cmd, mem] = ctrl_guiamento(st, mem, g, P)
%CTRL_GUIAMENTO Hybrid PN/PP guidance with range keeping
%   (hybrid_guidance_controller.cpp).  g: N, K, alpha, klat, kr, krd, vmin
r = st.p_rel;  vr = st.v_rel;  R = norm(r);
if R < 1e-3
    cmd.a = g.krd*vr;  cmd.v = st.v_chaser + cmd.a*st.dt;  return;
end
los = r/R;
rdot = (r'*vr)/R;
Vc = -rdot;
Om = cross(r, vr)/R^2;
a_pn = g.N*Vc*cross(Om, los);
sp = norm(st.v_chaser);
if sp > g.vmin, hd = st.v_chaser/sp; else, hd = los; end
a_pp = g.K*max(sp, g.vmin)*(los - (los'*hd)*hd);
al = min(max(g.alpha, 0), 1);
a_lat = al*a_pn + (1 - al)*a_pp + g.klat*(vr - (vr'*los)*los);
a_lat = a_lat - (a_lat'*los)*los;
a_r = (g.kr*(R - P.d) + g.krd*rdot)*los;
cmd.a = a_lat + a_r;
cmd.v = st.v_chaser + cmd.a*st.dt;
end
