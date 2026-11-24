function [cmd, mem] = ctrl_smc(st, mem, g, P)
%CTRL_SMC Modos deslizantes adaptativo (sliding_mode_controller.cpp).
%   g: lambda, ks, k0, kmin, kmax, gamma, sigma, phi
if isempty(mem), mem.k = g.k0; end
ep = -standoff_point(st.p_rel, P.d);
ev = st.v_chaser - st.v_target;
s  = ev + g.lambda*ep;
ns = norm(s);
phi = max(1e-3, g.phi);
if ns > phi
    mem.k = mem.k + g.gamma*(ns - g.sigma*mem.k)*st.dt;
    mem.k = min(max(mem.k, g.kmin), g.kmax);
end
sat = s/max(ns, phi);
cmd.a = -g.lambda*ev - g.ks*s - mem.k*sat;
cmd.v = st.v_chaser + cmd.a*st.dt;
end
