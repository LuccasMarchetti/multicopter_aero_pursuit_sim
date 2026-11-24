function cmd = aplicar_limites(cmd, v_prev, vmax, amax, dt)
%APLICAR_LIMITES Same as control::applyLimits: limits |a| (rate of v_ref) and |v|.
a = clamp_norm((cmd.v - v_prev)/dt, amax);
cmd.v = clamp_norm(v_prev + a*dt, vmax);
cmd.a = a;
end

function v = clamp_norm(v, m)
n = norm(v);
if n > m && n > 0
    v = v*(m/n);
end
end
