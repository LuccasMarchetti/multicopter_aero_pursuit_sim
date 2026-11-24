function p = standoff_point(p_rel, d)
%STANDOFF_POINT Pursuit point relative to the pursuer: r - d*r/|r|.
r = norm(p_rel);
if r < 1e-3
    p = zeros(3,1);
else
    p = p_rel - (d/r)*p_rel;
end
end
