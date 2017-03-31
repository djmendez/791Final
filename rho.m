function bump = rho(z,h)
    if (z < 0 || z > 1)
        bump = 0;
    elseif (z < h)
        bump = 1;
    else % h <= z < 1
        bump = .5 * (1 + cos (pi * ((z - h)/(1 - h))));
end
