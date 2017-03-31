function phi_alpha_z = phiAlpha(z,p)
    phi_alpha_z = rho(z/p.r_alpha,p.h) * phi(z - p.d_alpha);
end

function phi_z = phi(z)
    % set per paper page 413 
    %parameters 0 < a <= b, c = |a -b| / sqrt(4ab) to guarantee phi(0) = 0
    a = 5;
    b = 5;
    c = 0;

    % dont waste the calculation if it will turn out to be 0
    if (a ~= b)
        c = abs(a - b)/sqrt(4*a*b);
    end

    phi_z = .5 * (((a+b) * sigma1(z+c)) + (a - b));
end



