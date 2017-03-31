function phi_beta_z = phiBeta(z,p)
    phi_beta_z = rho(z/p.r_beta,p.h_beta) * (sigma1(z - p.d_beta) -1);
end