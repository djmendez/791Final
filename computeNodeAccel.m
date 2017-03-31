% Algorithm implementation
% this function computes the accelration of nodes based on the algorithms
% NOTE: most of the algorithm constants were set at the beginning and
% are passed here as part of the params (p) stracture (to reduce
% computation)
function u = computeNodeAccel(node,neighbors,MSN,p,t)
    prevt = t - 1;
    u = [0 0 0];

    qi = [MSN.pos(prevt,node,1) MSN.pos(prevt,node,2) MSN.pos(prevt,node,3)];
    pi = [MSN.vel(prevt,node,1) MSN.vel(prevt,node,2) MSN.vel(prevt,node,3)];

    % for all runs compute algorithm 1
    % all are based on algorithm 1 so do for all cases
    for neighbor = neighbors
        % get position current neighbor
        qj = [MSN.pos(prevt,neighbor,1) MSN.pos(prevt,neighbor,2) MSN.pos(prevt,neighbor,3)];
        pj = [MSN.vel(prevt,neighbor,1) MSN.vel(prevt,neighbor,2) MSN.vel(prevt,neighbor,3)];

        u = u + algorithm1(qi,qj,pi,pj,p);
    end

    % for algorithms 2 and 3, add algorithm 2
    if p.algorithm > 1          %captures cases 2 and 3
        u = u + algorithm2(qi,pi,p);
    end

    % finally for algorithm 3, add obstacle repulsion
    if p.algorithm == 3        
        u = u + algorithm3(qi,pi,p);
    end
return

function u = algorithm1(qi,qj,pi,pj,p)
    % compute position difference vector
    q_ji = qj - qi;
    p_ji = pj - pi;
    
    % compute sigmaNorm of relative positions
    sn_ji = sigmaNorm(qj,qi,p.eps);
    
    %Compute phi_alpha_ji
    phi_alpha_ji = phiAlpha(sn_ji,p);
    
    %compute nij vector
    %distance_ji = sqrt((qj(1) - qi(1))^2 + (qj(2) - qi(2))^2) + (qj(3) - qi(3))^2;
    distance_ji = norm(qj - qi,2);
    nij = q_ji / (sqrt (1 + p.eps * distance_ji^2));
    
    %compute aij
    aij_q = rho(sn_ji/p.r_alpha,p.h);
        
    % algorithm1
    u =  (p.c1 * phi_alpha_ji * nij) + (p.c2 * aij_q * p_ji);
return

function u = algorithm2(qi,pi,p)
    if (p.target_pmt == 0)
        u = -1 * (p.c1mt * (qi - p.target_qmt));
    else
        u = -1 * (p.c1mt * (qi - p.target_qmt) + p.c2mt * (pi - p.target_pmt));
    end
return

function u = algorithm3(qi,pi,p)

    u = [0 0 0];
    I = [1 0 0; 0 1 0; 0 0 1];

    for obstacle = 1:p.obstacles.number;
        obstacle_center = p.obstacles.center(obstacle,:);
        obstacle_radius = p.obstacles.radii(obstacle);
        %distance = sqrt((qi(1) - obstacle_center(1))^2 + (qi(2) - obstacle_center(2))^2);
        distance = norm(qi - obstacle_center,2);
        if distance < obstacle_radius + p.r
            mu = obstacle_radius / distance;
            ak = (qi - obstacle_center) / obstacle_radius;
            P = I - (ak*transpose(ak));

            q_hat_ik = (mu * qi) + ((1 - mu) * obstacle_center);   
            p_hat_ik = transpose(mu * P * transpose(pi)); 

            q_ki = q_hat_ik - qi;
            p_ki = p_hat_ik - pi;
            
            % compute sigmaNorm of relative positions
            sn_ki = sigmaNorm(q_hat_ik,qi,p.eps);
            
            %Compute phi_alpha_ji
            phi_beta_ki = phiBeta(sn_ki,p);

            %compute nik vector
            %distance_ik = sqrt((q_hat_ik(1) - qi(1))^2 + (q_hat_ik(2) - qi(2))^2);
            distance_ik = norm(q_hat_ik - qi,2);
            
            nik = q_ki / (sqrt (1 + p.eps * distance_ik^2));
        
            %compute aij
            bik_q = rho(sn_ki/p.d_beta,p.h_beta);

            % basically algorithm1 with beta functions
            u =  (p.c1_beta * phi_beta_ki * nik) + (p.c2_beta * bik_q * p_ki);
        end
    end
return