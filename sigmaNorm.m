% programing function to compute a sigma norm 

% euclidean pairwise distance: sqrt((x2 - x1)^2 + (y2 - y1)^2)
% sigma-norm is 1/eps * [sqrt(1 + (eps * (pdist^2))) -1]

function sn = sigmaNorm(qi,qj,eps)
    distance = sqrt((qi(1) - qj(1))^2 + (qi(2) - qj(2))^2 + (qi(3) - qj(3))^2);
    sn = (1 / eps) * (sqrt(1 + (eps * distance^2)) - 1);
end

