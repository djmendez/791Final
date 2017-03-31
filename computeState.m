function state = computeState(node,p) %target,

% distance = sqrt((target(1) - node(1))^2 + (target(2) - node(2))^2);
% if distance > p.max_distance
%     state = 10;
% else
%     state = ceil(distance/p.circle_radius);
% end

state = min(ceil(node/p.gridsize),p.num_states);
return