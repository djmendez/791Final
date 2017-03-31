function next_pos = computeMovement(pos,target,vel,dt)

target_delta = [target(1) - pos(1) target(2) - pos(2)];
target_distance = sqrt((target(1) - pos(1))^2 + (target(2) - pos(2))^2);
target_angle = atan2(target_delta(2),target_delta(1));

% scale velocity as we get closer
% if target_distance < vel
%      scaled_vel = .04 * target_distance^2;
% else
    scaled_vel = vel;
% end

next_pos(1) =  scaled_vel * dt * cos(target_angle) * abs(target_delta(1));
next_pos(2) =  scaled_vel * dt * sin(target_angle) * abs(target_delta(2));
return