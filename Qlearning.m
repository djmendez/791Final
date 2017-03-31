% Qlearning fro Project 1A
%Main Qlearning function
function [MSN,Q] = Qlearning(robot,MSN,Q,t,p)
prevt = t - 1;

curr_pos = [MSN.pos(prevt,robot,1) MSN.pos(prevt,robot,2)];

curr_action = max(1,MSN.action(prevt,robot,1));
curr_state = computeState(curr_pos,p);

%determine whether we have reached target 
target_state = isTargetState(curr_pos,p);
%and if so stop moving
if target_state
    next_action = 5;
    next_target = curr_action;
    next_pos = curr_pos;
    next_state = curr_state;
%otherwise Qlearn
else
    %generate random number 0 - 1 to determine exploit vs explore
    max_Q = 0;
    exploit = randi(100)*.01;
    %exploit
    if exploit >= p.epsilon
        %find action that maximizes Q-value
        for a = 1:size(p.safespaces)
            pot_target = p.safespaces(a,:);
            pot_pos = curr_pos + computeMovement(curr_pos,pot_target,p.vel,p.dt);
            pot_state = computeState(pot_pos,p);
            pot_Q = Q(robot,pot_state(1),pot_state(2),a);
            %save max action
            if pot_Q > max_Q
                max_Q = pot_Q;
                next_action = a;
                next_target = pot_target;
                next_pos = pot_pos;
                next_state = pot_state;
            end
        end
    end
    % If explore OR if all Q-values are 0, take random action
    if exploit < p.epsilon || max_Q == 0
        next_action = randi(4);
        next_target = p.safespaces(next_action,:);
        next_pos = curr_pos + computeMovement(curr_pos,next_target,p.vel,p.dt);
        next_state = computeState(next_pos,p);
    end
end

% Compute reward - 10 * neighbors, and 500 more if reach safe target
curr_reward = 10 * cellfun('length',MSN.neighbors(robot));
if isTargetState(next_pos,p);
    curr_reward = curr_reward + 500;
end

MSN.action(t,robot) = next_action;
MSN.state(t,robot,:) = next_state;
MSN.reward(t,robot) = curr_reward;

% for debugging
%     str = sprintf('Processing %d at time %d: curr a %d +s %d %d rew %d next a %d +s: %d %d', ...
%         robot,t,curr_action, curr_state(1),curr_state(2),curr_reward,next_action,next_state(1),next_state(2));
%     disp(str);

%Compute next Qvalue based on algorithm
Q(robot,curr_state(1),curr_state(2),curr_action) = ...
    Q(robot,curr_state(1),curr_state(2),curr_action) ...
    + p.learning_rate * (curr_reward + p.discount_factor * ...
    Q(robot,next_state(1),next_state(2),next_action) - ...
    Q(robot,curr_state(1),curr_state(2),curr_action));

MSN.pos(t,robot,:) = next_pos;

% Cooperative Q-learning
% Note that I am NOT forcing same action, but rather seeing impact of a
% changed Q-table
if p.q_learning_algorithm == 2
    for robot = 1:p.robots
        curr_state = MSN.state(prevt,robot,:);
        curr_action = MSN.action(prevt,robot,1);
        next_action = MSN.action(t,robot);
        next_state = MSN.state(t,robot,:);
        neighbors = MSN.neighbors{robot};
        
        Q(robot,curr_state(1),curr_state(2),curr_action) = ...
            (p.cl_weight * Q(robot,curr_state(1),curr_state(2),curr_action)) ...
            + (1 - p.cl_weight) * ...
            learnFromNeighbors(robot,next_state,next_action,neighbors,Q);
    end
end
return

% learn from neighbors for collaborative Qlearning
function learn_from_neighbors = learnFromNeighbors(robot,sj,aj,neighbors,Q)

learn_from_neighbors = 0;
for neighbor = neighbors
    learn_from_neighbors = learn_from_neighbors + Q(neighbor,sj(1),sj(2),aj);
end



