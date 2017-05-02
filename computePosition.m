% Main function Project1A -- computes the next position, velocity, com and
% connectivity of the MSN
function [MSN,Q,Pred] = computePosition(MSN,Q,Pred,t,p)
    prevt = t - 1;

%     % Compute Target Position
%     if p.algorithm > 1 && p.target_movement > 0
%         p.target_qmt(:,1) =  p.target_origin(:,1) - (p.circle_radius * cos(.005*t));
%         p.target_qmt(:,2) =  p.target_origin(:,2) + (p.circle_radius * sin(.005*t));
%         % amplitude * sin (frequency * t)
%         p.target_qmt(:,3) =  p.target_origin(:,3) - (200 * sin(.01*t));
%         %MSN.target_qmt(t,:) = p.target_qmt;
%     end

    % Compute Predator Position            
    Pred = computePredator(Pred,MSN,t,p);
    
    % Compute Node position
    % 1) determine neighbors
    % for all nodes, get neighbors and check if a predator is around
    for node = 1:p.maxnodes
        currNode = [MSN.pos(prevt,node,1) MSN.pos(prevt,node,2) MSN.pos(prevt,node,3)];
        %get neighbors
        [MSN.neighbors{node}] = computeNeighbors(node,currNode,prevt,MSN,p);
    end

    %call Qlearning to determine action
    % expected return is MSN.action(t,all nodes)
    % will determine position at current time t based on
    % prev position at time t-1
    % determine action for this time
    [MSN] = GetAction(MSN,Q,t,p);
    % perform action and compute position for time t (based on t-1)
    [MSN] = doMovement(MSN,t,p);
    % Compute new State based on taken action and new position
    [MSN] = getStateAndReward(MSN,t,p,Pred);
    %Update Q-values - ONLY IF IN TRAINING
    %       if p.training
    [Q] = QUpdate(MSN,Q,t,p);
    %        end
    
% End Function
end

%DO ACTION
function MSN = doMovement(MSN,t,p)
    prevt = t - 1;

    for node = 1:p.maxnodes

        [p.target_qmt,p.target_pmt] = computeTarget(MSN,node,t,p);
     
        %Actually compute movement
        % In the event of Qlearning/predator, target has been set to direction
        MSN.accel(t,node,:) = computeNodeAccel(node,MSN.neighbors{node},MSN,p,t);
        %q(k) = qi(k-1) + Delta_t*p (k) + ((Delta_t)^2/2) *ui (k);
        MSN.pos(t,node,:) =  MSN.pos(prevt,node,:) + (MSN.vel(prevt,node,:) * p.dt) + (MSN.accel(t,node,:) * .5 * p.dt^2);
        %vel = pi(k) = (qi(k) - qi(k - 1)) / delta_t
        
        %make sure they havent swum through glass
        temp_x = MSN.pos(t,node,:);
        temp_x(temp_x>p.maxgrid) = p.maxgrid - 25;
        temp_x(temp_x<0) = 25;
        MSN.pos(t,node,:) = temp_x;
        
        % compute velocity given new position
        MSN.vel(t,node,:) = (MSN.pos(t,node,:) - MSN.pos(prevt,node,:)) / p.dt;
    end
end

function [target,target_velocity] = computeTarget(MSN,node,t,p)
    action = MSN.action(t-1,node);
    target_velocity = [0 0 0];
    target = reshape(MSN.pos(t-1,node,:),1,p.dimensions);
    switch action
        case p.direction.NORTH
            target(2) = target(2) + p.target_distance;
            target_velocity(2) = p.target_velocity;
        case p.direction.SOUTH
            target(2) = target(2) - p.target_distance;
            target_velocity(2) = p.target_velocity;  
            
        case p.direction.EAST
            target(1) = target(1) + p.target_distance;
            target_velocity(1) = p.target_velocity;       
        case p.direction.WEST
            target(1) = target(1) - p.target_distance;
            target_velocity(1) = p.target_velocity;      
            
        case p.direction.UP
            target(3) = target(3) + p.target_distance;
            target_velocity(3) = p.target_velocity;          
        case p.direction.DOWN
            target(3) = target(3) - p.target_distance;
            target_velocity(3) = p.target_velocity;            
    end
end

% function predator = isPredatorDetected(node,predpos,pred_radius)
%     predator = false;
%     distance = sqrt((node(1) - predpos(1))^2 + (node(2) - predpos(2))^2);
%     if distance < pred_radius
%         predator = true;
%     end
% end
