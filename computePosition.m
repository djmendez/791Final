% Main function Project1A -- computes the next position, velocity, com and
% connectivity of the MSN
function [MSN,Q,Pred] = computePosition(MSN,Q,Pred,t,p)
    prevt = t - 1;

    % Compute Target Position
    if p.algorithm > 1 && p.target_movement > 0
        p.target_qmt(:,1) =  p.target_origin(:,1) - (p.circle_radius * cos(.005*t));
        p.target_qmt(:,2) =  p.target_origin(:,2) + (p.circle_radius * sin(.005*t));
        % amplitude * sin (frequency * t)
        p.target_qmt(:,3) =  p.target_origin(:,3) - (200 * sin(.01*t));
        MSN.target_qmt(t,:) = p.target_qmt;
    end

    % Compute Predator Position            
    Pred = computePredator(Pred,MSN,t,p);
    
    % Compute Node position
    % 1) determine neighbors
    % for all nodes, get neighbors and check if a predator is around
    for node = 1:p.maxnodes
        currNode = [MSN.pos(prevt,node,1) MSN.pos(prevt,node,2) MSN.pos(prevt,node,3)];
        %get neighbors
        [MSN.neighbors{node}] = computeNeighbors(node,currNode,prevt,MSN,p);

        % if predator is visible then do qlearning
        % once the first bird sees the predator Qlearning will remain engaged
        % Qlearning will be used to select action and then
        % action determines target, target determines movement
        if p.engage_Qlearning == false && Pred.active
            if isPredatorDetected(currNode,Pred.pos(prevt,:),Pred.prey_visibility)
                % WAITING FOR QLEARNING TO BE CONNECTED - Currently never
                % turn on
                % p.engage_Qlearning = true;
            end
        end
    end

    % If a predator has been detected and Qlearning engaged, then use it to
    % move
    if p.engage_Qlearning 
        %DISCUSS: DOn't think we should make the predator an obstacle --
        %let them figure it out
%         %add obstacle to account for predator
%         p.obstacles.center(p.obstacles.number+1,:) = pred.pos(prevt,:);
%         p.obstacles.radii(p.obstacles.number+1) = pred.prey_visibility;

        %call Qlearning to determine action
        [MSN] = Qlearning(MSN,Q,t,p);
        % perform action
        [MSN] = doMovement(MSN,t,p);
        % Compute new State based on taken action
        [MSN] = getState(MSN,t,p,Pred);
        %Update Q-values
        [MSN,Q] = QUpdate(MSN,Q,t,p);
    % else continue moving using just the target
    else
        MSN = doMovement(MSN,t,p);
    end
    
% End Function
end

%DO ACTION
function MSN = doMovement(MSN,t,p)
    prevt = t - 1;

    for node = 1:p.maxnodes
        % if Qlearning, then target gets set based on action
        if p.engage_Qlearning
            %get action
            %set target based on action
        end
        
        %Actually compute movement
        %ComputeNodeAccel will compute accurately 
        % In the event of Qlearning/predator, target has been set
        % to direction and predator as obstacle
        % If no Qlearning/predator, target is the moving target and no
        % predator
        MSN.accel(t,node,:) = computeNodeAccel(node,MSN.neighbors{node},MSN,p,t);
        %q(k) = qi(k-1) + Delta_t*p (k) + ((Delta_t)^2/2) *ui (k);
        MSN.pos(t,node,:) =  MSN.pos(prevt,node,:) + (MSN.vel(prevt,node,:) * p.dt) + (MSN.accel(t,node,:) * .5 * p.dt^2);
        %vel = pi(k) = (qi(k) - qi(k - 1)) / delta_t
        MSN.vel(t,node,:) = (MSN.pos(t,node,:) - MSN.pos(prevt,node,:)) / p.dt;
    end

    %make sure they dont swim through glass
    % x = MSN.pos(t,:,:);
    % x(x>p.maxgrid)=p.maxgrid-1;
    % x(x<0)=1;
    % MSN.pos(t,:,:) = x;

    % Compute Center of mass
    MSN.center_mass(t,1) = mean(MSN.pos(t,:,1));
    MSN.center_mass(t,2) = mean(MSN.pos(t,:,2));
    MSN.center_mass(t,3) = mean(MSN.pos(t,:,3));
end

function predator = isPredatorDetected(node,predpos,pred_radius)
    predator = false;
    distance = sqrt((node(1) - predpos(1))^2 + (node(2) - predpos(2))^2);
    if distance < pred_radius
        predator = true;
    end
end


%%%% OLD CODE JUST IN CASE
%     % fly towards safe space
%     if action < 5
%         p.target_qmt = p.safespaces(action,:);
%         MSN.accel(t,node,:) = ComputeNodeAccel(node,MSN.neighbors{node},MSN,p,t);
%         % q(k) = qi(k-1) + Delta_t*p (k) + ((Delta_t)^2/2) *ui (k);
%         MSN.pos(t,node,:) =  MSN.pos(prevt,node,:) + (MSN.vel(prevt,node,:) * p.dt) + (MSN.accel(t,node,:) * .5 * p.dt^2);
%         % vel = pi(k) = (qi(k) - qi(k - 1)) / delta_t
%         MSN.vel(t,node,:) = (MSN.pos(t,node,:) - MSN.pos(prevt,node,:)) / p.dt;
%     else
%         % target reached dont move
%         % position doesnt change
%         MSN.vel(t,node,:) = [0 0];
%         MSN.pos(t,node,:) = MSN.pos(prevt,node,:);
%         MSN.robots_at_target = MSN.robots_at_target + 1;
%     end
%     
%     %cleanup by putting everything back to they way it was
%     p.obstacles.center(end,:) = [];
%     p.obstacles.radii(end)=[];
% else
