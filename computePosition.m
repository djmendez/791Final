% Main function Project1A -- computes the next position, velocity, com and
% connectivity of the MSN
function [MSN,Q] = computePosition(MSN,Q,pred,t,p)
prevt = t - 1;

% Compute target position for nodes
% will be overriden on a per node basis if a predator is detected
if p.algorithm > 1 && p.target_movement > 0
    if p.target_movement == 1 % line
        p.target_qmt(:,:) =  p.target_qmt(:,:) + (p.target_pmt(:,:) * p.dt);
    elseif p.target_movement == 2 %sine
        p.target_qmt(:,1) =  p.target_origin(:,1) + (.25 * t);
        p.target_qmt(:,2) =  p.target_origin(:,2) - (100 * sin(.005*t));
    elseif p.target_movement == 3 %circle
        p.target_qmt(:,1) =  p.target_origin(:,1) - (p.circle_radius * cos(.005*t));
        p.target_qmt(:,2) =  p.target_origin(:,2) + (p.circle_radius * sin(.005*t));
        % amplitude * sin (frequency * t)
        p.target_qmt(:,3) =  p.target_origin(:,3) - (200 * sin(.01*t));
    end
    MSN.target_qmt(t,:) = p.target_qmt;
end

% for all robots/nodes
for node = 1:p.maxnodes
    currNode = [MSN.pos(prevt,node,1) MSN.pos(prevt,node,2) MSN.pos(prevt,node,3)];

    %get neighbors
    [MSN.neighbors{node}] = computeNeighbors(node,currNode,prevt,MSN,p);
    
    % if predator is visible then do qlearning 
    % once the first bird sees the predator Qlearning will remain engaged
    % Qlearning will be used to select action and then
    % action determines target, target determines
    if p.engage_Qlearning == false
        if isPredatorDetected(currNode,pred.pos(prevt,:),pred.prey_visibility) ...
            && pred.active
                %p.engage_Qlearning = true;
                %set targets top be static
                p.target_pmt = 0;                
        end
    end
    
    if p.engage_Qlearning
        [MSN,Q] = Qlearning(node,MSN,Q,t,p);
        %add obstacle to account for predator
        p.obstacles.center(end+1,:) = pred.pos(prevt,:);
        p.obstacles.radii(end+1) = pred.prey_visibility;
        %determine target_qmt based on action;
        action = MSN.action(t,node);

        % fly towards safe space
        if action < 5
            p.target_qmt = p.safespaces(action,:);
            MSN.accel(t,node,:) = ComputeNodeAccel(node,MSN.neighbors{node},MSN,p,t);
            % q(k) = qi(k-1) + Delta_t*p (k) + ((Delta_t)^2/2) *ui (k);
            MSN.pos(t,node,:) =  MSN.pos(prevt,node,:) + (MSN.vel(prevt,node,:) * p.dt) + (MSN.accel(t,node,:) * .5 * p.dt^2);    
            % vel = pi(k) = (qi(k) - qi(k - 1)) / delta_t
            MSN.vel(t,node,:) = (MSN.pos(t,node,:) - MSN.pos(prevt,node,:)) / p.dt;
        else
            % target reached dont move
            % position doesnt change
            MSN.vel(t,node,:) = [0 0];
            MSN.pos(t,node,:) = MSN.pos(prevt,node,:);
            MSN.robots_at_target = MSN.robots_at_target + 1;
        end
        
        %cleanup by putting everything back to they way it was
        p.obstacles.center(end,:) = [];
        p.obstacles.radii(end)=[];
    else
        MSN.accel(t,node,:) = computeNodeAccel(node,MSN.neighbors{node},MSN,p,t);
        % q(k) = qi(k-1) + Delta_t*p (k) + ((Delta_t)^2/2) *ui (k);
        MSN.pos(t,node,:) =  MSN.pos(prevt,node,:) + (MSN.vel(prevt,node,:) * p.dt) + (MSN.accel(t,node,:) * .5 * p.dt^2);
        % vel = pi(k) = (qi(k) - qi(k - 1)) / delta_t
        MSN.vel(t,node,:) = (MSN.pos(t,node,:) - MSN.pos(prevt,node,:)) / p.dt;
    end
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
return

function predator = isPredatorDetected(node,predpos,pred_radius)
predator = false;
distance = sqrt((node(1) - predpos(1))^2 + (node(2) - predpos(2))^2);
if distance < pred_radius
    predator = true;
end