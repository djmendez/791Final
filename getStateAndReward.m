function MSN = getStateAndReward(MSN,t,p,pred) 
% states are a tuple combination of the number of neighbors and the
% direction of the predator
    for node = 1:p.maxnodes
        currNode = [MSN.pos(t,node,1) MSN.pos(t,node,2) MSN.pos(t,node,3)];
        number_neighbors = size(MSN.neighbors{node},2);
        
        % Determine direction of predator approach
        % Note here that predator_direction can be 0 if there is no predator
        % or none has been detected yet (i.e. outise radius of detection)
        predator_direction = 0;        
        distance = sqrt((pred.pos(t,1) - currNode(1))^2 + ...
            (pred.pos(t,2) - currNode(2))^2 + ...
            (pred.pos(t,3) - currNode(3))^2);
        MSN.pred_distance(t,node) = distance;
        % if predator within detection radius
        if distance < p.r
            predator_direction = directionPredator(pred.pos(t,:),currNode,p);
        end
        MSN.Report_Pred_distance(MSN.current_run,t,node) = distance;
        
        % add one to count self as a neighbor, makes indexing work later
        MSN.state(t,node,1) = number_neighbors + 1;
        %if Qlearning engaged (i.e. predator detected) 
%        if p.engage_Qlearning
        MSN.state(t,node,2) = predator_direction;
%        end
        % Alternate reward approaches 
        reward1 = 0;
        reward2 = 0;
        
        % For reward approach 1 or combo(3)
        if p.reward == 1 || p.reward == 3
             reward1 = number_neighbors;     
             MSN.reward(t,node) = reward1;
        end

        % For reward approach 2 or combo(3)        
        if (p.reward == 2 || p.reward == 3) && (t > 1)
            if (MSN.pred_distance(t,node) > MSN.pred_distance(t-1,node))
                reward2 = 5;
                MSN.reward(t,node) = reward2;
            end
        end
            
        % For combo reward approach (3)
        % rewrite reward with sum of both approaches
        if p.reward == 3
             MSN.reward(t,node) = reward1 + reward2;
        end
    end
return

% determine which relative direction predator is coming from
function pred_direction = directionPredator(predPos,nodePos,p)   
    delta_x = nodePos(1) - predPos(1);
    delta_y = nodePos(2) - predPos(2);
    delta_z = nodePos(3) - predPos(3);
    
    %calculate angle in degrees
    % Angle from the z-axis approach
    z_theta = atan2d(delta_z,delta_x);
    % Angle from the y-axis approach
    y_theta = atan2d(delta_y,delta_x);
    
    if z_theta > 45 && z_theta < 135
        pred_direction = p.direction.DOWN;
    elseif z_theta < -45 && z_theta > -135
        pred_direction = p.direction.UP;
    elseif y_theta < 45 && y_theta > -45
        pred_direction = p.direction.WEST;
    elseif y_theta < 135 && y_theta > 45
        pred_direction = p.direction.SOUTH;
    elseif y_theta > 135 && y_theta < -135
        pred_direction = p.direction.EAST;
    else
        pred_direction = p.direction.NORTH;
    end
return