function MSN = getStateAndReward(MSN,t,p,pred) 
% states are a tuple combination of the number of neighbors and the
% direction of the predator
    for node = 1:p.maxnodes
        currNode = [MSN.pos(t,node,1) MSN.pos(t,node,2) MSN.pos(t,node,3)];
        number_neighbors = size(MSN.neighbors{node},2);
        % Note here that predator_direction can be 0 if there is no predator
        % or none has been detected yet (i.e. outise radius of detection)
        predator_direction = directionPredator(pred.pos(t,:),currNode,p);
        % add one to count self as a nieghbor, makes indexing work later
        %fprintf('number of neighbors is %d', number_neighbors);
        MSN.state(t,node,1) = number_neighbors + 1;
        %if Qlearning engaged (i.e. predator detected) 
        if p.engage_Qlearning
            MSN.state(t,node,2) = predator_direction;  
        end
        % Alternate reward approaches 
        % Approach 1: Note 20 is arbitrary weight
        % going to try to keep this one simple with number of neighbors
        reward1 = 0;
        reward2 = 0;
        
        if p.reward == 1 || p.reward == 3
             reward1 = number_neighbors;     
             MSN.reward(t,node) = reward1;
        end
        % approach #2
        
        if p.reward > 1
            if (norm(pred.pos(t,:)-reshape(MSN.pos(t,node,:),1,p.dimensions)) > ...
                    norm(pred.pos(t-1,:)-reshape(MSN.pos(t-1,node,:),1,p.dimensions)))
                reward2 = 5;
                MSN.reward(t,node) = reward2;
            end
        end
            
        if p.reward == 3
             MSN.reward(t_node) = reward1 + reward2;
        end
    end
return

% If a predator is within detection range, determine which relative
% direction it's coming from
function pred_direction = directionPredator(predPos,nodePos,p)
    distance = norm(predPos - nodePos,2);
    % if predator within detection radius
    if distance < p.r
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
    % if not within detection
    else
        pred_direction = 0;
    end
return