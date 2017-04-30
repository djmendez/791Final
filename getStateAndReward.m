function MSN = getStateAndReward(MSN,t,p,pred) 
% states are a tuple combination of the number of neighbors and the
% direction of the predator
    for node = 1:p.maxnodes
        currNode = [MSN.pos(t,node,1) MSN.pos(t,node,2) MSN.pos(t,node,3)];
        number_neighbors = size(MSN.neighbors{node},2);
        
        % Determine direction of predator approach
   
        distance_to_pred = sqrt((pred.pos(t,1) - currNode(1))^2 + ...
            (pred.pos(t,2) - currNode(2))^2 + ...
            (pred.pos(t,3) - currNode(3))^2);
        MSN.pred_distance(t,node) = distance_to_pred;

        %%% decide whether can be not seen - then not call
        if distance_to_pred < pred.prey_visibility
            predator_direction = directionPredator(pred.pos(t,:),currNode,p);
        else
            predator_direction = p.direction.NONE;
        end
     
        MSN.Report_Pred_distance(MSN.current_run,t,node) = distance_to_pred;
        
        MSN.state(t,node) = predator_direction;

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
            delta_distance = MSN.pred_distance(t,node) - MSN.pred_distance(t-1,node);
            if delta_distance > 0
                % pass 2: make reward max -20 (min -50 delta distance * -1)
                reward2 = min(10,delta_distance);
                %reward2 = 10;
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
    delta_x = predPos(1) - nodePos(1);
    delta_y = predPos(2) - nodePos(2);
    delta_z = predPos(3) - nodePos(3);
    
    %calculate angle in degrees
    % Angle from the z-axis approach
    z_theta = atan2d(delta_z,delta_x) + 360*(delta_z<0);
    % Angle from the y-axis approach
    y_theta = atan2d(delta_y,delta_x) + 360*(delta_y<0);
    
    if z_theta > 45 && z_theta < 135
        pred_direction = p.direction.UP;
    elseif z_theta > 225 && z_theta < 315 
        pred_direction = p.direction.DOWN;
        
    elseif y_theta < 45 || y_theta > 315
        pred_direction = p.direction.EAST;
    elseif y_theta > 135 && y_theta < 225
        pred_direction = p.direction.WEST;
        
    elseif y_theta > 45 && y_theta < 135
        pred_direction = p.direction.NORTH;        
    elseif y_theta < 315 && y_theta > 225
        pred_direction = p.direction.SOUTH;
      
    % should never get here
    else
        pred_direction = p.direction.NONE;
    end
return