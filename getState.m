function MSN = getState(MSN,t,p,pred) 
% states are a tuple combination of the number of neighbors and the
% direction of the predator
    for node = 1:p.maxnodes
        currNode = [MSN.pos(t,node,1) MSN.pos(t,node,2) MSN.pos(t,node,3)];
        number_neighbors = size(MSN.neighbors{node},2);
        predator_direction = directionPredator(pred.pos,currNode,p);
        MSN.state(t,currNode,1) = number_neighbors;
        %if Qlearning engaged (i.e. predator detected) 
        if p.engage_Qlearning
            MSN.state(t,currNode,2) = predator_direction;  
        end
    end
return

% If a predator is within detection range, determine which relative
% direction it's coming from
function pred_direction = directionPredator(predPos,nodePos,p)
    distance = norm(predPos - nodePos,2);
    if distance < p.r
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
    else
        pred_direction = 0;
    end
return