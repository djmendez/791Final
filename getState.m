function MSN = getState(MSN,t,p,pred) 
    for node = 1:p.maxnodes
        currNode = [MSN.pos(t,node,1) MSN.pos(t,node,2) MSN.pos(t,node,3)];
        number_neighbors = size(MSN.neighbors{node},2);
        predator_direction = directionPredator(pred.pos,currNode,p);
        MSN.state(t,currNode,1) = number_neighbors;
        MSN.state(t,currNode,2) = predator_direction;  
    end
return

% NOT DONE YET
function pred_direction = directionPredator (predPos,nodePos,p)
    u = predPos - nodePos;
    x_unit = [1 0 0];
    y_unit = [0 1 0];
    %calculate angle in degrees
    z_theta = atan2d(cross(u,x_unit),dot(u,x_unit));
    y_theta = atan2d(cross(u,y_unit),dot(u,y_unit));
    
    if z_theta > 45 && z_theta < 135
        pred_direction = p.direction.UP;
    elseif z_theta < -45 && z_theta > -135
        pred_direction = p.direction.DOWN;
    elseif y_theta < 45 && y_theta > -45
        pred_direction = p.direction.EAST;
    elseif y_theta < 135 && y_theta > 45
        pred_direction = p.direction.NORTH;
    elseif y_theta > 135 && y_theta < -135
        pred_direction = p.direction.WEST;
    else
        pred_direction = p.direction.SOUTH;
    end
return