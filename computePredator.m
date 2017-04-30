function Pred = computePredator(Pred,MSN,t,p)
%move predator

% Compute Center of mass
%     MSN.center_mass(t-1,1) = mean(MSN.pos(t-1,:,1));
%     MSN.center_mass(t-1,2) = mean(MSN.pos(t-1,:,2));
%     MSN.center_mass(t-1,3) = mean(MSN.pos(t-1,:,3));


%     if isPreyDetected(MSN.center_mass(t-1,:),Pred.pos(t-1,:),Pred.predator_visibility)
%        pred_target = MSN.center_mass(t-1,:);
%     else
        pred_target = [ ...
            (p.maxgrid/4+randi(p.maxgrid/2)) ...
            (p.maxgrid/4+randi(p.maxgrid/2)) ...
            (p.maxgrid/4+randi(p.maxgrid/2))];
%     end
    
    delta_x = pred_target(1) - Pred.pos(t-1,1);
    delta_y = pred_target(2) - Pred.pos(t-1,2);
    delta_z = pred_target(3) - Pred.pos(t-1,3);
    
    %calculate angle in degrees
    % Angle from the z-axis approach
    z_theta = atan2d(delta_z,delta_x);
    % Angle from the y-axis approach
    y_theta = atan2d(delta_y,delta_x);
    
    Pred.pos(t,1) =  max(0,min(p.maxgrid,Pred.pos(t-1,1) + (Pred.vel * p.dt * delta_x)));
    Pred.pos(t,2) =  max(0,min(p.maxgrid,Pred.pos(t-1,2) + (Pred.vel * p.dt * cos(y_theta) * delta_y)));
    Pred.pos(t,3) =  max(0,min(p.maxgrid,Pred.pos(t-1,3) + (Pred.vel * p.dt * cos(z_theta) * delta_z)));
return

function prey = isPreyDetected(MSN_center,predpos,predvisibility)
prey = false;
distance = norm(predpos-MSN_center,2);
if distance < predvisibility
    prey = true;
end
return
