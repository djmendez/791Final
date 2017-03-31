function P = computePredator(P,MSN,t,params)
%move predator
if isPreyDetected(MSN.center_mass(t,:),P.pos(t-1,:),P.predator_visibility)
    pred_target = MSN.center_mass(t,:);
else
    distance = sqrt((P.pos(t-1,1) - params.safe_center(1))^2 + ...
        (P.pos(t-1,2) - params.safe_center(2))^2);
    if distance < 10
        pred_target = [1200 randi(600,1)];
        P.outbound = true;
    elseif distance > 500 && P.outbound
        pred_target = params.safe_center;
        P.outbound = false;
    elseif P.outbound
        pred_target = [1200 randi(600,1)];
    else
        pred_target = params.safe_center;
    end
end
P.pos(t,:) = P.pos(t-1,:) + computeMovement(P.pos(t-1,:),pred_target,P.vel,params.dt);
return

function prey = isPreyDetected(MSN_center,predpos,predvisibility)
prey = false;
distance = sqrt((predpos(1) - MSN_center(1))^2 + ...
    (predpos(2) - MSN_center(2))^2);
if distance < predvisibility
    prey = true;
end
return
