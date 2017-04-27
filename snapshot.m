function snapshot(MSN,pred,e,i,t,p,h)

if p.training
    figure('Name','Simulation Window','units','normalized','position',[.1 .1 .7 .7]);
else
    figure(h);
    clf;
end

hold on
title(sprintf('Episode %d Iteration: %d Timestep: %d, at time %2.1f seconds',e,i,t,t*p.dt));
xlabel('X: Front')
ylabel('Y: Deep')
zlabel('Z: High')

% these look good, maybe change during simulation like rotating around tank
view(-30, 20)
c = [0 .9 0];
fa = .1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MAKE AQUARIUM 
p1 = [0 p.maxgrid 0];
p2 = [p.maxgrid p.maxgrid 0];
p3 = [p.maxgrid p.maxgrid p.maxgrid];
p4 = [0 p.maxgrid p.maxgrid]; 
p5 = [p.maxgrid 0 0];
p6 = [p.maxgrid 0 p.maxgrid];
p7 = [0 0 0];
p8 = [0 0 p.maxgrid];

%Back Wall (y = 1000)
x = [p1(1) p2(1) p3(1) p4(1)];
y = [p1(2) p2(2) p3(2) p4(2)];
z = [p1(3) p2(3) p3(3) p4(3)];
fill3(x,y,z,c,'FaceAlpha',fa)

%Front Wall (y = 0)
x = [p5(1) p6(1) p8(1) p7(1)];
y = [p5(2) p6(2) p8(2) p7(2)];
z = [p5(3) p6(3) p8(3) p7(3)];
fill3(x,y,z,c,'FaceAlpha',.05)

% Right Side wall (x = 1000)
x = [p2(1) p5(1) p6(1) p3(1)];
y = [p2(2) p5(2) p6(2) p3(2)];
z = [p2(3) p5(3) p6(3) p3(3)];
fill3(x,y,z,c,'FaceAlpha',fa)

% Left Side wall (x = 0)
x = [p7(1) p1(1) p4(1) p8(1)];
y = [p7(2) p1(2) p4(2) p8(2)];
z = [p7(3) p1(3) p4(3) p8(3)];
fill3(x,y,z,c,'FaceAlpha',.05)

% Bottom (z = 0) 
x = [p7(1) p1(1) p2(1) p5(1)];
y = [p7(2) p1(2) p2(2) p5(2)];
z = [p7(3) p1(3) p2(3) p5(3)];
fill3(x,y,z,'black','FaceAlpha',.5)

%axis equal
axis ([0 p.maxgrid 0 p.maxgrid 0 p.maxgrid]);
scatter3(MSN.pos(t,:,1),MSN.pos(t,:,2),MSN.pos(t,:,3),50,'filled','>','MarkerFaceColor','r')

for node = 1:p.maxnodes
    currNode = [MSN.pos(t,node,1) MSN.pos(t,node,2) MSN.pos(t,node,3)];    
    if t == 1
        [MSN.neighbors{node}] = computeNeighbors(node,currNode,1,MSN,p);    
    end
    for neighbor= MSN.neighbors{node}
        currNeighbor = [MSN.pos(t,neighbor,1) MSN.pos(t,neighbor,2) MSN.pos(t,neighbor,3)];
        line([currNode(1) currNeighbor(1)],[currNode(2) currNeighbor(2)],...
            [currNode(3) currNeighbor(3)])
    end
end

%draw obstacles
for obstacle = 1:p.obstacles.number
    [x,y,z] = sphere();
    surf((x*p.obstacles.radii(obstacle))+p.obstacles.center(obstacle,1),...
        (y*p.obstacles.radii(obstacle))+p.obstacles.center(obstacle,2),...
        (z*p.obstacles.radii(obstacle))+p.obstacles.center(obstacle,3),...
        'Facecolor','blue','FaceAlpha',.8,'FaceLighting','gouraud')
    
    [x,y,z] = cylinder();
    surf((x*20)+p.obstacles.center(obstacle,1),...
        (y*20)+p.obstacles.center(obstacle,2),...
        (z*p.obstacles.center(obstacle,3)),...
        'Facecolor','blue','FaceAlpha',.8,'FaceLighting','gouraud')
        
end


%draw target if there is a moving target
if ~p.enable_Qlearning && p.target_movement > 0
    scatter3(MSN.target_qmt(t,1),MSN.target_qmt(t,2),MSN.target_qmt(t,3),50,'filled','o','MarkerFaceColor','blue')
    plot3(MSN.target_qmt(max(2,t-200):t,1),...
            MSN.target_qmt(max(2,t-200):t,2), ...
            MSN.target_qmt(max(2,t-200):t,3),'blue');
end


%draw predator if there is one
if pred.active 
    phi = 0:.1:2.05*pi;
%     plot3(pred.pos(max(2,t-200):t,1),pred.pos(max(2,t-200):t,2),pred.pos(max(2,t-200):t,3),'r');
    plot3(pred.pos(t,1),pred.pos(t,2),pred.pos(t,3),'*','MarkerFaceColor','red');
%     X = (pred.prey_visibility/2)*cos(phi); 
%     Y = (pred.prey_visibility/2)*sin(phi); 
%     plot((X*(p.r/10))+pred.pos(t,1),(Y*(p.r/10))+pred.pos(t,2),'r') 
%     X = (pred.predator_visibility/2)*cos(phi); 
%     Y = (pred.predator_visibility/2)*sin(phi); 
%     plot((X*(p.r/10))+pred.pos(t,1),(Y*(p.r/10))+pred.pos(t,2),'r') 
end

hold off