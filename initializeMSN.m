
function MSN = initializeMSN(MSN,p)

MSN.reward = zeros(p.timesteps, p.maxnodes,1);

MSN.connectivity = zeros(p.timesteps,1);
MSN.reward_all = zeros(p.timesteps,1);

%{
% Initial Configuration
p.startpos_center = randi(p.maxgrid+200,3,1);
% Initial Configfuration - random for training - fix for publish
if p.publish
    p.startpos_center =[100 100];
else
    p.startpos_center = randi(p.maxgrid+200,2,1);
end
startpos(:,1) = startpos(:,1) + p.startpos_center(1);
startpos(:,2) = startpos(:,2) + p.startpos_center(2);
startpos(:,3) = startpos(:,3) + p.startpos_center(3);
%}

startpos = randi(p.maxgrid,p.maxnodes,p.dimensions);

MSN.pos = zeros(p.timesteps,p.maxnodes,p.dimensions);
MSN.pos(1,:,:) = startpos;

startaction = randi(p.num_actions,p.maxnodes,1);
MSN.action = zeros(p.timesteps,p.maxnodes,1);
MSN.action(1,:,:) = startaction;

% state is a tuple {neighbors,pred_direction}
MSN.state = zeros(p.timesteps,p.maxnodes,2);
for node = 1:p.maxnodes
    currNode =  [MSN.pos(1,node,1) MSN.pos(1,node,2) MSN.pos(1,node,3)];
    %get neighbors
    [MSN.neighbors{node}] = computeNeighbors(node,currNode,1,MSN,p);
    % number of initial neighbors
    MSN.state(1,node,1) = size(MSN.neighbors{node},2);
    % No predator right away
    MSN.state(1,node,2) = 0;
end

