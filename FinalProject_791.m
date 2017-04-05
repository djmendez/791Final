%%%%% CS 791 - Final Project
clc, clear
close all

%%%%%%%%%%%%%%%%%%%%%%%%% INPUT PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%% Frequently changed
total_sim_time = 20;        % total simulation time
params.algorithm = 3;       % Alg1 - plain, Alg2 - attractor target, Alg3 - obstacle avoidance
params.target_movement = 3; % proper values are 0 (static), 1 (line), 2 (sin) 3 (circle) 
% Note that target_movement is only applicable for alg > 1
episodes = 5;
training_runs = 1; 

params.publish = false; % true for final results to get different figures

if params.publish
    % will set origin to a fixed point so as to compare apples to apples
    % will set number of snapshots to 6 to add to paper report
    % will also set
    episodes = 1;
    training_runs = 1;
    % velocity should not drop below 40 without increasing granularity (i.e.
    % number of grids) and retraining. Q matrix provided (training with 160
    % 20x20 grids has been trained for at least that velocity)
    params.vel = 50;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Other key world parameters

%%%%%%%%% World definition parameters
params.maxnodes = 30; 
%%%%%%%%% Other key world parameters
params.maxgrid = 1000;
%params.graphsize = 100;
params.dimensions = 3;
% range to see others
params.d = 200;

params.dt = .008;
params.timesteps = ceil(total_sim_time / params.dt);
snapshots = params.timesteps / 100; % number of snapshotsof movement to take

params.direction = struct('NORTH',1,'EAST',2,'SOUTH',3,'WEST',4,'UP',5,'DOWN',6);

% obstable parameters
params.obstacles.center = [200 400 500; 400 200 700; 500 700 400; 600 500 600; 800 350 300];
params.obstacles.radii = [100; 50; 70; 40; 50];
params.obstacles.number = size(params.obstacles.center,1);

%%%%%%%%%%%%%%% Movement parameters
params.target_origin = [params.maxgrid/2 params.maxgrid/2 params.maxgrid/2];
params.target_qmt = params.target_origin;
params.target_pmt = [15 20 10];
params.circle_radius = params.maxgrid *.35;

%Movement algorithm parameters
params.eps = .1;
params.k = 1.2;

params.r = params.k * params.d;
params.d_prime = 0.6 * params.d;
params.r_prime = 1.2 * params.d_prime;
params.h = .2;

params.r_alpha = (1/params.eps)*(sqrt(1 + (params.eps*params.r^2)) - 1);
params.d_alpha = (1/params.eps)*(sqrt(1 + (params.eps*params.d^2)) - 1);
params.r_beta = (1/params.eps)*(sqrt(1 + (params.eps*params.r_prime^2)) - 1);
params.d_beta = (1/params.eps)*(sqrt(1 + (params.eps*params.d_prime^2)) - 1);
params.h_beta = .9;

% Obstacle movement algorithm parameters - compute here so only do once
params.h_alpha = .2;
params.h_beta = .9;
params.c1 = 30;
params.c2 = 2 * sqrt(params.c1);
params.c1_beta = 1500;
params.c2_beta = 2 * sqrt(params.c1_beta);
params.c1mt = 1.1;
params.c2mt = 2 * sqrt(params.c1mt);

%%%% PREDATOR
Pred.pos = zeros(params.timesteps,params.dimensions);
Pred.prey_visibility = 100;
Pred.predator_visibility = 200;
Pred.vel = 3;
Pred.active = true;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Q leanring world parameters
params.gridsize = params.maxgrid / 40;

params.num_states = params.maxgrid / params.gridsize;
params.num_actions = 5;

params.states = 1:params.num_states;
params.safe_circle_radius = 50;
params.max_distance = params.safe_circle_radius * params.num_states;
params.actions = 1:params.num_actions;

%Q learning algorithm parameters
params.q_learning_algorithm = 1;
params.epsilon = .5;
params.learning_rate = .2;
params.discount_factor = .9;
params.qlearning_r = 30;
params.cl_weight = .8;


%%%%% Q values
use_stored_Q = true;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% Set up data structures
% Structure containing Sensor Network
MSN.node = 1:params.maxnodes;
MSN.neighbors = {};


MSN.target_qmt = zeros(params.timesteps,params.dimensions);
MSN.target_qmt(1,:) = params.target_origin;
MSN.center_mass = zeros(params.timesteps,params.dimensions);

% probably dont need these below but lets leave for now
MSN.vel = zeros(params.timesteps,params.maxnodes,params.dimensions);
MSN.accel = zeros(params.timesteps,params.maxnodes,params.dimensions);
MSN.connectivity = zeros(params.timesteps,1);

% set up Q
if use_stored_Q == true && ...
    exist('\\files\users\djmendez\Documents\CS791\Final\Q.mat','file') == 2
    load('\\files\users\djmendez\Documents\CS791\Final\Q.mat');
else
    Q = zeros(params.maxnodes, params.num_states, params.num_states, params.num_actions);
end

% display figure
h = figure('Name','Simulation Window','units','normalized','position',[.1 .1 .7 .7]);

MSN = initializeMSN(MSN,params);  
% snapshot(MSN,P,1,1,1,params,h);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Main program
episodes = 1;
for e = 1:episodes
    for i = 1:training_runs   
        % Reset MSN
        MSN = initializeMSN(MSN,params);  
        
        %Reset Predator
        Pred.pos(1,:) = [
            randi(params.maxgrid) ...
            randi(params.maxgrid) ...
            randi(params.maxgrid)];
        
        % Reset Qlearning (wait til predator detected)
        params.engage_Qlearning = false;

        for t = 2:params.timesteps
            %Main function: recalc MSN next position
            [MSN,Q,Pred] = computePosition(MSN,Q,Pred,t,params);

            % take a snapshot if needed
            if (mod((params.timesteps-t),floor(params.timesteps / snapshots)) == 0)
                snapshot(MSN,Pred,e,i,t,params,h);
                getframe();
            end
        end
    end
    
    % change parameters between episodes
    %params.epsilon = params.epsilon * .95;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% PLOTS
%%% Plot position of nodes over time
%{
figure('Name','Position');
hold on
for node = 1:params.maxnodes
    plot(MSN.pos(:,node,1),MSN.pos(:,node,2));
end
hold off

%%% Plot COM of nodes and target trajectory over time
figure('Name','COM vs Target');
hold on
plot(MSN.center_mass(2:params.timesteps));
plot(MSN.target_qmt(2:params.timesteps));
hold off

%%% Plot position of nodes over time
figure('Name','Velocity');
velocity_vector = sqrt(MSN.vel(:,:,1).^2 + MSN.vel(:,:,2).^2);
hold on
plot (velocity_vector);
%for node = 1:params.maxnodes
   % plot(MSN.vel(:,node,1),MSN.vel(:,node,2));
%end
hold off

%%% Plot connectivity of nodes over time
figure('Name','Connectivity');
hold on
axis ([0 params.timesteps 0 1]);
plot(MSN.connectivity(2:end));
hold off

save('\\files\users\djmendez\Documents\CS791\Final\Q.mat','Q');
%}