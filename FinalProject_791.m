%%%%% CS 791 - Final Project
clc, clear
close all

%%%%%%%%%%%%%%%%%%%%%%%%% INPUT PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%% Frequently changed Parameters HERE
% Reward type: 
% (1) Straigh Number neighbors 
% (2) Distance to Predator 
% (3) Combo
params.reward = 3;

% enable for training runs (disables graphics)
params.training = true;

% total fish
params.maxnodes = 15; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% set running parameters depending on whether training or display
if params.training
    episodes = 2;
    training_runs = 5;
    total_sim_time = 10;        % total simulation time
    snapshot_frequency = 100;    % fewer snapshots -- although turning off altogether
    take_video = false;    
    %%%%% Use stored Qvalues table
    use_stored_Q = true;
else
    episodes = 1;
    training_runs = 2;
    total_sim_time = 5;         % total simulation time
    snapshot_frequency = 1;     % lower number so it looks more smooth
    take_video = true;
    %%%%% ALSO Use stored Qvalues table - duplicated for flexibility
    use_stored_Q = true;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Other key world parameters
% Flocking parameters - should not change anymore
params.algorithm = 3;       % Alg1 - plain, Alg2 - attractor target, Alg3 - obstacle avoidance
params.target_movement = 3; % proper values are 0 (static), 1 (line), 2 (sin) 3 (circle) 
% Note that target_movement is only applicable for alg > 1

params.maxgrid = 1000;
params.dimensions = 3;
%range to see others
params.d = 200;

% snapshots
params.dt = .008;
params.timesteps = ceil(total_sim_time / params.dt);
snapshots = params.timesteps / snapshot_frequency; 

params.direction = struct('NORTH',1,'EAST',2,'SOUTH',3,'WEST',4,'UP',5,'DOWN',6);

%%%%%%%%%%%%%%% Obstable parameters
params.obstacles.center = [200 400 500; 400 200 700; 500 700 400; 600 500 600; 800 350 300];
params.obstacles.radii = [100; 50; 70; 40; 50];
params.obstacles.number = size(params.obstacles.center,1);

%%%%%%%%%%%%%%% Movement parameters
params.target_origin = [params.maxgrid/2 params.maxgrid/2 params.maxgrid/2];

params.circle_radius = params.maxgrid *.35;
%target position and velocity *for when no predator is used*
params.target_qmt = params.target_origin;
params.target_pmt = [15 20 10];
% when using predator/qlearning, below controls how far is the target for
% an action and the speed the fish move to get there
params.target_distance = 200;
params.target_velocity = 20;

%Movement algorithm parameters
params.eps = .3;
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
%these parameters below control how fast fish moves towards target and
%tries to match its speed (params.target_velocity)
params.c1mt = 1.5;
params.c2mt = 2 * sqrt(params.c1mt);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Predator definition
Pred.pos = zeros(params.timesteps,params.dimensions);
Pred.prey_visibility = 100;
Pred.predator_visibility = 200;
Pred.vel = 2;
Pred.active = true;
params.maxpred = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Q learning world parameters
params.gridsize = params.maxgrid / 40;

params.num_states = params.maxnodes + (params.maxnodes * (params.maxpred * 6));
params.num_actions = 5;

params.states = 1:params.num_states;
params.safe_circle_radius = 50;
params.max_distance = params.safe_circle_radius * params.num_states;
params.actions = 1:params.num_actions;

%Q learning algorithm parameters
params.enable_Qlearning = 1;
params.q_learning_algorithm = 1;
params.epsilon = .5;
params.learning_rate = .2;
params.discount_factor = .9;
params.qlearning_r = 30;
params.cl_weight = .8;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Data capture for final reporting
%%% reward: per training run, per timestep
%%% capture the mean network reward
MSN.Report_Reward = zeros(episodes*training_runs,params.timesteps);
MSN.Report_NN = zeros(episodes*training_runs,params.timesteps);
MSN.Report_Pred_distance = zeros(episodes*training_runs,params.timesteps,params.maxnodes);
MSN.Report_Mean_pred_distance = zeros(episodes*training_runs,params.timesteps);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% Set up data structures
% Structure containing Sensor Network
MSN.node = 1:params.maxnodes;
MSN.neighbors = {};

%MSN.target_qmt = zeros(params.timesteps,params.dimensions);
MSN.target_qmt(1,:) = params.target_origin;
MSN.center_mass = zeros(params.timesteps,params.dimensions);

% probably dont need these below but lets leave for now
MSN.vel = zeros(params.timesteps,params.maxnodes,params.dimensions);
MSN.accel = zeros(params.timesteps,params.maxnodes,params.dimensions);
MSN.connectivity = zeros(params.timesteps,1);

% set up Q
% filename = '\\files\users\djmendez\Documents\CS791\Final\Q.mat'
Qfile_reward = sprintf('Q%d.mat',params.reward);

if use_stored_Q == true && ...
    exist(Qfile_reward,'file') == 2
    load(Qfile_reward);
else
    Q = zeros(params.num_states, params.num_actions, params.maxnodes);
end

% display figure
if ~params.training
    h = figure('Name','Simulation Window','units','normalized','position',[.1 .1 .7 .7]);
end

MSN = initializeMSN(MSN,params);  
%Prep for eventual movie
if take_video
    movie_frames = snapshots; %set number of frames for the movie
    mov(1:movie_frames) = struct('cdata', [],'colormap', []); % Preallocate movie structure.
    v = VideoWriter('\\files\users\djmendez\Documents\CS791\Final\flocking.mp4', 'MPEG-4');
    open(v);
    snapshot(MSN,P,1,1,1,params,h);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Main program
MSN.current_run = 1;
for e = 1:episodes
    for i = 1:training_runs   
        % Reset MSN
        MSN = initializeMSN(MSN,params);  
        
        %Reset Predator
        Pred.pos(1,:) = [
            randi(params.maxgrid) ...
            randi(params.maxgrid) ...
            randi(params.maxgrid)];
        
%         % Reset Qlearning (wait til predator detected)
%         params.engage_Qlearning = false;

        for t = 2:params.timesteps
            %Main function: recalc MSN next position
            [MSN,Q,Pred] = computePosition(MSN,Q,Pred,t,params);

            % take a snapshot if needed
            if ~params.training
                if (mod((params.timesteps-t),floor(params.timesteps / snapshots)) == 0)
                    snapshot(MSN,Pred,e,i,t,params,h);
                    frame = getframe();
                    if take_video
                        writeVideo(v,frame);
                    end
                end
            end
            
            % record reward
            MSN.Report_Reward(MSN.current_run,t) = mean(MSN.reward(t,:));
            MSN.Report_NN(MSN.current_run,t) = mean(cellfun(@(x) numel(x),MSN.neighbors));          
            MSN.Report_Mean_pred_distance(MSN.current_run,t) = ...
                mean(MSN.Report_Pred_distance(MSN.current_run,t,:));
        end
        MSN.current_run = MSN.current_run + 1;
    end
    
    % change parameters between episodes
    %params.epsilon = params.epsilon * .95;
end

if take_video
    close(v);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% PLOTS
%%% Plot position of nodes over time

figure('Name','Reward');
hold on
for run = 1:MSN.current_run-1
    plot(MSN.Report_Reward(run,2:end))
end
hold off

figure('Name','Number of Neighbors');
hold on
for run = 1:MSN.current_run-1
    plot(MSN.Report_NN(run,2:end))
end
hold off

figure('Name','Mean Predator Distance');
hold on
for run = 1:MSN.current_run-1
    plot(MSN.Report_Mean_pred_distance(run,2:end))
end
hold off

save(Qfile_reward,'Q');
