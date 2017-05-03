
function do_plots(MSN,params,sample_step,Pred,use_stored_Q)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% PLOTS
%%% Plot position of nodes over time

if use_stored_Q; Qstring = 'True'; else Qstring = 'False'; end

figure('Name','Reward');
hold on
switch params.reward
    case 1
        firstline = 'Reward #1: number of neighbors';
        axis ([0 sample_step 0 params.maxnodes])
    case 2
        firstline = 'Reward #2: +10 if distance from predator is increased';
        axis ([0 sample_step 0 20])
    case 3
        firstline = 'Reward #3 (Combined):number of neighbors +5 if increased distance';
        axis ([0 sample_step 0 params.maxnodes])
end
secondline = sprintf('Q: %s: Average reward: %2.2f',Qstring,mean(MSN.Report_Reward(2:end)));
title ({firstline,secondline})
xlabel('Training Iteration')
ylabel('Reward')
plot(MSN.Report_Reward(2:end))
hold off

figure('Name','Number of Neighbors');
hold on
secondline = sprintf('Q: %s: Average neighbors: %2.2f',Qstring,mean(MSN.Report_NN(2:end)));
title (secondline)
axis ([0 sample_step 0 params.maxnodes])
xlabel('Training Iteration')
ylabel('Number of Neighbors')
plot(MSN.Report_NN(2:end))
hold off

figure('Name','Mean Predator Distance');
hold on
secondline = sprintf('Q: %s: Average distance: %2.2f Pred:%3.0f %3.0f %3.0f',Qstring,...
    mean(MSN.Report_Mean_pred_distance(2:end)), ...
    Pred.pos(1,1),Pred.pos(1,2),Pred.pos(1,3));
title (secondline)
axis ([0 sample_step 0 params.maxgrid])
xlabel('Training Iteration')
ylabel('Mean Predator Distance')
plot(MSN.Report_Mean_pred_distance(2:end))
hold off

end
