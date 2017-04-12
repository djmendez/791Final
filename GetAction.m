function MSN = GetAction(MSN, Q, t, p)
    %chooses an action for each robot based on the epsilon greedy policy
    %may want to pass this in later to update value as episodes change
    epsilon = .5;
    
    %for each robot node in MSN
    for i = 1:p.maxnodes
        %get a random number between 0 and 1
        r = rand;
        if (r < epsilon)
            %if r is less than epsilon choose a random action for time t
            MSN.action(t,i) = randi(p.num_actions);
        else
            % else get the max / most optimal action in the current state
            [~, MSN.action(t,i)] = max(Q(MSN.state(t,i)),:,i);
        end
    end
end