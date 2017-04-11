function MSN = GetAction(MSN, Q)
    %chooses an action for each robot based on the epsilon greedy policy
    %may want to pass this in later to update value as episodes change
    epsilon = .5;
    
    %for each robot node in MSN
    for i = 1:size(MSN.node)
        %get a random number between 0 and 1
        r = rand;
        if (r < epsilon)
            %if r is less than epsilon choose a random action
            choices(i) = actions((randi(numel(actions))));
        else
            % else get the max / most optimal action in the current state
            [~, choices(i)] = max(Q(states(i),:,i));
        end
        ii = ii + 1;
    end
end