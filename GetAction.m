function MSN = GetAction(MSN, Q, t, p)
    %chooses an action for each robot based on the epsilon greedy policy
    %may want to pass this in later to update value as episodes change
    
    %for each robot node in MSN
    for i = 1:p.maxnodes
        %get a random number between 0 and 1
        r = rand;
        if (r < p.qlearning_epsilon)
            %if r is less than epsilon choose a random action for time t
            MSN.action(t-1,i) = randi(p.num_actions-1);
        else
            % else get the max / most optimal action in the current state
            state = MSN.state(t-1,i);
            %disp (state);
            [~, idx] = max(Q(i,state,:));
            if (idx ~= p.direction.NONE)
                MSN.action(t-1,i) = idx;
            else
                MSN.action(t-1,i) = randi(p.num_actions-1);
            end
        end
    end
end