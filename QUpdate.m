function Q = QUpdate(MSN, Q, t, p)
    % performs a q-update step using actions, states, and rewards
    % for each robot's Q matrix update current state action pair
    for i = 1:p.maxnodes
        % variables for clarity
        current_state = MSN.state(t-1,i,1) + (p.maxnodes * MSN.state(t-1,i,2));
        next_state = MSN.state(t,i,1) + (p.maxnodes * MSN.state(t,i,2));
        [~, max_next_action] = max(Q(next_state, :, i));
        Q_state_action = Q(current_state, MSN.action(t-1, i), i);
        
        %do the q update equation
        update = Q_state_action + (p.learning_rate * (MSN.reward(t,i) + ...
            (p.discount_factor * max_next_action) - Q_state_action));
        
        % if the trust weight is less than 1, consider neighbors q-values
        % come back to this later
        %if (trust_weight < 1)
        %    update = (trust_weight * update) + ((1 - trust_weight) * sum_neighbors(states(i), chosen_actions(i), Q_mat, neighbors{i}));
        %end
        
        %state an action should really be a pair at the same times
        %still need to figure this out
        Q(current_state, MSN.action(t-1, i), i) = update;
    end
end