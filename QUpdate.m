function Q = QUpdate(MSN, Q, t, p)
    % performs a q-update step using actions, states, and rewards
    % for each robot's Q matrix update current state action pair
    for i = 1:p.maxnodes
        % variables for clarity
        [~, max_next_action] = max(Q(MSN.state(t, i), :, i));
        Q_state_action = Q_mat(MSN.state(t-1, i), MSN.action(t, i), i);
        
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
        Q(MSN.state(t-1, i), MSN.action(t, i), i) = update;
    end
end