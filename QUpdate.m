function Q = QUpdate(MSN, Q, p)
    % performs a q-update step using actions, states, and rewards
    % for each robot's Q matrix update current state action pair
    for n = 1:num_robots
        % variables for clarity
        [~, max_next_action] = max(Q_mat(next_states(n), :, n));
        Q_state_action = Q_mat(states(n), chosen_actions(n), n);
        
        update = Q_state_action + (learning_rate * (rewards(n) + ...
            (discount_factor * max_next_action) - Q_state_action));
        
        % if the trust weight is less than 1, consider neighbors q-values
        if (trust_weight < 1)
            update = (trust_weight * update) + ((1 - trust_weight) * sum_neighbors(states(n), chosen_actions(n), Q_mat, neighbors{n}));
        end
        
        Q_mat(states(n), chosen_actions(n), n) = update;
    end
end