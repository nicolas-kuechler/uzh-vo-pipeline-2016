function hidden_state = runBA(hidden_state, observations, K)

n = observations(1);
m = observations(2);

% find all ks and li
ks = zeros(1, 4);
lis = cell(n, 1);
counter = 3;
for i = 1 : n
    k_i = observations(counter);
    ks(i) = k_i;
    lis{i} = observations(counter + 1 + 2 * k_i : counter + 3 * k_i)';   
    counter = counter + 1 + 3 * k_i;
end

M = sparse(2 * sum(ks), 6*n+3*m);

% encode dependencies of p_i_ki with tau_i
row_counter = 1; col_counter = 1; num_kp_counter = 1;
for i = 1 : n 
    % dependencies of p_i_ki with tau_i
    M(row_counter : row_counter + 2 * ks(i) - 1, ...
        col_counter : col_counter + 5) = 1;   
    row_counter = row_counter + 2 * ks(i);
    col_counter = col_counter + 6;
    
    % dependencies of p_i_ki & P
    li = lis{i};
    for k = 1 : k_i
        kp_indices = num_kp_counter :  num_kp_counter + 1;
        num_kp_counter = num_kp_counter + 2;
        
        landmark_indices = 6 * n + 3 * li(k) - 2 : 6 * n + 3 * li(k);  
        M(kp_indices, landmark_indices) = 1;
    end
end

options = optimoptions('lsqnonlin', 'JacobPattern', M);

error_function = @(hidden_state) BAError(hidden_state, observations, K);
hidden_state = lsqnonlin(error_function, hidden_state, [], [], options);
end 
