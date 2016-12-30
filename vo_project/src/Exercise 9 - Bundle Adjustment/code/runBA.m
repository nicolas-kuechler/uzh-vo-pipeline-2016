function refined_hidden_state = runBA(hidden_state, observations, K)
% run bundle adjustment (BA) on hidden state and observations to refine the
% pose and landmark estimates obtained previously.
%% Build Jacobian Pattern
n = observations(1);
m = observations(2);

% find all ks (keypoint number in each frame) and li (correspondences to 3D
% landmarks)
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
    
    % dependencies of key points and corresponding landmarks
    li = lis{i};
    for k = 1 : ks(i)
        kp_indices = num_kp_counter :  num_kp_counter + 1;
        num_kp_counter = num_kp_counter + 2;
        
        landmark_indices = 6 * n + 3 * li(k) - 2 : 6 * n + 3 * li(k);  
        M(kp_indices, landmark_indices) = 1;
    end
end

% Choose other parameters depending on algorithm being run
pose_indices = 6 * (n - 2) + 1 : 6 * n;
if 2 * sum(ks) < 6 * n + 3 * m % uses levenberg marquart
    MaxIterations = 5;
    lower_bound = [];
    upper_bound = [];
else
    MaxIterations = 20; % uses 'trust-region-reflective'
    % define lower and upper bound poses which have
    % already been bundle adjusted.
    l2 = lis{end - 2};
    l1 = lis{end - 1};
    l_tot = unique([l2', l1']);

    upper_bound = inf * ones(size(hidden_state));
    lower_bound = -inf * ones(size(hidden_state));

    % put bound on first two poses to make them consistent
    upper_bound(pose_indices) =  hidden_state(pose_indices) + 0.1;
    lower_bound(pose_indices) =  hidden_state(pose_indices) - 0.1;
end

options = optimoptions('lsqnonlin', 'MaxIterations', MaxIterations, 'JacobPattern', M, ...
    'Display', 'iter', 'ScaleProblem', 'jacobian');

% Define error function BAError
error_function = @(hidden_state) BAError(hidden_state, observations, K, 2 * sum(ks));

% run BA
refined_hidden_state = lsqnonlin(error_function, hidden_state, lower_bound, upper_bound, options);

% align new point cloud and poses so that the first poses coincide (this is
% done if the chosen algorithm is levenberg marquart which does not allow
% constraints on the poses
if 2 * sum(ks) < 6*n+3*m
    % get first two poses
    prev_poses = hidden_state(pose_indices);

    % new first two poses
    new_poses = refined_hidden_state(pose_indices);

    % define difference vector that must be added to new trajectory to
    % align it.
    inv_H_prev = twist2HomogMatrix(prev_poses(1:6)');
    inv_H_new = twist2HomogMatrix(new_poses(1:6)');
    difference_translation = inv_H_prev(1:3, 4) - inv_H_new(1:3, 4);

    % align poses
    for i = 1 : n
        tau_i = refined_hidden_state(i * 6 - 5: 6 * i);
        inv_H_curr = twist2HomogMatrix(tau_i');
        inv_H_curr(1:3,4) = difference_translation + inv_H_curr(1:3,4);
        refined_hidden_state(i * 6 - 5: 6 * i) = HomogMatrix2twist(inv_H_curr);
    end
    
    % align point cloud
    new_landmarks = reshape(refined_hidden_state(1 + 6 * n : end), 3, m);
    adjusted_landmarks = new_landmarks +  repmat(difference_translation, 1, m);
    refined_hidden_state(1 + 6 * n : end) = adjusted_landmarks(:)';
end 
