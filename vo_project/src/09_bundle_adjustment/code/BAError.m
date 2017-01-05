function e = BAErrorFunction( hidden_state, observations, K, y_dim)
%BAERRORFUNCTION error function that is called where running runBA
n = observations(1);
m = observations(2);

counter = 3;
kp_counter = 1;
Y = zeros(1,y_dim);
fx = zeros(1, y_dim);

% separate hidden_state
land_marks = reshape(hidden_state(6 * n + 1 : end), 3, m);
taus = reshape(hidden_state(1: 6 * n), 6, n);

for i = 1 : n
    % get homography for frame i (inv because inverse is saved in the
    % hidden state)
    homography = twist2HomogMatrix(taus(:, i)); 
    
    % get ith observation
    k_i = observations(counter);
    observation_i = observations(counter + 1: counter + 3 * k_i);
    
    % extract correspondence and kp
    p_i = observation_i(1 : 2 * k_i);
    li = observation_i(2 * k_i + 1 : 3 * k_i);
    
    % extract fitting land marks
    lm = land_marks(:, li);
    
    % reproject landmarks
    R = homography(1:3, 1:3)';
    t = -R * homography(1:3, 4);
    
    fx_i = reprojectPoints(lm, [R, t], K);
    
    fx(kp_counter : kp_counter + 2 * k_i - 1) = fx_i(:)';
    Y(kp_counter : kp_counter + 2 * k_i - 1) = p_i;
    
    % increment counter
    kp_counter = kp_counter + 2 * k_i;
    counter = counter + 3 * k_i + 1;
end

e = Y - fx;
end
