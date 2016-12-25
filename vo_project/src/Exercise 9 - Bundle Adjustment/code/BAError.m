function e = BAErrorFunction( hidden_state, observations, K)
%BAERRORFUNCTION Summary of this function goes here
%   Detailed explanation goes here
n = observations(1);
m = observations(2);

counter = 3;
Y = [];
fx = [];
% separate hidden_state
land_marks = reshape(hidden_state(6 * n + 1 : end), 3, m);
taus = reshape(hidden_state(1: 6 * n), 6, n);

for i = 1 : n
    % get homography for frame i
    homography = twist2HomogMatrix(taus(:, i)); 
    
    % get ith observation
    k_i = observations(counter);
    observation_i = observations(counter + 1: counter + 3 * k_i);
    counter = counter + 3 * k_i + 1;
    
    % extract correspondence and kp
    p_i = observation_i(1 : 2 * k_i);
    li = observation_i(2 * k_i + 1 : 3 * k_i);
    
    % extract fitting land marks
    lm = land_marks(:, li);
    
    % reproject landmarks
    fx_i = reprojectPoints(lm, homography(1:3, :), K);
    
    fx = [fx, fx_i(:)'];
    Y = [Y, p_i];
end

e = Y - fx;
end
