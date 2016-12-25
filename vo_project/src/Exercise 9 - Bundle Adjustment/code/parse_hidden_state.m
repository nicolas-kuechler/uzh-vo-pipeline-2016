function [locations, orientations, land_marks] = parse_hidden_state(hidden_state, n, m)
%PARSE_HIDDEN_STATE Summary of this function goes here
%   Detailed explanation goes here
land_marks = reshape(hidden_state(6*n + 1: end), 3, m);
taus = reshape(hidden_state(1 : 6 * n), 6, n);

locations = zeros(3, n);
orientations = zeros(9, n);
for i = 1 : n
    M = twist2HomogMatrix(taus(:, i));
    locations(:, i) = M(1:3, 4);
    R = M(1:3, 1:3);
    orientations(:, i) = R(:); 
end
end

