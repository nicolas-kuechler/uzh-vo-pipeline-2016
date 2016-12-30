function [locations, orientations, land_marks] = parse_hidden_state(hidden_state, n, m)
%PARSE_HIDDEN_STATE takes the hidden_state and returns the orientations and
% locations of each pose in the format M^-1 where M is the homography
% matrix.
% 
% inputs: hidden_state: data structure as described in the BA exercise
%         n: number of frames that will be bundle adjusted
%         m: number of landmarks that were used in the total observations
%
% outputs: locations: 3 x n matrix with each column being the position of the
%                     camera in the world frame.
%          orientations: 9 x n matrix with each column being R(:) where is
%                        the camera orientation in the world frame
%          land_marks: 3 x m matrix containing the landmarks that are in
%                      the hidden state. 

land_marks = reshape(hidden_state(6*n + 1: end), 3, m);
taus = reshape(hidden_state(1 : 6 * n), 6, n);

locations = zeros(3, n);
orientations = zeros(9, n);
for i = 1 : n
    % M are saved as [R, T; 0, 0, 0, 1]^-1  where R and T denote the homography matrix. 
    M = twist2HomogMatrix(taus(:, i));
    locations(:, i) = M(1:3, 4);
    R = M(1:3, 1:3)';
    orientations(:, i) = R(:); 
end
end

