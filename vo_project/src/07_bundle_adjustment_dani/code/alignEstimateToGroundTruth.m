function [p_W_estimate_aligned, location_error, orientation_error] = ...
    alignEstimateToGroundTruth(ground_truth, location_estimate, orientation_estimate)

% split ground truth in rotations and locations
ground_truth_locations = ground_truth(:, 4:4:12)';
ground_truth_orientations = ground_truth(:, [1,5,9,2,6,10,3,7,11])';

% determine best rescaling and rotation of estimated trajectory with error
x0 = [0, 0, 0, 0, 0, 0, 1]';
error_function = @(x) alignmentError(x, ground_truth_locations, location_estimate);
[x, location_error] = lsqnonlin(error_function, x0);
T = twist2HomogMatrix(x(1:6, 1));
R = T(1:3, 1:3);
t = T(1:3, 4);
s = x(7);

% rescale and rotate estimated trajectory
p_W_estimate_aligned = s * R * location_estimate + t;

% determine orientation error
orientation_error = 0;
for i = 1 : size(ground_truth, 1)
    R_GT = reshape(ground_truth_orientations(:, i), 3, 3);
    R_est = reshape(orientation_estimate(:, i), 3, 3);
    
    error_matrix = (R * R_est) * R_GT';
    orientation_error = orientation_error + norm(rotMatToRotVec(error_matrix))^2;
end
end
