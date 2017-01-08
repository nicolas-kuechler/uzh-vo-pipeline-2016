function [p_W_estimate_aligned, x, location_error] = ...
    alignEstimateToGroundTruth(x0, ground_truth, location_estimate)

% split ground truth in rotations and locations
ground_truth_locations = ground_truth(:, 4:4:12)';

% determine best rescaling and rotation of estimated trajectory with error
error_function = @(x) alignmentError(x, ground_truth_locations, location_estimate);

options = optimset('display','iter');

[x, location_error] = lsqnonlin(error_function, x0, [],[],options);
M = twist2HomogMatrix(x(1:6, 1));
R = M(1:3, 1:3);
t = M(1:3, 4);
s = x(7);

% rescale and rotate estimated trajectory
p_W_estimate_aligned = s * R * location_estimate + t;

end
