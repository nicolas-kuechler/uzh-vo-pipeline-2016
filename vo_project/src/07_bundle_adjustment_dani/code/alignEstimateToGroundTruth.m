function [p_W_estimate_aligned, error] = alignEstimateToGroundTruth(...
    p_W_GT, p_W_estimate)

x0 = [0, 0, 0, 0, 0, 0, 1]';

% determine solution
error_function = @(x) alignmentError(x, p_W_GT, p_W_estimate);
[x, error] = lsqnonlin(error_function, x0);

T = twist2HomogMatrix(x(1:6, 1));
R = T(1:3, 1:3);
t = T(1:3, 4);
s = x(7);

p_W_estimate_aligned = s * R * p_W_estimate + t;
end

