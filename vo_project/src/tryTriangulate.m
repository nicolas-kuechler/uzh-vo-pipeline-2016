function [cloud, matched_kp, remain] = ...
    tryTriangulate(candidates_current, candidates_start, candidates_start_pose, T, K, not_critical_kp, params)
% TRYTRIANGULATE function that takes candidate keypoints (candidate_k) and
% key points at the start of the track (kp_track_start) and triangulates
% new 3D points if their bearing angle in between is large enough.
% 
% Inputs: candidates_current: (2 x K) array of tracked candidate keypoints in
%                       current frame
%         candidates_start: (2 x K) array of starting coordinates of candidate 
%                         keypoints from first detection
%         candidates_start_pose: (12 x K) array of camera poses (12 x 1 array) when
%                        the candidate key points were first detected.
%         T: (3 x 4) camera pose from the current frame
%         K: (3 x 3) camera calibration matrix.
%         not_critical_kp: flag to show that the keypoint count is critically
%                          low
%         params: parameter struct containing parameters to execute
%                 triangulation

num_kp = size(candidates_current, 2);

% calculate bearing vectors (3 x L) of current candidated key points
% defined as vector pointing from camera origin to corresponding 3D point
v = K \ [candidates_current; ones(1, num_kp)];
v_norm = v ./ sqrt(sum(v.^2));
candidate_start_bearings = K \ [candidates_start; ones(1, num_kp)];

% transform bearings into other frame
T_i = candidates_start_pose;
c = candidate_start_bearings;
W_candidate_start_bearings = [c(1, :) .* T_i(1, :) + c(2, :) .* T_i(2, :) + c(3, :) .* T_i(3, :); ...
                              c(1, :) .* T_i(4, :) + c(2, :) .* T_i(5, :) + c(3, :) .* T_i(6, :); ...
                              c(1, :) .* T_i(7, :) + c(2, :) .* T_i(8, :) + c(3, :) .* T_i(9, :)]; 
u = T(:,1:3) * W_candidate_start_bearings;
u_norm = u ./ sqrt(sum(u.^2));

% calculate angles
angles = 180/pi * acos(sum(u_norm .* v_norm));
tri = angles > params.triangulation_angle_threshold;

% triangulate all eligible points 
new_3d_pt = zeros(3, sum(tri));
repr_Errors = zeros(1, sum(tri));
triangulation_indices = find(tri); 
counter = 1;

for i = triangulation_indices
    T_i = reshape(candidates_start_pose(:, i), 3, 4); 

    [worldPoint, reprojectionError] = triangulate(candidates_current(:,i)', ...
        candidates_start(:,i)', T' * K', T_i' * K');

    repr_Errors(:,counter) = reprojectionError;
    new_3d_pt(:,counter) = worldPoint';
    counter = counter + 1;
end

% remove those with high repr error and negative z
z_component = T(3, 1 : 3) * new_3d_pt + T(3, 4);
z_mask = z_component > 0;
if not_critical_kp
    r_mask = repr_Errors < params.triangulate_max_repr_error;
else
    r_mask = repr_Errors > 0;
end

% extract all points and keypoints and construct remain vector
cloud = new_3d_pt(:, z_mask & r_mask);
matched_kp = candidates_current(:, triangulation_indices(z_mask & r_mask));

remain = ~tri;

end
