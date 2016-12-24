function [cloud, matched_kp, remain] = ...
    tryTriangulate(candidates_current, candidates_start, candidates_start_pose, T, K, params)
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
%% MATLAB Comparison
cameraParams = cameraParameters('IntrinsicMatrix',K');
new_3d_pt = zeros(3, sum(tri));
repr_Errors = zeros(3, sum(tri));
triangulation_indices = find(tri); 
counter = 1;

for i = triangulation_indices
    T_i = reshape(candidates_start_pose(:, i), 3, 4); 
    cameraMatrix1 = cameraMatrix(cameraParams, T_i(:,1:3)', T_i(:,4));
    cameraMatrix2 = cameraMatrix(cameraParams, T(:,1:3)', T(:,4));

    [worldPoint, reprojectionError] = triangulate(candidates_current(:,i)', ...
        candidates_start(:,i)', cameraMatrix2, cameraMatrix1);

    repr_Errors(:,counter) = reprojectionError;
    new_3d_pt(:,counter) = worldPoint';
    counter = counter + 1;
end

% remove those with high repr error and negative z
z_component = T(3, 1 : 3) * new_3d_pt + T(3, 4);
z_mask = z_component > 0;
r_mask = repr_Errors < params.triangulate_max_repr_error;

% extract all points and keypoints and construct remain vector
cloud = new_3d_pt(z_mask & r_mask);
matched_kp = candidates_current(:, triangulation_indices(z_mask & r_mask));

remain = ~tri;

% 
% for i = 1 : num_kp
%     v = candidate_kp_bearings(:, i);
%     T_i = reshape(candidates_start_pose(:,i), 3, 4);
%     
%     % transform track start bearing vector in current camera frame
%     u = T(:,1:3) * T_i(:, 1:3)' * candidate_start_bearings(:, i);
%     
%     % calculate the angle between bearing vectors
%     Theta = 180 / pi * acos(dot(u/norm(u), v/norm(v)));
%     
%     if Theta > params.triangulation_angle_threshold
%         % if the angle is big enough triangulate a 3d point between the track
%         % start key point and the current candidate key point
%         %new_3d_pt = linearTriangulation(candidates_start(:, i), ...
%         %    candidates_current(:, i), K * T_i ,K * T);
%         
%         %% MATLAB Comparison
%         cameraParams = cameraParameters('IntrinsicMatrix',K');
%         orientation1 = T_i(:,1:3)';
%         location1 = T_i(:,4);
%         cameraMatrix1 = cameraMatrix(cameraParams,orientation1,location1);
%         
%         orientation2 = T(:,1:3)';
%         location2 = T(:,4);
%         cameraMatrix2 = cameraMatrix(cameraParams,orientation2,location2);
%         
%         [worldPoint, reprojectionError]= triangulate(candidates_current(:, i)',candidates_start(:, i)',cameraMatrix2,cameraMatrix1);
%         new_3d_pt = worldPoint';
%         %% 
%         
%         % transform into current camera frame
%         new_3d_pt_camera_frame = T(:, 1:3) * new_3d_pt + T(:, 4);
%         
%         % reject new 3d point if it lies behind the camera (z < 0)
%         if reprojectionError(1) < params.triangulate_max_repr_error && new_3d_pt_camera_frame(3) > 0
%             cloud = [cloud, new_3d_pt];
%             matched_kp = [matched_kp, candidates_current(:, i)];
%         end
%         % current keypoint is no longer tracked
%         remain(i) = false;    
%     end
%     
% end
debug = false;
%% DEBUG
if debug
    figure(4)
    imshow(new_image);
    hold on;
    % plot track start of each candidate keypoints starts
    plot(candidates_start(1,:), candidates_start(2,:), 'go', 'Linewidth', 2);
    % plot track start of each candidate keypoints
    plot(candidates_current(1,:), candidates_current(2,:), 'rx', 'Linewidth', 2);
end

end
