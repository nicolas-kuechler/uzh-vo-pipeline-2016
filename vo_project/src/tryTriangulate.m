function [cloud, matched_kp, remain] = ...
    tryTriangulate(candidate_kp, kp_track_start, kp_pose_start, T, K)
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
cloud = [];
matched_kp = [];
remain = true(1, num_kp);
% calculate bearing vectors (3 x L) of current candidated key points
% defined as vector pointing from camera origin to corresponding 3D point
candidate_kp_bearings = K \ [candidates_current; ones(1, num_kp)];

for i = 1 : num_kp
    v = candidate_kp_bearings(:, i);
    T_i = reshape(candidates_start_pose(:,i), 3, 4);
    
    % calculate bearing vector of bearing vector of key point at track start
    u = K \ [candidates_start(:, i); 1];
    
    % transform track start bearing vector in current camera frame
    u = T(:,1:3) * T_i(:, 1:3)' * u;
    
    % calculate the angle between bearing vectors
    Theta = 180 / pi * acos(dot(u/norm(u), v/norm(v)));
    
    if Theta > params.triangulation_angle_threshold
        % if the angle is big enough triangulate a 3d point between the track
        % start key point and the current candidate key point
        %new_3d_pt = linearTriangulation(candidates_start(:, i), ...
        %    candidates_current(:, i), K * T_i ,K * T);
        
        %% MATLAB Comparison
        cameraParams = cameraParameters('IntrinsicMatrix',K');
        orientation1 = T_i(:,1:3)';
        location1 = T_i(:,4);
        cameraMatrix1 = cameraMatrix(cameraParams,orientation1,location1);
        
        orientation2 = T(:,1:3)';
        location2 = T(:,4);
        cameraMatrix2 = cameraMatrix(cameraParams,orientation2,location2);
        
        [worldPoint, reprojectionError]= triangulate(candidates_current(:, i)',candidates_start(:, i)',cameraMatrix2,cameraMatrix1);
        new_3d_pt = worldPoint';
        %% 
        
        % transform into current camera frame
        new_3d_pt_camera_frame = T(:,1:3)*new_3d_pt + T(:,4);
        
        % reject new 3d point if it lies behind the camera (z < 0)
        if reprojectionError(1) < params.triangulate_max_repr_error && new_3d_pt_camera_frame(3) > 0
            cloud = [cloud, new_3d_pt];
            matched_kp = [matched_kp, candidates_current(:, i)];
        end
        % current keypoint is no longer tracked
        remain(i) = false;    
    end
    
end
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
