function [cloud, matched_kp, remain, maxAngle] = ...
    tryTriangulate(candidate_kp, kp_track_start, kp_pose_start, T, K)
% TRYTRIANGULATE function that takes candidate keypoints (candidate_k) and
% key points at the start of the track (kp_track_start) and triangulates
% new 3D points if their bearing angle in between is large enough.
% 
% Inputs: candidate_kp: (2 x K) array of tracked candidate keypoints in
%                       current frame
%         kp_track_start: (2 x K) array of starting coordinates of candidate 
%                         keypoints from first detection
%         kp_pose_start: (12 x K) array of camera poses (12 x 1 array) when
%                        the candidate key points were first detected.
%         T: (3 x 4) camera pose from the current frame
%         K: (3 x 3) camera calibration matrix.

threshold = 5; % threshold on in between bearing angle
num_kp = size(candidate_kp, 2);
cloud = [];
matched_kp = [];
remain = true(1, num_kp);
t = [];
% calculate bearing vectors (3 x L) of current candidated key points
% defined as vector pointing from camera origin to corresponding 3D point
candidate_kp_bearings = K \ [candidate_kp; ones(1, num_kp)];

for i = 1 : num_kp
    v = candidate_kp_bearings(:, i);
    T_i = reshape(kp_pose_start(:,i), 3, 4);
    
    % calculate bearing vector of bearing vector of key point at track start
    u = K \ [kp_track_start(:, i); 1];
    
    % transform track start bearing vector in current camera frame
    u = T(:,1:3)' * T_i(:, 1:3) * u;
    
    % calculate the angle between bearing vectors
    Theta = 180 / pi * acos(dot(u/norm(u), v/norm(v)));
    t = [t, Theta];
   
    if Theta > threshold
        % if the angle is big enough triangulate a 3d point between the track
        % start key point and the current candidate key point
        new_3d_pt = linearTriangulation(kp_track_start(:, i), ...
            candidate_kp(:, i), K * T_i, K * T);
        
        % transform into current camera frame
        new_3d_pt_camera_frame = T(:,1:3)'*new_3d_pt - T(:,1:3)'*T(:,4);
        
        % reject new 3d point if it lies behind the camera (z < 0)
        if new_3d_pt_camera_frame(3) > 0
            cloud = [cloud, new_3d_pt];
            matched_kp = [matched_kp, candidate_kp(:, i)];
        end
        % current keypoint is no longer tracked
        remain(i) = false;    
    end
    
end
maxAngle = max(t);
end
