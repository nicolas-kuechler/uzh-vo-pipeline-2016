function [R, T, inlier_mask] = ...
    ransacLocalizationSpecial(query_keypoints, p_W_landmarks, K, params)
% save camera parameters
cameraParams = cameraParameters('IntrinsicMatrix',K');

% turn off warnings
 warningstate = warning('off','vision:ransac:maxTrialsReached');
 
% estimate orientation, location and inliers with estimateWorldCameraPose 
[worldOrientation,worldLocation, inlierIdx] = ...
    estimateWorldCameraPose(query_keypoints', p_W_landmarks' ,cameraParams,...
    'MaxNumTrials', 3000, 'Confidence', params.eWCP_confidence, ...
    'MaxReprojectionError', params.eWCP_max_repr_error); 
warning(warningstate)

% format output
inlier_mask = inlierIdx';
R = worldOrientation;
T = -worldOrientation*worldLocation';
end
