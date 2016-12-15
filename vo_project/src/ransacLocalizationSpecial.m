function [R, T, inlier_mask] = ...
    ransacLocalizationSpecial(query_keypoints, p_W_landmarks, K)
% N: number of 2D->3D matches
% query_keypoints should be 2xN
% p_W_landmarks should be 3xN
% inlier_mask should be 1xN, 0 if the match is an outlier, 1 otherwise.

assert(size(query_keypoints, 2) == size(p_W_landmarks, 2));

num_iterations = 200;
pixel_tolerance = 10;
k = 3;

% Initialize RANSAC.
inlier_mask = zeros(1, size(query_keypoints, 2));
max_num_inliers = 0;

alt = 0;

% RANSAC
for i = 1:num_iterations
    [landmark_sample, idx] = datasample(...
        p_W_landmarks, k, 2, 'Replace', false);
    keypoint_sample = query_keypoints(:, idx);
    
    
    normalized_bearings = K\[keypoint_sample; ones(1, 3)];
    for ii = 1:3
        normalized_bearings(:, ii) = normalized_bearings(:, ii) / ...
            norm(normalized_bearings(:, ii), 2);
    end
    poses = p3p(landmark_sample, normalized_bearings);
    R_C_W_guess = zeros(3, 3, 2);
    t_C_W_guess = zeros(3, 1, 2);
    for ii = 0:1
        R_W_C_ii = real(poses(:, (2+ii*4):(4+ii*4)));
        t_W_C_ii = real(poses(:, (1+ii*4)));
        R_C_W_guess(:,:,ii+1) = R_W_C_ii';
        t_C_W_guess(:,:,ii+1) = -R_W_C_ii'*t_W_C_ii;
    end
    
    % Count inliers:
    projected_points = reprojectPoints(p_W_landmarks, ...
        [R_C_W_guess(:,:,1) t_C_W_guess(:,:,1)], K);
    difference = query_keypoints - projected_points;
    errors = sum(difference.^2, 1);
    is_inlier = errors < pixel_tolerance^2;
    
    projected_points = reprojectPoints(p_W_landmarks, ...
        [R_C_W_guess(:,:,2) t_C_W_guess(:,:,2)], K);
    
    difference = query_keypoints - projected_points;
    errors = sum(difference.^2, 1);
    alternative_is_inlier = errors < pixel_tolerance^2;
    if nnz(alternative_is_inlier) > nnz(is_inlier)
        alt = 2;
        is_inlier = alternative_is_inlier;
    else
        alt = 1;
    end
    
    if nnz(is_inlier) > max_num_inliers
        max_num_inliers = nnz(is_inlier);
        inlier_mask = is_inlier;
        R = R_C_W_guess(:,:,alt);
        T = t_C_W_guess(:,:,alt);
    end
end
% %ALTERNATIVE ESTIMATION with MATLAB functions
% cameraParams = cameraParameters('IntrinsicMatrix',K');
% 
% [worldOrientation,worldLocation, inlierIdx] = estimateWorldCameraPose(...
%     query_keypoints',p_W_landmarks',cameraParams, 'MaxReprojectionError', 10); 
% 
% inlier_mask = inlierIdx';
end
