function [R_C_W, t_C_W, inlier_mask] = ...
    ransacLocalizationSpecial(query_keypoints, all_matches, p_W_landmarks, K)
% query_keypoints should be 2x1000
% all_matches should be 1x1000 and correspond to the output from the
%   matchDescriptors() function from exercise 3.
% inlier_mask should be 1xnum_matched (!!!) and contain, only for the
%   matched keypoints (!!!), 0 if the match is an outlier, 1 otherwise.
assert(size(query_keypoints, 2 ) == size(all_matches,2));

use_p3p = false;

if use_p3p
    num_iterations = 200;
    pixel_tolerance = 3;
    k = 3;
else
    num_iterations = 2000;
    pixel_tolerance = 5;
    k = 6;
end

corresponding_landmarks = p_W_landmarks(:, all_matches);

% Initialize RANSAC.
inlier_mask = zeros(1, size(query_keypoints, 2));
max_num_inliers = 0;

% RANSAC
for i = 1:num_iterations
    [landmark_sample, idx] = datasample(...
        corresponding_landmarks, k, 2, 'Replace', false);
    keypoint_sample = query_keypoints(:, idx);
    
    if use_p3p
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
    else
        M_C_W_guess = estimatePoseDLT(...
            keypoint_sample, landmark_sample, K);
        R_C_W_guess = M_C_W_guess(:, 1:3);
        t_C_W_guess = M_C_W_guess(:, end);
    end
    
    % Count inliers:
    projected_points = projectPoints(...
        (R_C_W_guess(:,:,1) * corresponding_landmarks) + ...
        repmat(t_C_W_guess(:,:,1), ...
        [1 size(corresponding_landmarks, 2)]), K);
    difference = query_keypoints - projected_points;
    errors = sum(difference.^2, 1);
    is_inlier = errors < pixel_tolerance^2;
    
    if use_p3p
        projected_points = projectPoints(...
            (R_C_W_guess(:,:,2) * corresponding_landmarks) + ...
            repmat(t_C_W_guess(:,:,2), ...
            [1 size(corresponding_landmarks, 2)]), K);
        difference = query_keypoints - projected_points;
        errors = sum(difference.^2, 1);
        alternative_is_inlier = errors < pixel_tolerance^2;
        if nnz(alternative_is_inlier) > nnz(is_inlier)
            is_inlier = alternative_is_inlier;
        end
    end
    
    if nnz(is_inlier) > max_num_inliers
        max_num_inliers = nnz(is_inlier);        
        inlier_mask = is_inlier;
    end
end

if max_num_inliers == 0
    R_C_W = [];
    t_C_W = [];
else
    M_C_W = estimatePoseDLT(...
        query_keypoints(:, inlier_mask>0), ...
        corresponding_landmarks(:, inlier_mask>0), K);
    R_C_W = M_C_W(:, 1:3);
    t_C_W = M_C_W(:, end);
end

end

