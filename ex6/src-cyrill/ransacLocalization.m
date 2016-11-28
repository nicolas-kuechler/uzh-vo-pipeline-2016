function [R_C_W, t_C_W, query_keypoints, all_matches, inlier_mask, ...
    max_num_inliers_history] = ransacLocalization(...
    query_image, database_image, database_keypoints, p_W_landmarks, K)
% query_keypoints should be 2x1000
% all_matches should be 1x1000 and correspond to the output from the
%   matchDescriptors() function from exercise 3.
% inlier_mask should be 1xnum_matched (!!!) and contain, only for the
%   matched keypoints (!!!), 0 if the match is an outlier, 1 otherwise.

    harris_patch_size = 9;
    harris_kappa = 0.08;
    num_keypoints = 1000;
    nonmaximum_supression_radius = 8;
    descriptor_radius = 9;
    match_lambda = 5;
    
    k = 2000;
    
    max_dist_reproj = 10;
    
    query_scores = harris(query_image, harris_patch_size, harris_kappa);
    query_keypoints = selectKeypoints(...
        query_scores, num_keypoints, nonmaximum_supression_radius);
    query_desc = describeKeypoints(query_image, query_keypoints, descriptor_radius);

    db_desc = describeKeypoints(database_image, database_keypoints, descriptor_radius);
    
    all_matches = matchDescriptors(query_desc, db_desc, match_lambda);
    all_matches(2,:) = 1:size(all_matches);
    all_matches = all_matches(:, all_matches(1,:) ~= 0);
    matched_query_keypoints = query_keypoints(:,all_matches(2,:));
    matched_p_W = p_W_landmarks(:,all_matches(1,:));
    
    max_num_inliers_history = zeros(1,k);
    rng(2);
    
    for i = 1:k
        
        % pick 6 random points that have a match
        
        y = datasample(all_matches, 6, 2, 'Replace', false);
        
        % estimate M for random 2D - 3D correspondences
        sampled_keypoints = query_keypoints(:,y(2,:));
        sampled_world_points = p_W_landmarks(:,y(1,:));
        M = estimatePoseDLT(sampled_keypoints', sampled_world_points',K);
        
        % calculate num inliers
        p_reproj = reprojectPoints(matched_p_W',M,K)';
        
        diff = matched_query_keypoints - p_reproj;
        
        diff_norm = sqrt(sum(diff.^2, 1));
        
        inlier_mask_tmp = zeros(1:size(matched_query_keypoints(1,:)));
        
        inlier_mask_tmp(diff_norm <= max_dist_reproj) = 1;
        
        num_inliers = sum(inlier_mask_tmp);
        
        if i == 1 || num_inliers > max_num_inliers_history(i-1)
            inlier_mask = inlier_mask_tmp;
            max_num_inliers_history(i) = num_inliers;
            R_C_W = M(:,1:3);
            t_C_W = M(:,4);
        else
            max_num_inliers_history(i) = max_num_inliers_history(i-1);
        end
        
    end
   

end
