function [R_C_W, t_C_W, query_keypoints, all_matches, inlier_mask, ...
    max_num_inliers_history] = ransacLocalization(...
    query_image, database_image, database_keypoints, p_W_landmarks, K)
% query_keypoints should be 2x1000
% all_matches should be 1x1000 and correspond to the output from the
%   matchDescriptors() function from exercise 3.
% inlier_mask should be 1xnum_matched (!!!) and contain, only for the
%   matched keypoints (!!!), 0 if the match is an outlier, 1 otherwise.

%Parameters
harris_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 1000;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 5;


k=2000;

query_scores = harris(query_image, harris_patch_size, harris_kappa);
query_keypoints = selectKeypoints(query_scores, num_keypoints, nonmaximum_supression_radius);
query_desc = describeKeypoints(query_image, query_keypoints, descriptor_radius);

db_desc = describeKeypoints(database_image, database_keypoints, descriptor_radius);


all_matches = matchDescriptors(query_desc, db_desc, match_lambda);
all_matches_red = [all_matches;1:length(all_matches)];
all_matches_red = all_matches_red(:,all_matches_red(1,:)~= 0);
rng(2);
s = 6;
    max_dist_threshold = 10; %pixels
    p = query_keypoints(:, all_matches_red(2,:));
    p_W = p_W_landmarks(:, all_matches_red(1,:));
    
    max_num_inliers_history = zeros(1,length(all_matches_red));
    max_num_inliers=0;
    max_inlier_mask = zeros(1,length(all_matches_red));
    ind = 1:length(all_matches_red);
for i = 1:k
    % 1st pick 6 random pts of all_matches
    Y = datasample(all_matches_red,s,2,'Replace',false);
    
    p_rand =query_keypoints(:, Y(2,:));
    p_W_rand = p_W_landmarks(:, Y(1,:));
    
    
    
    % 2nd solve DLT
    M_rand = estimatePoseDLT(p_rand',p_W_rand',K);
    
    % 3rd error (with max)
    %p_reproj = reprojectPoints(p_W_rand',M_rand,K);
    p_reproj = reprojectPoints(p_W',M_rand,K);
    
    %TODO Check here if the right points are compared to the reprojection 
    rms = abs(p-p_reproj');
    diff_norm =sqrt(sum(rms.^2)); %calc norm columnwise
    
    % 4th calc inlier
    inlier_mask = zeros(1,length(all_matches_red));
    inlier_mask(ind(diff_norm<=max_dist_threshold)) = 1;

    num_inliers = sum(inlier_mask);
    
    if num_inliers > max_num_inliers
        max_num_inliers = num_inliers; 
        max_inlier_mask = inlier_mask;
        R_C_W = M_rand(:,1:3);
        t_C_W = M_rand(:,4);
    end
    
    max_num_inliers_history(i) = max_num_inliers;
    
end

    inlier_mask = max_inlier_mask;

end
