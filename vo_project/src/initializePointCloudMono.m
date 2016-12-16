function [R1, T1, repr_error, Point_Cloud, kp0_selected, kp1_selected] = initializePointCloudMono( img0, img1, K , params)
%INITIALIZEPOINTCLOUDMONO Determines pose of camera with img1 in frame of 
% camera with img0. Also determines point cloud of features in left image 
% in left image frame. 

% inputs: img0: N x M image of left camera
%         img1: N x M image of right camera
%         K: calibration matrix of both cameras
%
% outputs: R1: rotation of camera 1 in frame of camera 0
%          T1: translation of camera 1 in frame of camera 0
%          repr_error: average reprojection error of reprojected point cloud into
%                      right image.
%          Point_Cloud: [3xL] array of 3d points triangulated from features
%                       in left image in left image frame.
%          kp0_selected: [2xL] array of 2d key points in left image that are
%                        selected by RANSAC
%          kp1_selected: [2xL] array of 2d key points in right image that are
%                        selected by RANSAC

% extract features and descriptors from left image
scores0 = harris(img0, params.harris_patch_size, params.harris_kappa);
kp0 = selectKeypoints(scores0, params.num_keypoints, params.nonmaximum_supression_radius);
desc0 = describeKeypoints(img0, kp0, params.descriptor_radius);

% extract features and descriptors from left image
scores1 = harris(img1, params.harris_patch_size, params.harris_kappa);
kp1 = selectKeypoints(scores1, params.num_keypoints, params.nonmaximum_supression_radius);
desc1 = describeKeypoints(img1, kp1, params.descriptor_radius);

% match key points from left (database) to right image (query)
% and remove unmatched points
matches = matchDescriptors(desc1, desc0, params.match_lambda);
kp1_matched = kp1(:, matches ~= 0);
kp0_matched = kp0(:, matches(matches ~= 0));

% perform RANSAC to find best R and T through 8pt algorithm
k = 8;
num_iterations = 1200;
max_num_inliers = 0;
pixel_tolerance = 3;
inlier_mask = [];
    
for i = 1:num_iterations
    % randomly sample k key points
    [kp0_selected, idx] = datasample(kp0_matched, k, 2, 'Replace', false);
    
    % find corresponding right key points and put them in same order as
    % left selected keypoints
    kp1_selected = kp1_matched(:, idx);
    
    % estimate R and T from left to right image frame through essential
    % matrix
    E = estimateEssentialMatrix(kp0_selected, kp1_selected, K, K);
    [Rots, u3] = decomposeEssentialMatrix(E);
    [R1, T1] = disambiguateRelativePose(Rots, u3, kp0_selected, kp1_selected, K, K);
    
    % Find 3d point cloud in left camera frame through linear triangulation
    Points_3d = linearTriangulation(kp0_matched, kp1_matched, ...
        K * eye(3,4), K * [R1, T1]);
    
    % reproject them into the right image
    kp1_reprojected = reprojectPoints(Points_3d, [R1, T1], K);
    
    % find the inliers in the right image
    difference = kp1_matched - kp1_reprojected;
    errors = sum(difference.^2, 1);
    is_inlier = errors < pixel_tolerance^2;
    
    % update inlier mask if sample with higher inlier count is found
    if nnz(is_inlier) > max_num_inliers
        max_num_inliers = nnz(is_inlier);        
        inlier_mask = is_inlier;
    end
end

% With inlier mask with highest inlier count do 8point algorithm on all
% inliers
if max_num_inliers == 0
    Point_Cloud = [];
else
    % select inlier key points from left and right
    kp0_selected = kp0_matched(:, inlier_mask);
    kp1_selected = kp1_matched(:, inlier_mask);
    
    % estimate essential matrix to determine R and T of right image
    E = estimateEssentialMatrix(kp0_selected, kp1_selected, K, K);
    [Rots, u3] = decomposeEssentialMatrix(E);
    [R1, T1] = disambiguateRelativePose(Rots, u3, kp0_selected, ...
        kp1_selected, K, K);
    
    % find the point cloud in left camera frame
    Point_Cloud = linearTriangulation(kp0_selected, kp1_selected, ...
        K * eye(3,4),K * [R1, T1]);

    in_front_of_camera = Point_Cloud(3,:) > 0;
    Point_Cloud = Point_Cloud(:,in_front_of_camera);
    kp0_selected = kp0_selected(:,in_front_of_camera);
    kp1_selected = kp1_selected(:,in_front_of_camera);
    
    % get reprojection error of point cloud into right image
    kp1_reprojected = reprojectPoints(Point_Cloud,[R1, T1],K);
    difference = kp1_selected - kp1_reprojected;
    repr_error = sum(difference(:).^2) / size(difference, 2);
end

%% DEBUG: (delete after testing)
debug = false;

if(debug)
    imshow(img0);
    hold on;
    plot(kp1_matched(1,:), kp1_matched(2,:), 'rx', 'Linewidth', 2);
    plot(kp1_reprojected(1,:), kp1_reprojected(2,:), 'bx', 'Linewidth', 2);
    
    title('Left Image (Red matches: Inlier keypoints, blue matches: reprojected point cloud.');
end

end

