function Point_Cloud = initializePointCloudMono( img0, img1, K )
%INITIALIZEPOINTCLOUDSTEREO Summary of this function goes here
%   Detailed explanation goes here

rng(2);

harris_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 200;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 4;

scores0 = harris(img0,harris_patch_size,harris_kappa);
kp0 = selectKeypoints(scores0,num_keypoints,nonmaximum_supression_radius);

scores1 = harris(img1,harris_patch_size,harris_kappa);
kp1 = selectKeypoints(scores1,num_keypoints,nonmaximum_supression_radius);

desc0 = describeKeypoints(img0,kp0,descriptor_radius);
desc1 = describeKeypoints(img1,kp1,descriptor_radius);

matches = matchDescriptors(desc1,desc0,match_lambda);

kp1_matched = flipud(kp1(:,matches ~= 0));
kp0_matched = flipud(kp0(:,matches(matches ~= 0)));

kp0_matched = [kp0_matched; ones(size(kp0_matched(1,:)))];
kp1_matched = [kp1_matched; ones(size(kp1_matched(1,:)))];

k = 8;
num_iterations = 1000;
max_num_inliers_history = zeros(1,num_iterations);
max_num_inliers = 0;
pixel_tolerance = 5;
inlier_mask = [];

    
for i = 1:num_iterations
    
    r = randi([1,size(kp0_matched,2)],k,1);
    
    kp0_selected = kp0_matched(:, r);
    kp1_selected = kp1_matched(:, r);
    
    E = estimateEssentialMatrix(kp0_selected, kp1_selected, K, K);
    
    [Rots,u3] = decomposeEssentialMatrix(E);
    [R1,T1] = disambiguateRelativePose(Rots,u3,kp0_selected,kp1_selected,K,K);
    Points_3d = linearTriangulation(kp0_matched,kp1_matched,K*eye(3,4),K*[R1,T1]);
    Points_3d = Points_3d(1:3,:);
    
    kp1_reprojected = reprojectPoints(Points_3d', [R1,T1], K);
    
    difference = kp1_matched(1:2,:) - kp1_reprojected';
    errors = sum(difference.^2, 1);
    is_inlier = errors < pixel_tolerance^2;
    
    if nnz(is_inlier) > max_num_inliers && nnz(is_inlier) >= k
        max_num_inliers = nnz(is_inlier);        
        inlier_mask = is_inlier;
    end
    
    max_num_inliers_history(i) = max_num_inliers;
    
end

if max_num_inliers == 0
    Point_Cloud = [];
else
    kp0_selected = kp0_matched(:,inlier_mask);
    kp1_selected = kp1_matched(:,inlier_mask);
    
    E = estimateEssentialMatrix(kp0_selected, kp1_selected, K, K);
    
    [Rots,u3] = decomposeEssentialMatrix(E);
    [R1,T1] = disambiguateRelativePose(Rots,u3,kp0_selected,kp1_selected,K,K);
    
    Point_Cloud = linearTriangulation(kp0_selected,kp1_selected,K * eye(3,4),K * [R1,T1]);
    Point_Cloud = Point_Cloud(1:3,:);
end
debug = 1;

if(debug)
    subplot(2,1,1), imshow(img0);
    hold on;
    kp1_reprojected = reprojectPoints(Point_Cloud',[R1,T1],K);
    plot(kp1_matched(1,:), kp1_matched(2,:), 'rx', 'Linewidth', 2);
    plot(kp1_reprojected(:, 1)', kp1_reprojected(:, 2)', 'bx', 'Linewidth', 2);
    
    title('Left Image (Blue matches)');
    
    
    subplot(2,1,2), scatter3(Point_Cloud(1, :), Point_Cloud(2, :), Point_Cloud(3, :), ...
        20 * ones(1, length(Point_Cloud)));
    axis equal;
    axis vis3d;
    grid off;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('3D Point matches');
end

end

