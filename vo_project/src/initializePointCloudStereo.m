function [points, desc ] = initializePointCloudStereo(left_img, right_img ,K)


debug = true;

harris_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 200;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 4;
baseline = 0.54; %Kitti Dataset

left_scores = harris(left_img, harris_patch_size, harris_kappa);
left_kp = selectKeypoints(left_scores, num_keypoints, nonmaximum_supression_radius);
left_desc = describeKeypoints(left_img, left_kp, descriptor_radius);

right_scores = harris(right_img, harris_patch_size, harris_kappa);
right_kp = selectKeypoints(right_scores, num_keypoints, nonmaximum_supression_radius);
right_desc = describeKeypoints(right_img, right_kp, descriptor_radius);

matches = matchDescriptors(right_desc, left_desc, match_lambda);

left_kp_matched = left_kp(:,matches(matches ~= 0));
left_desc_matched = left_desc(:,matches(matches ~= 0));
right_kp_matched = right_kp(:,matches ~= 0);

% Build a disparity image from the harris keypoint matching directly
disp_img = zeros(size(left_img));
lin_index = sub2ind(size(disp_img),left_kp_matched(1,:),left_kp_matched(2,:));
disp_img(lin_index)=left_kp_matched(2,:) - right_kp_matched(2,:);

% Use the disparity image to build a point cloud
[points, intensities] = disparityToPointCloud(disp_img, K, baseline, left_img);

%Convert from Camera Frame to World Frame (Copied from ex03_stereo)
p_F_points = [0 -1 0; 0 0 -1; 1 0 0]^-1 * points;


points = p_F_points;
desc = left_desc_matched;

% Only to visualize the 3D points
if(debug)
    subplot(3,1,1), imshow(left_img);
    hold on;
    plot(left_kp(2, :), left_kp(1, :), 'rx', 'Linewidth', 2);
    plot(left_kp_matched(2, :), left_kp_matched(1, :), 'bx', 'Linewidth', 2);
    title('Left Image (Blue matches)');
    
    subplot(3,1,2), imshow(right_img);
    hold on;
    plot(right_kp(2, :), right_kp(1, :), 'yx', 'Linewidth', 2);
    plot(right_kp_matched(2, :), right_kp_matched(1, :), 'gx', 'Linewidth', 2);
    title('Right Image (Green matches)');
    
    subplot(3,1,3), scatter3(points(1, :), points(2, :), points(3, :), ...
        20 * ones(1, length(points)), ...
        repmat(single(intensities)'/255, [1 3]), 'filled');
    axis equal;
    axis vis3d;
    grid off;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('3D Point matches');
end
end

