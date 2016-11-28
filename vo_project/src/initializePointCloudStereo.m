function [repr_error, points, desc ] = initializePointCloudStereo(left_img, right_img ,K)


harris_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 200;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 4;

patch_radius = 5;
min_disp = 5;
max_disp = 50;
baseline = 0.54; %Kitti Dataset
harris_patch_size = 9;


cameraParams = cameraParameters('IntrinsicMatrix',K, 'WorldUnits', 'm');
R = eye(3);
T = [baseline; 0; 0];

stereoParams = stereoParameters(cameraParams,cameraParams,R,T);



left_scores = harris(left_img, harris_patch_size, harris_kappa);
left_kp = selectKeypoints(left_scores, num_keypoints, nonmaximum_supression_radius);
left_desc = describeKeypoints(left_img, left_kp, descriptor_radius);

right_scores = harris(right_img, harris_patch_size, harris_kappa);
right_kp = selectKeypoints(right_scores, num_keypoints, nonmaximum_supression_radius);
right_desc = describeKeypoints(right_img, right_kp, descriptor_radius);

matches = matchDescriptors(right_desc, left_desc, match_lambda);
% Returns a 1xQ matrix where the i-th coefficient is the index of the
% left_descriptor which matches to the i-th right_descriptor.

left_kp_matched = left_kp(:,matches(matches ~= 0));
left_desc_matched = left_desc(matches ~= 0);
right_kp_matched = right_kp(:,matches ~= 0);
right_desc_matched = right_desc(matches(matches ~= 0));

[points, repr_error] = triangulate(flipud(left_kp_matched)', flipud(right_kp_matched)', stereoParams);

imshow(left_img);
hold on;
plot(left_kp(2, :), left_kp(1, :), 'rx', 'Linewidth', 2);
plot(left_kp_matched(2, :), left_kp_matched(1, :), 'bx', 'Linewidth', 2);

figure
imshow(right_img);
hold on;
plot(right_kp(2, :), right_kp(1, :), 'gx', 'Linewidth', 2);
plot(right_kp_matched(2, :), right_kp_matched(1, :), 'yx', 'Linewidth', 2);

figure
plot3(points(:,1), points(:,2),points(:,3), 'o');
end

