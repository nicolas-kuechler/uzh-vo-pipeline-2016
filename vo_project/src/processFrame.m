function [ next_keypoints, next_point_cloud, R_next, T_next ] = ...
    processFrame(next_img, prev_img, prev_keypoints, prev_point_cloud, K)
%PROCESSFRAME Determines pose of camera with next_image in frame of 
% camera with prev_img. Determine point_cloud <-> keypoint correspondence
% with keypoints from next image and point cloud from prev img. 
%
% inputs: next_img: N x M key frame image of camera
%         prev_img: N x M image of camera
%         prev_keypoints: 2 x L array of keypoints in prev_img which are in 1 to 1 
%                         correspondence with prev_point_cloud
%         prev_point_cloud: 3 x L array of 3D points in frame of camera
%                           with prev_img 
%         K: calibration matrix of camera with next_img
%
% outputs: next_keypoints: keypoints that are matched between next_img and
%                          prev_keypoints
%          next_point_cloud: 3D points that are in 1 to 1 correspondence
%                            with nex_keypoints and are in frame of camera 
%                            with prev_img
%          R_next: rotation of camera with next_img with respect to prev_img.
%          T_next: translation of camera with next_img with respect to
%                  prev_img.

% create pointTracker
pointTracker = vision.PointTracker;

% initialize point tracker
initialize(pointTracker, prev_keypoints', prev_img);

% track point to next frame
[next_keypoints, point_validity] = step(pointTracker,next_img);
next_keypoints = next_keypoints';

% remove points that were not reliably tracked
next_point_cloud = prev_point_cloud(:, point_validity);
next_keypoints = next_keypoints(:, point_validity);
prev_keypoints_updated = prev_keypoints(:, point_validity);

% with new correspondence next_point_cloud <-> next_keypoints determine new
% pose with RANSAC and P3P
[R_next, T_next, ~, matches, inlier_mask, ~] = ransacLocalization(next_img, ...
    prev_img, prev_keypoints_updated, next_point_cloud, K);

% remove all non-matched key points from point cloud correspondence next_point_cloud <-> 
% next_keypoints (unique removes repetitions in matches)
matches(matches == 0) = [];
next_point_cloud = next_point_cloud(:, unique(matches));
next_keypoints = next_keypoints(:, unique(matches));

% remove all outliers from ransac
next_point_cloud = next_point_cloud(:, inlier_mask);
next_keypoints = next_keypoints(:, inlier_mask);


%% DEBUG: (remove after testing)
debug = true;
if debug
    figure;
    imshow(next_img);
    hold on;
    plot(next_keypoints(1,:), next_keypoints(2,:), 'rx', 'Linewidth', 2);
    % plot reprojected
    next_keypoints_reprojected = reprojectPoints(next_point_cloud, [R_next, T_next], K);
    plot(next_keypoints_reprojected(1,:), next_keypoints_reprojected(2,:), ...
        'bx', 'Linewidth', 2);
    
    title('Next Image (Red matches: Inlier keypoints, blue matches: reprojected point cloud.');
end

