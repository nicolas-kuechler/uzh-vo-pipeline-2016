function [ next_T_c, next_state ] = processFrame(next_img, prev_img, prev_state, K)
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

% TODO: correspondences are between pt_cloud_kf and keypoints_c (do not
% remove point clouds, make surjections mapping matrix)

T_kf = prev_state.T_kf; % pose of last key frame
pt_cloud_kf = prev_state.pt_cloud_kf; % pt_cloud w. r. t. last key frame
T_c = prev_state.T_c; % current pose
keypoints_c = prev_state.keypoints_c; % previous keypoints (tracked)
correspondence_c = prev_state.correspondence; % correspondence (entry i is j. ith tracked keypoint <-> jth 3D point)

% create pointTracker
pointTracker = vision.PointTracker;

% initialize point tracker
initialize(pointTracker, keypoints_c', prev_img);

% track points to next frame
[next_keypoints, point_validity] = step(pointTracker, next_img);
next_keypoints = next_keypoints';

assert(size(keypoints_c, 2) == size(next_keypoints, 2));

% remove points that were not reliably tracked from keyframe correspondence
% and from tracked keypoints
next_keypoints_c = next_keypoints(:, point_validity);
next_correspondence_c = correspondence_c(:, point_validity);

assert(size(next_correspondence_c, 2) == size(next_keypoints_c,2));

% with new correspondence next_point_cloud <-> next_keypoints determine new
% pose with RANSAC and P3P
[R_next, T_next, inlier_mask] = ...
    ransacLocalizationSpecial(next_keypoints_c, next_correspondence_c, pt_cloud_kf, K);

% update current pose
next_T_c = [R_next T_next];

% remove all outliers from ransac
next_keypoints_c = next_keypoints_c(:, inlier_mask);
next_correspondence_c = next_correspondence_c(:, inlier_mask);

% write all variables in new state
next_state = struct('T_kf', T_kf, ...
                    'pt_cloud_kf', pt_cloud_kf, ...
                    'T_c', next_T_c, ...
                    'keypoints_c', next_keypoints_c,...
                    'correspondence', next_correspondence_c);

%% DEBUG: (remove after testing)
debug = true;
if debug
    figure;
    imshow(next_img);
    hold on;
    % plot new keypoints
    plot(next_keypoints_c(1,:), next_keypoints_c(2,:), 'rx', 'Linewidth', 2);
    
    % plot old keypoints
    plot(keypoints_c(1,:), keypoints_c(2,:), 'yx', 'Linewidth', 2);
    
    % plot correspondences
    quiver(keypoints_c(1,:),keypoints_c(2,:),...
        -keypoints_c(1,:)+next_keypoints(1,:), -keypoints_c(2,:)+next_keypoints(2,:), 0);
    
    % plot reprojected
    next_keypoints_reprojected = reprojectPoints(pt_cloud_kf, next_T_c, K);
    plot(next_keypoints_reprojected(1,:), next_keypoints_reprojected(2,:), ...
        'bx', 'Linewidth', 2);
    
    title('Next Image (Red matches: tracked inlier keypoints, blue matches: reprojected point cloud.');
end

