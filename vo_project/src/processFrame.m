function [ next_T, next_state ] = processFrame(next_img, prev_img, prev_state, K)
%PROCESSFRAME Determines pose of camera with next_image in frame of 
% camera with prev_img. Determine point_cloud <-> keypoint correspondence
% with keypoints from next image and point cloud from prev img. 
%
% inputs: next_img: N x M key frame image of camera
%         prev_img: N x M image of camera
%         prev_state: struct with the following fields:
%                     pt_cloud: 3 x H array with 3D points that have been
%                               generated so far.
%                     matched_kp: 2 x L array with 2D key points in
%                                 prev_img which have a corresponding 3D point in
%                                 pt_cloud
%                     corr2d3d: 1 x L array with entry i equal to j where 
%                               ith matched keypoint <-> jth 3D point
%                     candidate_kp: 2 x P array with 2D key points which
%                                   have been tracked from some frame before
%                                   but not matched with a 3D point
%                     kp_track_start: 2 x P starting key point for each
%                                     track
%                     kp_pose_start: 12 x P starting pose for each key
%                                    point 
%                     
%         K: calibration matrix of camera with next_img
%
% outputs: next_state: state propagated to next time frame (same structure
%                      as prev_state
%          next_T: 3D points that are in 1 to 1 correspondence
%                            with nex_keypoints and are in frame of camera 
%                            with prev_img
%          R_next: rotation of camera with next_img with respect to prev_img.
%          T_next: translation of camera with next_img with respect to
%                  prev_img.
                
pt_cloud = prev_state.pt_cloud; % pt_cloud w. r. t. last key frame
matched_kp = prev_state.matched_kp; % keypoints (matched with pt_cloud)
corr2d3d = prev_state.corr2d3d; % correspondence (entry i is j. ith matched keypoint <-> jth 3D point)
candidate_kp = prev_state.candidate_kp;
kp_track_start = prev_state.kp_track_start;
kp_pose_start = prev_state.kp_pose_start;

%%%%%% Step 1: State Propagation %%%%%%
[prop_matched_kp, point_validity] = propagateState(matched_kp, prev_img, next_img);

% remove points that were not reliably tracked from keyframe correspondence
% and from tracked keypoints
next_matched_kp = prop_matched_kp(:, point_validity);
next_corr2d3d = corr2d3d(:, point_validity);

assert(size(next_corr2d3d, 2) == size(next_matched_kp,2));

%%%%%% Step 2: Pose Estimation %%%%%%
% with new correspondence next_point_cloud <-> next_keypoints determine new
% pose with RANSAC and P3P
[next_T, inlier_mask] = ...
    ransacLocalizationSpecial(next_matched_kp, next_corr2d3d, pt_cloud, K);

% remove all outliers from ransac
next_matched_kp = next_matched_kp(:, inlier_mask);
next_corr2d3d = next_corr2d3d(:, inlier_mask);

%%%%%% Step 3: Triangulating new landmarks %%%%%%
% generate candidate points for first iteration
if isempty(candidate_kp)
    % get first candidate keypoints
    harris_patch_size = 9;
    harris_kappa = 0.08;
    num_keypoints = 100; % TODO: tune
    nonmaximum_supression_radius = 8;

    scores = harris(next_img, harris_patch_size, harris_kappa);
    new_candidate_kp = selectKeypoints(scores, num_keypoints, nonmaximum_supression_radius);

    next_candidate_kp = new_candidate_kp;
    next_kp_track_start = new_candidate_kp;
    next_kp_pose_start = repmat(next_T(:), 1, num_keypoints);
    
    next_pt_cloud = pt_cloud;
else
    [prop_candidate_kp, point_validity] = propagateState(candidate_kp, prev_img, next_img);

    % remove points which could not be tracked
    next_candidate_kp = prop_candidate_kp(:, point_validity);
    next_kp_track_start = kp_track_start(:, point_validity);
    next_kp_pose_start = kp_pose_start(:, point_validity);

    loss = sum(1 - point_validity);

    % try to triangulate points
    [new_pt_cloud, new_matched_kp, remain] = ...
        tryTriangulate(next_candidate_kp, next_kp_track_start, next_kp_pose_start, next_T, K);
    added_matched = sum(1 - remain); 
    loss = loss + sum(1 - remain);

    % remove all candidate key points that were triangulated
    next_candidate_kp = next_candidate_kp(:, remain);
    next_kp_track_start = kp_track_start(:, remain);
    next_kp_pose_start = kp_pose_start(:, remain);

    % add new 3d points and matched key points
    num_new_pts = size(new_pt_cloud, 2);
    num_old_pts = size(pt_cloud, 2);
    next_pt_cloud = [pt_cloud, new_pt_cloud];
    next_matched_kp = [next_matched_kp, new_matched_kp];
    next_corr2d3d = [next_corr2d3d, num_old_pts + 1 : num_old_pts + num_new_pts];

    % get new candidate keypoints
    harris_patch_size = 9;
    harris_kappa = 0.08;
    num_keypoints = loss + 10; % TODO: tune
    nonmaximum_supression_radius = 8;

    scores = harris(next_img, harris_patch_size, harris_kappa);
    new_candidate_kp = selectKeypoints(scores, num_keypoints, nonmaximum_supression_radius);

    % add them to existing candidate keypoints
    next_candidate_kp = [next_candidate_kp, new_candidate_kp];
    next_kp_track_start = [next_kp_track_start, new_candidate_kp];
    next_kp_pose_start = [next_kp_pose_start, repmat(next_T(:), 1, num_keypoints)];
end

% write all variables in new state                
next_state = struct('pt_cloud', next_pt_cloud, ...
                    'matched_kp', next_matched_kp, ...
                    'corr2d3d', next_corr2d3d, ...
                    'candidate_kp', next_candidate_kp, ...
                    'kp_track_start', next_kp_track_start, ...
                    'kp_pose_start', next_kp_pose_start);
                
assert(size(next_candidate_kp, 2) == size(next_kp_track_start, 2));
assert(size(next_candidate_kp, 2) == size(next_kp_pose_start, 2));
assert(size(next_corr2d3d, 2) == size(next_matched_kp, 2));

%% DEBUG: (remove after testing)
debug = true;
if debug && ~isempty(candidate_kp)
    imshow(next_img);
    hold on;
    % plot new matched keypoints
    plot(next_matched_kp(1,:), next_matched_kp(2,:), 'rx', 'Linewidth', 2);
    
    % plot old matched keypoints
    plot(matched_kp(1,:), matched_kp(2,:), 'mv', 'Linewidth', 2);
    
    % plot correspondences between old and new matched keypoints
    quiver(matched_kp(1,:),matched_kp(2,:),...
        -matched_kp(1,:)+prop_matched_kp(1,:), -matched_kp(2,:)+prop_matched_kp(2,:), 0, 'm');
    
    % plot reprojected 3D points (should coincide with new_matched_kp)
    next_keypoints_reprojected = reprojectPoints(next_pt_cloud, next_T, K);
    plot(next_keypoints_reprojected(1,:), next_keypoints_reprojected(2,:), ...
        'ro', 'Linewidth', 2);
    
    % plot new candidate keypoints
    plot(next_candidate_kp(1,:), next_candidate_kp(2,:), 'bx', 'Linewidth', 2);
    
    % plot previous candidate keypoints
    plot(candidate_kp(1,:), candidate_kp(2,:), 'cv', 'Linewidth', 2);
    
    % plot correspondences between old and new candidate keypoints
    quiver(candidate_kp(1,:),candidate_kp(2,:),...
        -candidate_kp(1,:)+prop_candidate_kp(1,:), -candidate_kp(2,:)+prop_candidate_kp(2,:), 0, 'c');
    
    % plot track start of each candidate keypoints
    plot(next_kp_track_start(1,:), next_kp_track_start(2,:), 'co', 'Linewidth', 2);
    
%     % plot correspondences between track start and old candidate keypoints
%     quiver(next_kp_track_start(1,:),next_kp_track_start(2,:),...
%         -next_kp_track_start(1,:)+candidate_kp(1,:), -next_kp_track_start(2,:)+candidate_kp(2,:), 0, 'c');
    
    title(['Red x: current matched keypoints, Magenta v: previous matched keypoints,' ...
           'Magenta o: reprojected point cloud, Blue x: current candidate keypoints,' ...
           'Cyan v: previous candidate keypoints, Cyan o: track starts']);
    
    hold off;
    struct('Matched', size(next_matched_kp, 2), ...
          'Cloud', size(next_pt_cloud, 2), ...
          'Candidates', size(next_candidate_kp, 2), ...
          'added', added_matched)
    waitforbuttonpress;
end

