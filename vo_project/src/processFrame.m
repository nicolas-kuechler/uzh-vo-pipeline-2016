function [ curr_T, curr_state ] = processFrame(curr_img, prev_img, prev_state, K, params)
%PROCESSFRAME Determines pose of camera with next_image in frame of 
% camera with prev_img. Determine point_cloud <-> keypoint correspondence
% with keypoints from next image and point cloud from prev img. 
%
% inputs: curr_img: N x M key frame image of camera
%         prev_img: N x M image of camera
%         prev_state: struct with the following fields:
%                     pt_cloud: 3 x L array with 3D points that have been
%                               generated so far.
%                     matched_kp: 2 x L array with 2D key points in
%                                 prev_img which have a corresponding 3D point in
%                                 pt_cloud
%                     candidate_kp: 2 x P array with 2D key points which
%                                   have been tracked from some frame before
%                                   but not matched with a 3D point
%                     kp_track_start: 2 x P starting key point for each
%                                     track
%                     kp_pose_start: 12 x P starting pose for each key
%                                    point 
%                     
%         K: calibration matrix of camera with curr_img
%
% outputs: curr_state: state propagated to current time frame (same structure
%                      as prev_state
%          curr_T: 3D points that are in 1 to 1 correspondence
%                            with curr_keypoints and are in frame of camera 
%                            with prev_img
%          R_next: rotation of camera with next_img with respect to prev_img.
%          T_next: translation of camera with next_img with respect to
%                  prev_img.
                
pt_cloud = prev_state.pt_cloud; % pt_cloud w. r. t. last key frame
curr_matched_kp = prev_state.matched_kp; % keypoints (matched with pt_cloud)
candidate_kp = prev_state.candidate_kp;
kp_track_start = prev_state.kp_track_start;
kp_pose_start = prev_state.kp_pose_start;

%% Step 1: State Propagation
[curr_matched_kp, point_validity] = propagateState(curr_matched_kp, prev_img, curr_img);

% remove lost points
curr_matched_kp = curr_matched_kp(:, point_validity);
pt_cloud = pt_cloud(:,point_validity);

%% Step 2: Pose Estimation
% with new correspondence pt_cloud <-> curr_matched_kp determine new pose with RANSAC and P3P
[curr_T, inlier_mask] = ransacLocalizationSpecial(curr_matched_kp, pt_cloud, K);

% remove all outliers from ransac
curr_matched_kp = curr_matched_kp(:, inlier_mask);
pt_cloud = pt_cloud(:, inlier_mask);

%% Step 3: Triangulating new landmarks
loss = 0;

if ~isempty(candidate_kp)
    
    % Track candidate keypoints
    [candidate_kp, point_validity] = propagateState(candidate_kp, prev_img, curr_img);
    
    % Remove lost candidate keypoints
    candidate_kp = candidate_kp(:, point_validity);
    kp_track_start = kp_track_start(:, point_validity);
    kp_pose_start = kp_pose_start(:, point_validity);
    tracking_loss = sum(1 - point_validity);
    
    % Try to triangulate points (with triangulation check if possible)
    [new_pt_cloud, new_matched_kp, remain] = ...
        tryTriangulate(candidate_kp, kp_track_start, kp_pose_start, curr_T, K);
    triangulation_loss = sum(1 - remain);
    
    %%%Move successfully triangulated candidates to the matched keypoints with their  
    % Remove successfully triangulated candidates
    candidate_kp = candidate_kp(:, remain);
    kp_track_start = kp_track_start(:, remain);
    kp_pose_start = kp_pose_start(:, remain);

    % add new 3d points and matched key points
    pt_cloud = [pt_cloud, new_pt_cloud];
    curr_matched_kp = [curr_matched_kp, new_matched_kp];
    
    loss = triangulation_loss + tracking_loss;
end

%% Establish new keypoint candidates for current frame

    % TODO Check wheter good idea to select the number of keypoints
    % as a function of the currently tracked number of keypoints
    num_keypoints =  loss + 200 - size(curr_matched_kp,2); % TODO Tune
    
    scores = harris(curr_img, params.harris_patch_size, params.harris_kappa);
    new_candidate_kp = selectKeypoints(scores, num_keypoints, params.nonmaximum_supression_radius);

    % add them to existing candidate keypoints
    candidate_kp = [candidate_kp, new_candidate_kp];
    kp_track_start = [kp_track_start, new_candidate_kp];
    kp_pose_start = [kp_pose_start, repmat(curr_T(:), 1, num_keypoints)];


%% Write all variables to new state            
curr_state = struct('pt_cloud', pt_cloud, ...
                    'matched_kp', curr_matched_kp, ...
                    'candidate_kp', candidate_kp, ...
                    'kp_track_start', kp_track_start, ...
                    'kp_pose_start', kp_pose_start);
                
assert(size(candidate_kp, 2) == size(kp_track_start, 2));
assert(size(candidate_kp, 2) == size(kp_pose_start, 2));

%% DEBUG: (remove after testing)
debug = true;
if debug && ~isempty(candidate_kp)
    subplot(2,2,[3 4]);
    imshow(curr_img);
    hold on;
    % plot new matched keypoints
    plot(curr_matched_kp(1,:), curr_matched_kp(2,:), 'rx', 'Linewidth', 2);
    
%     % plot old matched keypoints
%     plot(matched_kp(1,:), matched_kp(2,:), 'mv', 'Linewidth', 2);
%     
%     % plot correspondences between old and new matched keypoints
%     quiver(matched_kp(1,:),matched_kp(2,:),...
%         -matched_kp(1,:)+prop_matched_kp(1,:), -matched_kp(2,:)+prop_matched_kp(2,:), 0, 'm');
%     
     % plot reprojected 3D points (should coincide with new_matched_kp)
    next_keypoints_reprojected = reprojectPoints(pt_cloud, curr_T, K);
    plot(next_keypoints_reprojected(1,:), next_keypoints_reprojected(2,:), ...
         'bx', 'Linewidth', 2);
    rms_repr_error = sqrt(sum((next_keypoints_reprojected(:) - curr_matched_kp(:)).^2) / size(curr_matched_kp, 2));
    text(10,10, ['RMS Reprojection error = ' num2str(rms_repr_error)], 'Color', [0,1,0]); 
     
%     % plot new candidate keypoints
%     plot(next_candidate_kp(1,:), next_candidate_kp(2,:), 'bx', 'Linewidth', 2);
%     
%     % plot previous candidate keypoints
%     plot(candidate_kp(1,:), candidate_kp(2,:), 'cv', 'Linewidth', 2);
    
%     % plot correspondences between old and new candidate keypoints
%     quiver(candidate_kp(1,:),candidate_kp(2,:),...
%         -candidate_kp(1,:)+prop_candidate_kp(1,:), -candidate_kp(2,:)+prop_candidate_kp(2,:), 0, 'c');
%     
%     % plot track start of each candidate keypoints
%     plot(next_kp_track_start(1,:), next_kp_track_start(2,:), 'go', 'Linewidth', 2);
%     
%     % plot correspondences between track start and old candidate keypoints
%     quiver(next_kp_track_start(1,:),next_kp_track_start(2,:),...
%         -next_kp_track_start(1,:)+candidate_kp(1,:), -next_kp_track_start(2,:)+candidate_kp(2,:), 0, 'c');
%
%      % plot correspondences between track start and old candidate keypoints
%      quiver(next_kp_track_start(1,:),next_kp_track_start(2,:),...
%          -next_kp_track_start(1,:)+next_candidate_kp(1,:), -next_kp_track_start(2,:)+next_candidate_kp(2,:), 0, 'c');
%     
    legend('current matched keypoints','reprojected 3D points','Location','NorthOutside')
    title(['Red x: current matched keypoints, Magenta v: previous matched keypoints,' ...
           'Magenta o: reprojected point cloud, Blue x: current candidate keypoints,' ...
           'Cyan v: previous candidate keypoints, Cyan o: track starts']);
       
       
    hold off;
    
    struct('Matched', size(curr_matched_kp, 2), ...
          'Cloud', size(pt_cloud, 2), ...
          'Candidates', size(candidate_kp, 2), ...
          'Candidates_added', num_keypoints)
    pause(0.01);
   % waitforbuttonpress;
end

