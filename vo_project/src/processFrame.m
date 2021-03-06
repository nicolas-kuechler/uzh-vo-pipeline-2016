function [ R, T, curr_state] = processFrame(curr_img, prev_img, prev_state, K, params)
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
%         params: struct containing various parameters for feature
%                 detection, matching, triangulation etc.
%
% outputs: curr_state: state propagated to current time frame (same structure
%                      as prev_state
%          T: 3D points that are in 1 to 1 correspondence
%                            with curr_keypoints and are in frame of camera
%                            with prev_img
%          R: rotation of camera with next_img with respect to prev_img.

% shorten names for in code useage
pt_cloud = prev_state.pt_cloud;
curr_matched_kp = prev_state.matched_kp;
candidates_prev = prev_state.candidates;
candidates_start = prev_state.candidates_start;
candidates_start_pose = prev_state.candidates_start_pose;
curr_hidden_state = prev_state.hidden_state;
curr_observations = prev_state.observations;

%% Step 1: State Propagation
[curr_matched_kp, point_validity_m] = propagateState(curr_matched_kp, prev_img, curr_img, params);

% remove lost points
curr_matched_kp = curr_matched_kp(:, point_validity_m);
pt_cloud = pt_cloud(:,point_validity_m);

%% Step 2: Pose Estimation
% with new correspondence pt_cloud <-> curr_matched_kp determine new pose with RANSAC and P3P
[R, T, inlier_mask] = localizationRANSAC(curr_matched_kp, pt_cloud, K, params);

% remove all outliers from ransac
curr_matched_kp = curr_matched_kp(:, inlier_mask);
pt_cloud = pt_cloud(:, inlier_mask);

%% Step 3: Triangulating new landmarks
if ~isempty(candidates_prev)
    
    % Track candidate keypoints
    [candidates_prev, point_validity_c] = propagateState(candidates_prev, prev_img, curr_img, params);
    
    % Remove lost candidate keypoints
    candidates_prev = candidates_prev(:, point_validity_c);
    candidates_start = candidates_start(:, point_validity_c);
    candidates_start_pose = candidates_start_pose(:, point_validity_c);
    
    if(params.use_adaptive_angles)
        nKp = size(curr_matched_kp,2);
        if nKp < params.adaptive_triangulation_angle(1)
            triangulation_angle_threshold = params.adaptive_triangulation_angle(2);
        elseif nKp < params.adaptive_triangulation_angle(3)
            triangulation_angle_threshold = params.adaptive_triangulation_angle(4);
        elseif nKp < params.adaptive_triangulation_angle(5)
            triangulation_angle_threshold = params.adaptive_triangulation_angle(6);
        elseif nKp < params.adaptive_triangulation_angle(7)
            triangulation_angle_threshold = params.adaptive_triangulation_angle(8);
        else
            triangulation_angle_threshold = params.adaptive_triangulation_angle(9);
        end
    else
        triangulation_angle_threshold = params.triangulation_angle_threshold;
    end
    
    % Try to triangulate points
    kp_critical = size(curr_matched_kp, 2) < params.critical_kp;
    [new_pt_cloud, new_matched_kp, remain] = ...
        tryTriangulate(candidates_prev, candidates_start, candidates_start_pose, [R,T], K, ~kp_critical,  triangulation_angle_threshold, params);
    
    % Remove successfully triangulated candidates
    candidates_prev = candidates_prev(:, remain);
    candidates_start = candidates_start(:, remain);
    candidates_start_pose = candidates_start_pose(:, remain);
    
    % add new 3d points and matched key points
    pt_cloud = [pt_cloud, new_pt_cloud];
    curr_matched_kp = [curr_matched_kp, new_matched_kp];
else
    new_pt_cloud = [];
    kp_critical = true;
end

%% Update Hidden State and Observations
if params.runBA
    % update hidden state
    current_tau = HomogMatrix2twist([R', -R' * T; 0, 0, 0, 1]);
    
    curr_n = curr_observations(1);
    hidden_taus = curr_hidden_state(1 : 6 * curr_n);
    hidden_pt_cloud = curr_hidden_state(6 * curr_n + 1 : end);
    
    hidden_state = [hidden_taus, current_tau', hidden_pt_cloud, new_pt_cloud(:)'];
    
    % update observations
    ki = size(curr_matched_kp, 2);
    
    curr_m = curr_observations(2);
    new_m = curr_m + size(new_pt_cloud, 2);
    
    prev_k = size(prev_state.matched_kp, 2);
    prev_l = curr_observations(end - prev_k + 1 : end);
    
    % tracking loss
    prev_l = prev_l(point_validity_m);
    
    % ransac loss
    prev_l = prev_l(inlier_mask);
    
    % new landmarks
    l = [prev_l, curr_m + 1 : new_m];

    observations = [curr_n + 1, new_m, curr_observations(3 : end), ki, curr_matched_kp(:)', l];
else
    hidden_state = [];
    observations = [];
end

%% Establish new keypoint candidates for current frame
if  size(candidates_prev,2) <= params.candidate_cap
    
    num_keypoints =  params.add_candidate_each_frame;
    
    scores = harris(curr_img, params.harris_patch_size, params.harris_kappa);
    if kp_critical || params.surpress_existing_matches == 1
        scores = suppressExistingMatches(scores, [candidates_prev, curr_matched_kp], ...
            params.nonmaximum_supression_radius);
    end
    new_candidate_kp = selectKeypoints(scores, num_keypoints, params.nonmaximum_supression_radius);
    
    % add them to existing candidate keypoints
    candidates_prev = [candidates_prev, new_candidate_kp];
    candidates_start = [candidates_start, new_candidate_kp];
    
    candidates_start_pose = [candidates_start_pose, repmat(reshape([R,T], 12, 1), 1, size(new_candidate_kp,2))];
end

%% Write all variables to new state
curr_state = struct('pt_cloud', pt_cloud, ...
    'matched_kp', curr_matched_kp, ...
    'candidates', candidates_prev, ...
    'candidates_start', candidates_start, ...
    'candidates_start_pose', candidates_start_pose, ...
    'hidden_state', hidden_state, ...
    'observations', observations);

end

