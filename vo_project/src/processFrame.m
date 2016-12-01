function [ curr_state, T ] = processFrame(curr_img, prev_img, prev_state)

% Propagate Keypoint Landmark correspondence from prev_img to curr_img
% (Propagate State)

pointTracker = vision.PointTracker;
initialize(pointTracker,prev_state.keypoints,prev_img);
[points,point_validity] = step(pointTracker,curr_img);

[R_C_W, t_C_W, query_keypoints, all_matches, inlier_mask, max_num_inliers_history] = ransacLocalization(curr_img, prev_img, prev_state.keypoints, prev_state.p_W_landmarks, prev_state.K)


% Estimate current camera pose T based on new state




end

