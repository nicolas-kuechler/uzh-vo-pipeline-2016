function [next_kp, point_validity] = propagateState(kp, prev_img, next_img)
%PROPAGATESTATE Summary of this function goes here
%   Detailed explanation goes here
% create pointTracker
pointTracker = vision.PointTracker;

% initialize point tracker
initialize(pointTracker, kp', prev_img);

% track points to next frame
[next_kp, point_validity] = step(pointTracker, next_img);
next_kp = next_kp';

assert(size(matched_kp, 2) == size(next_keypoints, 2));

end

