function [next_kp, point_validity] = propagateState(kp, prev_img, next_img)
%PROPAGATESTATE Summary of this function goes here
%   Detailed explanation goes here
% create pointTracker
tic
pointTracker = vision.PointTracker;

% initialize point tracker
initialize(pointTracker, kp', prev_img);
toc

tic
% track points to next frame
[next_kp, point_validity] = step(pointTracker, next_img);
next_kp = next_kp';
toc

assert(size(kp, 2) == size(next_kp, 2));

end

