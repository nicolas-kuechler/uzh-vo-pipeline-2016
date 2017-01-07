function [next_kp, point_validity] = propagateState(kp, prev_img, next_img, params, bder)
%PROPAGATESTATE Summary of this function goes here
%   Detailed explanation goes here
% create pointTracker

bs = params.tracker_blocksize;

pointTracker = vision.PointTracker( ...
    'MaxBidirectionalError', bder, ... 
    'BlockSize', [bs,bs]);

% initialize point tracker
initialize(pointTracker, kp', prev_img);

% track points to next frame
[next_kp, point_validity] = step(pointTracker, next_img);
next_kp = next_kp';

assert(size(kp, 2) == size(next_kp, 2));

end

