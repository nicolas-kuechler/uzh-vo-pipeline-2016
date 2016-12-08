function [ scores ] = suppressExistingMatches( scores, keypoints, ...
        r )
%SUPPRESSEXISTINGMATCHES Summary of this function goes here
%   Detailed explanation goes here
% TODO: make faster
[x,y] = size(scores);

temp_scores = padarray(scores, [r r]);
keypoints  = keypoints + r;
for kp = keypoints
    temp_scores(kp(2) - r:kp(2) + r, kp(1) - r:kp(1) + r) = 0;
end

scores = temp_scores(r + 1 : x + r, r + 1 : y + r);

