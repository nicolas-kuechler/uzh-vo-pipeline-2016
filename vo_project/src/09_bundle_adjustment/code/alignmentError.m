function [ e ] = alignmentError( x, p_W_GT, p_W_estimate )
%ALIGNMENTERROR Summary of this function goes here
%   Detailed explanation goes here

% determine transformation
T = twist2HomogMatrix(x(1:6, 1));
R = T(1:3, 1:3);
t = T(1:3, 4);
s = x(7);

transformed_poses = s * R * p_W_estimate + t;

e = transformed_poses(:) - p_W_GT(:);
end

