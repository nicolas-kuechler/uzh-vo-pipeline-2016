function [ p_reprojected ] = reprojectPoints( P, M, K )
%REPROJECTPOINTS Summary of this function goes here
%   Detailed explanation goes here

P = [P ones([size(P(:,1)), 1])]';
p_reprojected_homog = K * M * P;

lambdas = p_reprojected_homog(3,:);

p_reprojected = p_reprojected_homog ./ lambdas;

end

