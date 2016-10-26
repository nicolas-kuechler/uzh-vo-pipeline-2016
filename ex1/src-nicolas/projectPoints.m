function [ result ] = projectPoints(K, RT, X, Y, Z)
%PROJECTPOINTS Summary of this function goes here
%   Detailed explanation goes here

KRT = K*RT;
xyz = [X(:) Y(:) Z(:) ones(length(X(:)),1)]';
result = KRT*xyz;
lambdas = result(3,:);
result = result ./lambdas;
result = result(1:2, :);
end

