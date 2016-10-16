function [ discretizedPixelCoordinates ] = projectPointsWithDistortion(K, RT, X, Y, Z, k1, k2)
%PROJECTPOINTS Summary of this function goes here
%   Detailed explanation goes here
xyz = [X(:) Y(:) Z(:) ones(length(X(:)),1)]';

%Map the world point Pw to the camera frame
result = RT*xyz;

%Project the point to the image plane to get the normalized coordinates
z = result(3,:);
result = result ./z;
result = result(1:2,:);

%Apply lens distortion to get the distorted normalized coordinates
r = sqrt(result(1,:).^2 + result(2,:).^2);
distortedNormalizedCoordinates = (1+k1 .* r.^2 + k2.*r.^4) .*result;

%Convert the distorted normalized coordinates to get the discretized pixel coordinates
discretizedPixelCoordinates = K*[distortedNormalizedCoordinates; ones(length(distortedNormalizedCoordinates(1,:)),1)'];
lambdas = discretizedPixelCoordinates(3,:);
discretizedPixelCoordinates = discretizedPixelCoordinates./lambdas;

end

