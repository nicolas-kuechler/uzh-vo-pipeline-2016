function [ U,V ] = projectPoints( X,Y,Z, K, RT, D, imageDistorted )
%PROJECTPOINTS Summary of this function goes here
%   Detailed explanation goes here

XYZworld = [X(:),Y(:),Z(:),ones(length(X(:)),1)]';
XYZcamera = RT * XYZworld;

XY = [XYZcamera(1,:) ./ XYZcamera(3,:); XYZcamera(2,:) ./ XYZcamera(3,:)];

if imageDistorted
    XY = ( 1 + D(1) * (XY(1,:).^2 + XY(2,:).^2) + D(2) * (XY(1,:).^2 + XY(2,:).^2).^2 ) .* XY;
end

resultHomog = K * [XY ; ones(1, length(XY(1,:)))];

lambdas = resultHomog(3,:);

result = resultHomog ./ lambdas;

U = result(1,:);

V = result(2,:);

end

