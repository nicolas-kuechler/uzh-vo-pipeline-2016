function [ Q ] = buildQ( DetectedCorners, PWCorners, Kinv )
%BUILDQ Summary of this function goes here
%   Detailed explanation goes here

n = size(DetectedCorners, 2);
h = 1;

XYZ1 = [PWCorners ones(n,1)];

%DetectedCorners = [DetectedCorners(1:2:end); DetectedCorners(2:2:end)];
uv1 = [DetectedCorners; ones([1,n])];

xy1 = (Kinv * uv1)';

for k =1:n
    Q(h,:) = [XYZ1(k,:) zeros(1,4) -xy1(k,1)*XYZ1(k,:)];
    h = h+1;
    Q(h,:) = [zeros(1,4) XYZ1(k,:) -xy1(k,2)*XYZ1(k,:)];
    h = h+1;
    
end

