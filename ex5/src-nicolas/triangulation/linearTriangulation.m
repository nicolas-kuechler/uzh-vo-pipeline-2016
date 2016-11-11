% LINEARTRIANGULATION  Linear Triangulation
%
% Input:
%  - p1(3,N): homogeneous coordinates of points in image 1
%  - p2(3,N): homogeneous coordinates of points in image 2
%  - M1(3,4): projection matrix corresponding to first image
%  - M2(3,4): projection matrix corresponding to second image
%
% Output:
%  - P(4,N): homogeneous coordinates of 3-D points

function P = linearTriangulation(p1,p2,M1,M2)

N = size(p1, 2);
P_homog = zeros(4,N);
for i=1:N
    cross_p1 = createCrossProdMat(p1(:,i));
    cross_p2 = createCrossProdMat(p2(:,i));
    A = [cross_p1 * M1; cross_p2 * M2];
    [~,~,V] = svd(A);
    P_homog(:,i) = V(:,end);
end
    P = P_homog ./ P_homog(4,:);
end


