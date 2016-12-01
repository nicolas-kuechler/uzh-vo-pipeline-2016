function p_reproj = reprojectPoints(P, M, K)
% Reproject 3D points given a projection matrix
%
% P: [3xn] coordinates of the 3d points in the world frame
% M: [3x4] projection (includes only [R T])  matrix
% K: [3x3] camera matrix
%
% p_reproj: [2xn] coordinates of the reprojected 2d points
P = P';
p_homo = (K*M*[P';ones(1,length(P))])';
p_homo(:,1) = p_homo(:,1) ./ p_homo(:,3);
p_homo(:,2) = p_homo(:,2) ./ p_homo(:,3);

p_reproj = p_homo(:,1:2)';

end

