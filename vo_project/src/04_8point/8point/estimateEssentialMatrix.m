function E = estimateEssentialMatrix(p1, p2, K1, K2)
% estimateEssentialMatrix_normalized: estimates the essential matrix
% given matching point coordinates, and the camera calibration K
%
% Input: point correspondences
%  - p1(2,N): coordinates [x; y] of 2-D points in image 1
%  - p2(2,N): coordinates of 2-D points in image 2
%  - K1(3,3): calibration matrix of camera 1
%  - K2(3,3): calibration matrix of camera 2
%
% Output:
%  - E(3,3) : fundamental matrix
%
% get homogeneous coordinates
p1 = [p1; ones(1, size(p1, 2))];
p2 = [p2; ones(1, size(p2, 2))];

F = fundamentalEightPoint_normalized(p1, p2);

% Compute the essential matrix from the fundamental matrix given K
E = K2'*F*K1;

end
