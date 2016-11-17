% fundamentalEightPoint_normalized  The 8-point algorithm for the estimation of the fundamental matrix F
%
% The eight-point algorithm for the fundamental matrix with a posteriori
% enforcement of the singularity constraint (det(F)=0).
% Include data normalization.
%
% Reference: "Multiple View Geometry" (Hartley & Zisserman 2000), Sect. 10.1 page 262.
%
% Input: point correspondences
%  - p1(3,N): homogeneous coordinates of 2-D points in image 1
%  - p2(3,N): homogeneous coordinates of 2-D points in image 2
%
% Output:
%  - F(3,3) : fundamental matrix


function F = fundamentalEightPoint_normalized(p1, p2)
    
    [norm_p1, T1] = normalise2dpts(p1);
    [norm_p2, T2] = normalise2dpts(p2);
    
    F_tilde = fundamentalEightPoint(norm_p1, norm_p2);
    
    F = T2' * F_tilde * T1;
    
end
