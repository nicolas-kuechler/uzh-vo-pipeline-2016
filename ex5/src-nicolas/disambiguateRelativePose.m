% DISAMBIGUATERELATIVEPOSE- finds the correct relative camera pose (among
% four possible configurations) by returning the one that yields points
% lying in front of the image plane (with positive depth).
%
% Arguments:
%   Rots -  3x3x2: the two possible rotations returned by decomposeEssentialMatrix
%   u3   -  a 3x1 vector with the translation information returned by decomposeEssentialMatrix
%   p1   -  3xN homogeneous coordinates of point correspondences in image 1
%   p2   -  3xN homogeneous coordinates of point correspondences in image 2
%   K1   -  3x3 calibration matrix for camera 1
%   K2   -  3x3 calibration matrix for camera 2
%
% Returns:
%   R -  3x3 the correct rotation matrix
%   T -  3x1 the correct translation vector
%
%   where [R|t] = T_C2_W = T_C2_C1 is a transformation that maps points
%   from the world coordinate system (identical to the coordinate system of camera 1)
%   to camera 2.
%

function [R,T] = disambiguateRelativePose(Rots,u3,p1,p2,K1,K2)
    M1= K1 * [eye(3) zeros(3,1)];
    
    M2(:,:,1) = K2 * [Rots(:,:,1) u3];
    M2(:,:,2) = K2 * [Rots(:,:,2) u3];
    M2(:,:,3) = K2 * [Rots(:,:,1) -u3];
    M2(:,:,4) = K2 * [Rots(:,:,2) -u3];
    
    max_pos_point_count = 0
    max_i = 0;
    for i = 1:4
      P = linearTriangulation(p1,p2,M1,M2(:,:,i));
      pos_point_count = sum(P(3,:)>0)
      
      if(pos_point_count > max_pos_point_count)
          max_pos_point_count = pos_point_count;
          max_i = i;
      end
    end
 
    RT = inv(K2) * M2(:,:,max_i);
    R = RT(1:3,1:3);
    T = RT(1:3,4);
end

