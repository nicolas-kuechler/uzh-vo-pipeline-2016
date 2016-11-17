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
    P_homog = zeros(4, N);

    for i = 1:N
        
       A1 = xprodmat(p1(:,i)) * M1;
       A2 = xprodmat(p2(:,i)) * M2;
       
       A = [A1;A2];
       
       [~,~,V] = svd(A);
        
       P_homog(:,i) = V(:,end);
        
    end

    P = P_homog ./ P_homog(4,:);

end


