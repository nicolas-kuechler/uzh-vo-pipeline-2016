% DECOMPOSEESSENTIALMATRIX - Given an essential matrix, compute the camera motion, i.e.,  R and T such
% that E ~ T_x R
% 
% Input:
%   - E(3,3) : Essential matrix
%
% Output:
%   - R(3,3,2) : the two possible rotations
%   - u3(3,1)   : a vector with the translation information


function [R,u3] = decomposeEssentialMatrix(E)
    
    W = [0 -1 0; 1 0 0; 0 0 1];
    
    [U,~,V] = svd(E);
    
    R(:,:,1) = U * W * V';
    R(:,:,2) = U * W' * V';
    
    for i = 1:2
       if det(R(:,:,i)) < 0
           
           R(:,:,i) = R(:,:,i) * -1;
        
       end
    end
    
    u3 = U(:,end);
    
end
