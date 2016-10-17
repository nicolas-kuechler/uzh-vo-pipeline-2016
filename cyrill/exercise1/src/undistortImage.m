function [ ImgUndistorted ] = undistortImage( ImgDistorted, K, D )
%UNDISTORTIMAGE Summary of this function goes here
%   Detailed explanation goes here
    
    I = zeros(size(ImgDistorted));
    [i, j] = find(~isnan(I));
    
    XYZ = inv(K) * [j i ones(length(i),1)]';
    
    x = XYZ(1,:);
    y = XYZ(2,:);
    r2 = (x.^2 + y.^2);
    
    x = ( 1 + D(1) * r2 + D(2) * r2.^2 ) .* x;
    y = ( 1 + D(1) * r2 + D(2) * r2.^2 ) .* y;
    
    UV1 = K * [x; y; ones(1, length(x))];
    
    UV = [UV1(1,:); UV1(2,:)];
    UV = round(UV);
    
    u = reshape(UV(1,:),size(I));
    v = reshape(UV(2,:),size(I));
    
    ImgUndistorted = I;
    
    for j = 1:length(I(1,:))
        for i = 1:length(I(:,1))
            ImgUndistorted(i,j) = ImgDistorted(v(i,j),u(i,j));
        end
    end
    
    ImgUndistorted = uint8(ImgUndistorted);

end

