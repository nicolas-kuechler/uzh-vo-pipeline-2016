function [points, intensities] = disparityToPointCloud(...
    disp_img, K, baseline, left_img)
% points should be 3xN and intensities 1xN, where N is the amount of pixels
% which have a valid disparity. I.e., only return points and intensities
% for pixels of left_img which have a valid disparity estimate! The i-th
% intensity should correspond to the i-th point.

points = zeros(3,1);
intensities = zeros(1);

K_inv = inv(K);

n = 1;

for row = 1:size(disp_img,1)
    
   for col = 1:size(disp_img,2)
       
       disparity = disp_img(row,col);
       
       if disparity > 0
           
           A =  [K_inv * [col;row;1] -K_inv * [col-disparity;row;1]];
           
           lambda = A \ [baseline; 0; 0];
           
           P = lambda(1) * K_inv * [col; row; 1];
           
           points(:,n) = P;
           intensities(n) = left_img(row,col);
           
           n = n+1;
          
           
       end
       
       
   end
    
    
end

end

