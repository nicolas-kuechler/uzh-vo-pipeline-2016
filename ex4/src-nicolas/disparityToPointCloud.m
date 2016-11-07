function [points, intensities] = disparityToPointCloud(...
    disp_img, K, baseline, left_img)
% points should be 3xN and intensities 1xN, where N is the amount of pixels
% which have a valid disparity. I.e., only return points and intensities
% for pixels of left_img which have a valid disparity estimate! The i-th
% intensity should correspond to the i-th point.

N = sum(sum(disp_img>0));
points = zeros(3,N);
intensities = zeros(1,N);
n = 0;
inv_K = inv(K);

for row = 1:size(disp_img,1)
    for col = 1:size(disp_img,2)
        d = disp_img(row,col);
        if d > 0
            n = n + 1;
            p_0 = [col;row];
            p_1 = [col-d;row];
            A = [inv_K * [p_0;1] -inv_K*[p_1;1]];
            lambdas = A\[baseline;0;0];
            points(:,n) = lambdas(1) * inv_K*[p_0;1];
            intensities(n) = left_img(row,col);
         end        
    end
end
end

