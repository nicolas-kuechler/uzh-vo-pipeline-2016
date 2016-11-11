function disp_img = getDisparity(...
    left_img, right_img, patch_radius, min_disp, max_disp)
% left_img and right_img are both H x W and you should return a H x W
% matrix containing the disparity d for each pixel of left_img. Set
% disp_img to 0 for pixels where the SSD and/or d is not defined, and for d
% estimates rejected in Part 2. patch_radius specifies the SSD patch and
% each valid d should satisfy min_disp <= d <= max_disp.

%

patch_size = (2 * patch_radius + 1) ^ 2;
start = patch_radius + 1;
end_row = size(left_img,1) - patch_radius;
end_col = size(left_img,2) - patch_radius;


disp_img = zeros(end_row - start + 1, end_col - start + 1);
indices = (1:end_col - start + 1);

for row = (start : end_row)
    
    left_patches = zeros(patch_size, end_col - start +1);
    right_patches = left_patches;
    
    for col = (start : end_col)
        
        % get left patch
        left_patch = left_img(row - patch_radius: row + patch_radius, col - patch_radius : col + patch_radius);
        left_patches(:, col - patch_radius) = left_patch(:);
        
        % get right patch
        right_patch = right_img(row - patch_radius: row + patch_radius, col - patch_radius : col + patch_radius);
        right_patches(:,col - patch_radius) = right_patch(:);   
    end
    
    [D,I] = pdist2(right_patches', left_patches', 'squaredeuclidean', 'Smallest', 3);
    
    min_dists = 1.5 * D(1,:);
    
    disparities = indices - I(1,:);
    
    disparities(1, D(2,:) <= min_dists & D(3,:) <= min_dists) = 0;
    disparities(disparities <= min_disp | disparities >= max_disp) = 0;
    
    for x=max_disp + 2:size(disparities(:)) - 1 
        
        if disparities(x) ~= 0
            
            right_range = (x-disparities(x)-1:x-disparities(x)+1);
            
            D_spr = pdist2(right_patches(:,right_range)', left_patches(:,x)', 'squaredeuclidean');
            
            p = polyfit((-1:1), D_spr', 2);
            
            p_der = polyder(p);
            
            p_min = roots(p_der);
            
            disparities(x) = disparities(x) + p_min;
            
         end
         
     end
    
    disp_img(row-patch_radius,:) = disparities(1,:);
    
end

disp_img = padarray(disp_img, [patch_radius, patch_radius]);
disp_img(:,1: patch_radius + max_disp) = 0;
end

