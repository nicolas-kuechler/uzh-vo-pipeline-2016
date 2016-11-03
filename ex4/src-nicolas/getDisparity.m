function disp_img = getDisparity(...
    left_img, right_img, patch_radius, min_disp, max_disp)
% left_img and right_img are both H x W and you should return a H x W
% matrix containing the disparity d for each pixel of left_img. Set
% disp_img to 0 for pixels where the SSD and/or d is not defined, and for d
% estimates rejected in Part 2. patch_radius specifies the SSD patch and
% each valid d should satisfy min_disp <= d <= max_disp.

patch_size = (2 * patch_radius + 1)^2;

first_row = patch_radius+1
last_row = size(left_img, 1) - patch_radius;

first_col = patch_radius+1;
last_col = size(left_img, 2) - patch_radius;

disp_img = zeros(last_row-first_row+1, last_col-first_col+1);

indices = (1:last_col-first_col + 1);

parfor row = first_row : last_row
    left_patches = zeros(patch_size, last_col-first_col + 1);
    right_patches = zeros(patch_size, last_col-first_col + 1);
    for col = first_col : last_col
        %patch left
        patch = left_img(row-patch_radius:row+patch_radius, col-patch_radius:col+patch_radius);
        left_patches(:,col-patch_radius) = patch(:);
        
        %patch right
        patch = right_img(row-patch_radius:row+patch_radius, col-patch_radius:col+patch_radius);
        right_patches(:,col-patch_radius) = patch(:);
    end
    [~,I] = pdist2(right_patches', left_patches','squaredeuclidean', 'Smallest', 1);
    disp_img(row-patch_radius,:) = indices-I;
end
    disp_img(disp_img>max_disp)=max_disp;
    disp_img(disp_img<min_disp)=min_disp;
    disp_img = padarray(disp_img, [patch_radius, patch_radius]);
    disp_img(:,1:patch_radius+max_disp)=0;
end

