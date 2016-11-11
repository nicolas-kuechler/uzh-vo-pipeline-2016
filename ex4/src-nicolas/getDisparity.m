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
count = 0;
for row = first_row : last_row
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
    
    
    D = pdist2(right_patches', left_patches','squaredeuclidean');
    [M,I] = min(D);
    
    n = size(D,2);
    Int_I = zeros(1,n);
    
    for i = 1:n
        %1st Check D < 1.5 M
        minimum_match = 1.5 * M(i);
        sum_sat_cond = sum(D(:,i)<=minimum_match);
        
        if sum_sat_cond <=2 & i-I(i) > min_disp & i-I(i) < max_disp
            count = count + 1;
            left_i = i-1;
            left_i(left_i<1)=1;
            right_i = i+1;
            right_i(right_i>n)=n;
           
            x = [left_i i right_i];
            y = [D(I(i),left_i) M(i) D(I(i),right_i)];
            
            p = polyfit(x,y,2);
            d1p = polyder(p);
            Int_I(i) = roots(d1p);
        end      
    end
    
    disp = indices-Int_I;
    disp(Int_I==0 | disp<=min_disp | disp >= max_disp)=0;
    
   
    %[D,I] = pdist2(right_patches', left_patches','squaredeuclidean', 'Smallest', 3);
    
   
    
    %Filter 1: Reject if 3 or more matches with d <= 1.5 * min   
    %minimum_matches = 1.5 * D(1,:);
    %occ_sat_cond = sum(D<=minimum_matches);
    %disp(occ_sat_cond>2 | disp>=max_disp | disp<=min_disp) = 0;
    
    % Part 4
    
    %
    
    disp_img(row-patch_radius,:) = disp;
end  
    disp_img = padarray(disp_img, [patch_radius, patch_radius]);
    disp_img(:,1:patch_radius+max_disp)=0;
end

