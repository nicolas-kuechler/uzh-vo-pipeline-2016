function descriptors = describeKeypoints(img, keypoints, r)
% Returns a (2r+1)^2xN matrix of image patch vectors based on image
% img and a 2xN matrix containing the keypoint coordinates.
% r is the patch "radius".

    imgPadded = padarray(img, [r r]);
    patch_size = 2*r + 1;
    num = size(keypoints, 2);
    descriptors = zeros(patch_size^2, num);
    
    for i=1:num
        rowStart = keypoints(1,i);
        rowEnd = rowStart + 2 * r;
        colStart = keypoints(2,i);
        colEnd = colStart + 2 * r;
        descriptors(:, i) = reshape(imgPadded(rowStart:rowEnd, colStart:colEnd), patch_size ^ 2, 1);
        
    end

end
