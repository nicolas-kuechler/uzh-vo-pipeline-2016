function descriptors = describeKeypoints(img, keypoints, r)
% Returns a (2r+1)^2xN matrix of image patch vectors based on image
% img and a 2xN matrix containing the keypoint coordinates.
% r is the patch "radius".
padded_img = padarray(img, [r r] ,'replicate');
N = size(keypoints, 2);
descriptors = zeros((2*r+1)^2:N);

for k=1:N
    I_row = keypoints(1,k)+r;
    I_col = keypoints(2,k)+r;
    patch = padded_img(I_row-r:I_row+r, I_col-r:I_col+r);
    descriptors(:, k) = patch(:);
end

end
