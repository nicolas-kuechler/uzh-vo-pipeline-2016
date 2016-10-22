function scores = harris(img, patch_size, kappa)

patch_dim = sqrt(patch_size);

if(mod(patch_dim, 2) == 0)
    throw(MException('HARRIS:IllegalArgument', 'sqrt(patch_size) has to be odd'));
end

Sob = recSobel(patch_dim);
pad_size = floor(patch_dim/2);
img_padded = padarray(img, [pad_size pad_size], 'replicate');

Ix = conv2(im2double(img_padded), Sob, 'valid');
Iy = conv2(im2double(img_padded), Sob', 'valid');

%Alternative: 
%sigma = 5;
%gaussian_filter = fspecial('gaussian', patch_dim, sigma);

box_filter = fspecial('average', patch_dim);

Ix2 = conv2(padarray(Ix .^ 2, [pad_size pad_size]), box_filter, 'valid');
Iy2 = conv2(padarray(Iy .^ 2, [pad_size pad_size]), box_filter, 'valid');
IxIy = conv2(padarray(Ix .* Iy, [pad_size pad_size]), box_filter, 'valid');

scores = (Ix2 .* Iy2 - IxIy .^2) - kappa * (Ix2 + Iy2).^2;
end
