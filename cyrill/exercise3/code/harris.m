function scores = harris(img, patch_size, kappa)


padDim = sqrt(patch_size);

if mod(padDim, 2) == 0
    throw(MException('HARRIS:badArgument', 'patch_dim cannot be even'));
end

sobMat = recSobel(padDim);

padSize = floor(padDim/2);
imgPadded = padarray(img,[padSize padSize]);

Ix = conv2(imgPadded, sobMat, 'valid');
Iy = conv2(imgPadded, sobMat', 'valid');

Ix2 = Ix .^ 2;
Iy2 = Iy .^ 2;

IxIy = Ix .* Iy;

scores = Ix2 .* Iy2 - IxIy .^ 2 - kappa * (Ix2 + Iy2).^2;

end
