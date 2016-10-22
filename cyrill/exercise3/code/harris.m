function scores = harris(img, patch_size, kappa)


patchDim = sqrt(patch_size);

if mod(patchDim, 2) == 0
    throw(MException('HARRIS:badArgument', 'patch_dim cannot be even'));
end

sobMat = recSobel(patchDim);

padSize = floor(patchDim/2);
imgPadded = padarray(img,[padSize padSize], 'replicate');

Ix = conv2(imgPadded, sobMat, 'valid');
Iy = conv2(imgPadded, sobMat', 'valid');

g = fspecial('average',[patchDim patchDim]);

Ix2 = conv2(padarray(Ix .^ 2,[padSize padSize], 'replicate'), g, 'valid');
Iy2 = conv2(padarray(Iy .^ 2,[padSize padSize], 'replicate'), g, 'valid');

IxIy = conv2(padarray(Ix .* Iy,[padSize padSize], 'replicate'), g, 'valid');

scores = (Ix2 .* Iy2 - IxIy .^ 2 - kappa * (Ix2 + Iy2).^2);

end
