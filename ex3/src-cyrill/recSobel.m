function [ SobMat ] = recSobel( patch_dim )
%RECSOBEL Summary of this function goes here
%   Detailed explanation goes here
    
if mod(patch_dim, 2) == 0
    throw(MException('RECSOBEL:badArgument', 'patch_dim cannot be even'));
end

if patch_dim == 3
    SobMat = [1 2 1]' * [1 0 -1];
else
    SobMat = conv2 ([1 2 1]' * [1 2 1], recSobel(patch_dim -2));
end

end

