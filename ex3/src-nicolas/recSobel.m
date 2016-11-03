function [ Sob ] = recSobel( patch_dim )
%RECSOBEL Summary of this function goes here
%   Detailed explanation goes here
if(mod(patch_dim, 2) == 0)
    throw(MException('RECSOBEL:IllegalArgument', 'patch_dim has to be odd'));
end

% Base Case
if patch_dim == 3
    Sob = [1 2 1]'*[1 0 -1];
else %Rec Case
    Sob = conv2([1 2 1]'*[1 2 1], recSobel(patch_dim-2));
end

end

