function [ A ] = xprodmat( x )
%XPRODMAT Summary of this function goes here
%   Detailed explanation goes here

    A = [ 0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];

end

