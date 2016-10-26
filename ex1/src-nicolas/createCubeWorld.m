function [ CubeWorld ] = createCube(p1, size)
%CREATECUBE Summary of this function goes here
%   Detailed explanation goes here

x = p1(1);
y = p1(2);
z = p1(3);
s = size;

CubeWorld = [
    x y z;
    x+s y z;
    x+s y+s z;
    x y+s z;
    x y z-s;
    x+s y z-s;
    x+s y+s z-s;
    x y+s z-s;
];
end

