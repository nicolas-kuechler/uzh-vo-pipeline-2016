function [ XCoord, YCoord, ZCoord ] = getCubeCoordinates( sideLength, startPointX, startPointY )
%GETCUBECOORDINATES Summary of this function goes here
%   Detailed explanation goes here

XCoord=([0 1 1 1 1 1 1 0 0 0 0 0 1 1 0 0]*sideLength + startPointX) * 0.04;
YCoord=([0 0 0 0 1 1 1 1 1 1 0 0 0 1 1 0]*sideLength + startPointY) * 0.04;
ZCoord=[0 0 -1 0 0 -1 0 0 -1 0 0 -1 -1 -1 -1 -1]*sideLength * 0.04;

end

