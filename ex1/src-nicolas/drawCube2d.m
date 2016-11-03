function [  ] = drawCube( Cube2d )
%DRAWCUBE Summary of this function goes here
%   Detailed explanation goes here

line([Cube2d(1,1:4) Cube2d(1,1)], [Cube2d(2,1:4) Cube2d(2,1)], 'Color', 'red', 'LineWidth', 2);
line([Cube2d(1,5:8) Cube2d(1,5)], [Cube2d(2,5:8) Cube2d(2,5)], 'Color', 'red', 'LineWidth', 2);
line([Cube2d(1,1) Cube2d(1,5)],  [Cube2d(2,1) Cube2d(2,5)], 'Color', 'red', 'LineWidth', 2);
line([Cube2d(1,2) Cube2d(1,6)],  [Cube2d(2,2) Cube2d(2,6)], 'Color', 'red', 'LineWidth', 2);
line([Cube2d(1,3) Cube2d(1,7)],  [Cube2d(2,3) Cube2d(2,7)], 'Color', 'red', 'LineWidth', 2);
line([Cube2d(1,4) Cube2d(1,8)],  [Cube2d(2,4) Cube2d(2,8)], 'Color', 'red', 'LineWidth', 2);
end

