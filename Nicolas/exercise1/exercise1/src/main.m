clc;
clear;
clear all;

A = imread('..\data\images_undistorted\img_0001.jpg');
A = rgb2gray(A);

K = load('..\data\K.txt');
Poses = load('..\data\poses.txt');
D = load('..\data\D.txt');

x = 0:0.04:0.32;
y = 0:0.04:0.20;

[X,Y,Z] = meshgrid(x,y,0);

om = Poses(1,1:3);
t = Poses(1, 4:6);


RT = poseVectorToTransformationMatrix(om, t);
PointGrid = projectPoints(K, RT, X, Y, Z);

CubeWorld = createCubeWorld([0 0 0], 0.08);
Cube2d = projectPoints(K, RT, CubeWorld(:,1), CubeWorld(:,2), CubeWorld(:,3));

%imshow(A);
%hold on;
%plot(PointGrid(1,:),PointGrid(2,:), '.', 'MarkerSize', 15);
%drawCube2d(Cube2d);


img_0001 = imread('..\data\images\img_0001.jpg');
img_0001 = rgb2gray(img_0001);
imshow(img_0001);
un = undistortImage(img_0001, K, D);
%imshow(un);

PointGridDistorted = projectPointsWithDistortion(K, RT, X, Y, Z, D(1), D(2));

%imshow(img_0001);
%hold on;
%plot(PointGridDistorted(1,:),PointGridDistorted(2,:), '.', 'MarkerSize', 15);
