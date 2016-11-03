clc;
clear;
clear all;

A = imread('..\data\images_undistorted\img_0001.jpg');
A = rgb2gray(A);

K = load('..\data\K.txt');
detected_corners = load('..\data\detected_corners.txt');
p_W_corners = load('..\data\p_W_corners.txt');

p_W_corners = p_W_corners*1/100;

p = reshape(detected_corners(1,:),[2,length(detected_corners(1,:))/2])
p = [p;ones(1,length(p(1,:)))];

M = estimatePoseDLT(p, p_W_corners', K)

result = K*M*[p_W_corners';ones(1,12)];
lambdas = result(3,:);
result = result(1:2,:)./lambdas;

 imshow(A);
 hold on;
 plot(result(1,:), result(2,:), '.', 'MarkerSize', 15); 

actual = detected_corners(1,:)
actual = reshape(actual, 2, 12)
plot(actual(1,:),actual(2,:), '+', 'MarkerSize', 15); 
