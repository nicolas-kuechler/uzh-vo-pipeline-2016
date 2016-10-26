clc;
clear all;
close all;

Img = imread(strcat('../data/images/img_0001.jpg'));
ImgGray = rgb2gray(Img);

K = load('../data/K.txt');
D = load('../data/D.txt');

Img = undistortImage(ImgGray, K, D);

imshow(Img);