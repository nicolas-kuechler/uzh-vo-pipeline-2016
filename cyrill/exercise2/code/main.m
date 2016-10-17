clc;
clear all;
close all;

K = load('../data/K.txt');
PWCorners = load('../data/p_W_corners.txt');
DetectedCorners = load('../data/detected_corners.txt');

Kinv = inv(K);

FirstDetectedCorners = DetectedCorners(1,:);
FirstDetectedCorners = [FirstDetectedCorners(1:2:end); FirstDetectedCorners(2:2:end)];
M = estimatePoseDLT(FirstDetectedCorners, PWCorners, Kinv);

reprojected_Points = reprojectPoints(PWCorners, M, K);