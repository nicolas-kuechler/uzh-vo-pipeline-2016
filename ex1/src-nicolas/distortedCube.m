clc;
clear;
clear all;

K = load('..\data\K.txt');
D = load('..\data\D.txt');
Poses = load('..\data\poses.txt');

x = 0:0.04:0.32;
y = 0:0.04:0.20;

[X,Y,Z] = meshgrid(x,y,0);

n = length(Poses(:,1))

CubeWorld = createCubeWorld([0 0 0], 0.08);

for i = 1:n
    % Create an image filename, and read it in to a variable called imageData.
    jpgFileName = strcat('..\data\images\img_', sprintf('%04i', i), '.jpg');
    if exist(jpgFileName, 'file')
        imageData = imread(jpgFileName);
        imageData = rgb2gray(imageData);
    else
        fprintf('File %s does not exist.\n', jpgFileName);
    end
    
    om = Poses(i,1:3);
    t = Poses(i, 4:6);
    
    RT = poseVectorToTransformationMatrix(om, t);
    PointGrid = projectPoints(K, RT, X, Y, Z);
    
    %Cube2d = projectPointsWithDistortion(K, RT, CubeWorld(:,1), CubeWorld(:,2), CubeWorld(:,3), D(1), D(2));
    Cube2d = projectPoints(K, RT, CubeWorld(:,1), CubeWorld(:,2), CubeWorld(:,3));
    imageData = undistortImage(imageData, K, D);
    imshow(imageData);
    hold on;
    drawCube2d(Cube2d);
    drawnow;
end