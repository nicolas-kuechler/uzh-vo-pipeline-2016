clc;
clear;
clear all;

x = 0.04 * (0:1:8);
y = 0.04 * (0:1:5);

[X, Y, Z] = meshgrid (x, y, 0);
[LineX,LineY,LineZ] = getCubeCoordinates(3,2,2);

K = load('../data/K.txt');
Poses = load('../data/poses.txt');
D = load('../data/D.txt');

files = dir('../data/images/*.jpg');

outputVideo = VideoWriter(fullfile('../shuttle_out.avi'));
open(outputVideo);

for i = 1:length(files)
    
    Img = imread(strcat('../data/images/', files(i).name));
    ImgGray = rgb2gray(Img);
    ImgGray = undistortImage(ImgGray,K,D);
    imshow(ImgGray);
    hold on;
    
    om = Poses(i,1:3);
    t = Poses(i,4:6);

    RT = poseVectorToTransformationMatrix(om, t);

    %[U,V] = projectPoints(X,Y,Z,K, RT, D);

    %plot(U,V,'r.','MarkerSize',20)
    
    hold on;

    [LineU,LineV] = projectPoints(LineX,LineY,LineZ,K, RT, D, false);

    plot(LineU,LineV,'r','MarkerSize',20);
    
    img = getframe();
    writeVideo(outputVideo,img);
    
    clf;
    i

end

close(outputVideo);
