first_frame = 2000;
last_frame = 9004;

for x = first_frame:last_frame
    img = rgb2gray(imread([waedi_path ...
        sprintf('/images_distorted/waedi_%04d.png', x)]));
    
   img_undistorted = undistortImage(img,cameraParams,'OutputView','same', 'FillValues', 128);
   
   imwrite(img_undistorted, [waedi_path sprintf('/images_undistorted/waedi_%04d.png', (x - first_frame))]);
   
   x
    
end