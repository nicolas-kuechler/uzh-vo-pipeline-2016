first_frame = 1;
last_frame = 6495;

for x = first_frame:last_frame
    img = rgb2gray(imread([vespa_path ...
        sprintf('/images/vespa_%04d.png', x)]));
    
   img_undistorted = undistortImage(img,cameraParams,'OutputView','same', 'FillValues', 128);
   
   imwrite(img_undistorted, [vespa_path sprintf('/images_undistorted/vespa_%04d.png', x)]);
   
   x
    
end