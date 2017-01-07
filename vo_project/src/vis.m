%load('movie_VO.mat')
writerObj = VideoWriter('video_VO'); % Name it.
writerObj.FrameRate = 1.7; % How many frames per second.
open(writerObj); 
i = 135;
while (true)
    if waitforbuttonpress == 1
        i = i + 1;
    else
        i = i - 1;
    end
        frame = movie_cell{2,i};
    if ~isempty(frame)
        %writeVideo(writerObj, frame.cdata);
        imshow(frame.cdata);
    end
    pause(0.1)
end
close(writerObj)