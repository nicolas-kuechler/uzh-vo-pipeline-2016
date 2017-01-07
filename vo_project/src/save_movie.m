%load('movie_VO.mat')
writerObj = VideoWriter('video_VO'); % Name it.
writerObj.FrameRate = 1.7; % How many frames per second.
open(writerObj); 

for i = 1:1100
    frame = movie_cell{2,i};
    if ~isempty(frame)
        writeVideo(writerObj, frame.cdata);
        i
    end
end
close(writerObj)