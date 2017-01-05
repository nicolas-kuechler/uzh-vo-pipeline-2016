%load('movie_cell.mat')
for i = 1:4153
    frame = movie_cell{2,i};
    if ~isempty(frame)
        imshow(frame.cdata);
    end
    pause(0.1);
end
