function my_save_movie(file_name, multiplier, movie_cell) 

movObj = QTWriter(file_name); % Name it.

for i = size(movie_cell, 2)
    frame = movie_cell{2,i};
    if ~isempty(frame)
        time = movie_cell{1,i-1};
        movObj.FrameRate = multiplier/time;
        writeMovie(movObj, frame.cdata);
    end
end
close(movObj)
end