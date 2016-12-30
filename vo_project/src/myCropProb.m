function [new_l1, new_l2, new_land_marks, new_m] = myCropProb(l1, l2, land_marks)
%MYCROPPROB Updates correspondence l1 and l2 and removes obsolete landmarks
%from landmarks and returns updates variable. Also returns number of
%landmarks.
new_land_marks = [];
new_l1 = [];
new_l2 = [];
unique_counter = 1;
for i = 1 : size(land_marks, 2)
    
    if any(abs(i-l1)<1e-10) || any(abs(i-l2)<1e-10)
        % if i in l1
        if any(abs(i-l1)<1e-10)
            new_l1 = [new_l1, unique_counter];
        end
        
        % if i in l2
        if any(abs(i-l2)<1e-10)
            new_l2 = [new_l2, unique_counter];    
        end
        
        unique_counter = unique_counter + 1;
        new_land_marks = [new_land_marks, land_marks(:, i)'];
    end
end
new_m = size(new_land_marks, 2) / 3;
end

