function [O1, O2, new_l2, new_land_marks, new_m] = myCropProb(O1, O2, land_marks)
%MYCROPPROB Updates observations O1 and O2 and removes obsolete landmarks
%from landmarks and returns updated variable. Also returns number of
%landmarks.

% extract correspondences from observations
k1 = O1(1);
k2 = O2(1);

l1 = O1(2 + 2 * k1 : 3 * k1 + 1);
l2 = O2(2 + 2 * k2 : 3 * k2 + 1);

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
        new_land_marks = [new_land_marks, land_marks(:, i)];
    end
end
new_m = size(new_land_marks, 2);

O1(2 + 2 * k1 : 3 * k1 + 1) = new_l1;
O2(2 + 2 * k2 : 3 * k2 + 1) = new_l2;
end

