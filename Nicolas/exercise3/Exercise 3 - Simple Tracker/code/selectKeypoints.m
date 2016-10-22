function keypoints = selectKeypoints(scores, num, r)
% Selects the num best scores as keypoints and performs non-maximum 
% supression of a (2r + 1)*(2r + 1) box around the current maximum.

scores_copy = scores;
%add padding to ensure that the nan box doesn't get out of bound
scores_copy = padarray(scores_copy, [r r]);
rNan = NaN(2*r + 1, 'double');
keypoints = zeros(2, num);

for k=1:num
    [~,I] = max(scores_copy(:));
    [I_row, I_col] = ind2sub(size(scores_copy),I);
    
    %Non Maximum Surpression
    scores_copy(I_row-r:I_row+r, I_col-r:I_col+r) = rNan;
    keypoints(:,k) = [I_row-r; I_col-r];
end

end

