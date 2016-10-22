function keypoints = selectKeypoints(scores, num, r)
% Selects the num best scores as keypoints and performs non-maximum 
% supression of a (2r + 1)*(2r + 1) box around the current maximum.

    scoresCopy = padarray(scores,[r r]);
    rNan = zeros(2 * r + 1);
    keypoints = zeros(2,num);
    
    for i=1:num
        
        [~,I] = max(scoresCopy(:));
        [maxRow, maxCol] = ind2sub(size(scoresCopy),I);
        
        scoresCopy(maxRow-r:maxRow+r,maxCol-r:maxCol+r) = rNan;
        
        keypoints(:,i) = [maxRow-r; maxCol-r];
        
    end
    

end

