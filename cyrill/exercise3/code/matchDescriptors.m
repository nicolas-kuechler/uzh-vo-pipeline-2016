function matches = matchDescriptors(...
    query_descriptors, database_descriptors, lambda)
% Returns a 1xQ matrix where the i-th coefficient is the index of the
% database descriptor which matches to the i-th query descriptor.
% The descriptor vectors are MxQ and MxD where M is the descriptor
% dimension and Q and D the amount of query and database descriptors
% respectively. matches(i) will be zero if there is no database descriptor
% with an SSD < lambda * min(SSD). No two non-zero elements of matches will
% be equal.
    
    [D,I] = pdist2(query_descriptors', database_descriptors', 'euclidean', 'Smallest', 1);
    d_min = min(D);
    
    delta = lambda * d_min;
    
    n = size(query_descriptors, 2);
    matches = zeros(2, n);
    
    for i = 1:n
        
        curr_val = D(i);
        curr_min_val = matches(2,I(i));
        
        if curr_val < delta && curr_min_val == 0 || curr_min_val > curr_val
            matches(:,I(i)) = [i; curr_val];
        end  
    end
    
    matches = matches(1,:);
    
end
