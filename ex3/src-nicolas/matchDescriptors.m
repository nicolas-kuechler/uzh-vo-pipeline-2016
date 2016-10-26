function matches = matchDescriptors(...
    query_descriptors, database_descriptors, lambda)
% Returns a 1xQ matrix where the i-th coefficient is the index of the
% database descriptor which matches to the i-th query descriptor.
% The descriptor vectors are MxQ and MxD where M is the descriptor
% dimension and Q and D the amount of query and database descriptors
% respectively. matches(i) will be zero if there is no database descriptor
% with an SSD < lambda * min(SSD). No two non-zero elements of matches will
% be equal.

% Find for each db descriptor the best matching query descriptor
[D,I] = pdist2(query_descriptors',database_descriptors','euclidean', 'Smallest', 1);

% Calculate dynamic threshold
[d_min,~] = min(D);
delta = lambda * d_min;

% Initialize matches array. First row is used to store the index of the
% best matching db descriptor
% Second row is used to store the current known best match min value for the query
% desc i
matches = zeros(2,size(query_descriptors, 2));
n = size(I,2);

for k = 1:n
    curr_val = D(k);
    curr_min_val = matches(2,I(k));
    % If the best match for the db descriptor k is below the threshold and
    % smaller then the current known best match, then store it.
    if curr_val < delta && (curr_min_val == 0 || curr_val < curr_min_val) 
        matches(1,I(k)) = k;
        matches(2,I(k)) = curr_val; 
    end
end

matches = matches(1,:);
end
