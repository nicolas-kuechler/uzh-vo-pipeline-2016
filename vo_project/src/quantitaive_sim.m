%% Quantitative simulation:

%-------------------------------------------------------------------------
%%STEP 1
%setup the simulation by adding simulation parameters to the ranges struct

ranges = struct(...
    'test', 1, ...
    'num_keypoints', [500:100:800], ...
    'harris_patch_size', [0:2:10]);


env_params = struct()
%-------------------------------------------------------------------------

%extract the vectors from the struct
fields = fieldnames(ranges);
vectors = {}
for i=1:numel(fields)
   vectors = [vectors, ranges.(fields{i})]
end

%get all combinations
%credit to stackoverflow question 21895335
n = numel(vectors); 
combs = cell(1,n); %pre-define to generate comma-separated list
[combs{end:-1:1}] = ndgrid(vectors{end:-1:1}); %the reverse order in these two
%comma-separated lists is needed to produce the rows of the result matrix in lexicographical order 
combs = cat(n+1, combs{:}); %concat the n n-dim arrays along dimension n+1
combs = reshape(combs,[],n); %reshape to obtain desired matrix

%parallel for loop all combinations
parfor i=1:size(combs,1)
    params = struct();
    for f=1:numel(fields)
       params.(fields{f}) =   combs(i,f);
    end
    
    %IMPORTANT: params are merged in main with 'defaults'
    main(params)
end