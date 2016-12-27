%% Quantitative simulation:

% PREQUISITES:
% Install http://ch.mathworks.com/matlabcentral/fileexchange/33381-jsonlab--a-toolbox-to-encode-decode-json-files

clc;
clear all;
close all;

%-------------------------------------------------------------------------
% STEP 1
% setup the simulation by adding simulation parameters to the ranges struct

ranges = struct(...
    'num_keypoints', [500:100:800], ...
    'harris_patch_size', [10:2:14]);
%-------------------------------------------------------------------------

%extract the vectors from the struct
fields = fieldnames(ranges);
vectors = {};
for i=1:numel(fields)
   vectors = [vectors, ranges.(fields{i})];
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
    
    %-------------------------------------------------------------------------
    % STEP 2
    % setup env params (file names etc)
    env_params = struct(...
    'max_frames', 10, ...
    'csv_durations', true, ...
    'csv_errors', true, ...
    'csv_file_identifier', strcat('task', int2str(i)));
    
    
    %IMPORTANT: params are merged in main with 'defaults'
    main(params,env_params);
end