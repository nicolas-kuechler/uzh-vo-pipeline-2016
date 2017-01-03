%% Quantitative simulation:

% PREQUISITES:
% Install http://ch.mathworks.com/matlabcentral/fileexchange/33381-jsonlab--a-toolbox-to-encode-decode-json-files

clc;
clear all;
close all;

folder = 'sim';
prefix = 'task';
sepa = '\';
%-------------------------------------------------------------------------
% STEP 1
% setup the simulation by adding simulation parameters to the ranges struct

ranges = struct(...
    'nonmaximum_supression_radius', 6:2:16, ...
    'candidate_cap', 300:100:1000,...
    'surpress_existing_matches', 0:1:1 ,... %1 for true, 0 for false
    'add_candidate_each_frame', 50:10:150);
%-------------------------------------------------------------------------

%extract the vectors and field names from the struct
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

%Result structure
%0 = not completed
%1 = completed
%2 = error
sim_results = zeros(size(combs,1),1);

%Sim Index for all combinations
index = 1:size(combs,1);

disp(['There are ' num2str(size(combs,1)) ' tasks scheduled.']);
%parallel for loop all combinations
parfor i=index
    
    %Assemble struct for that iteration
    params = struct();
    for f=1:numel(fields)
       params.(fields{f}) =   combs(i,f);
    end
    
    %-------------------------------------------------------------------------
    % STEP 2
    % setup env params (file names etc)
    env_params = struct(...
    'max_frames', 500, ...
    'csv_durations', true, ...
    'csv_errors', true, ...
    'csv_file_identifier', strcat(folder, sepa, prefix , int2str(i)));
    sim_results(i) = 1;
    try
        %IMPORTANT: params are merged in main with 'defaults'
        main(params,env_params);
        sim_results(i) = 1;
    catch err 
        sim_results(i) = 2;
        file = fopen(strcat(env_params.csv_file_identifier,'_exception.json'),'w');
        fprintf(file, '%s', err.getReport('extended', 'hyperlinks','off'))
        fclose(file);
    end
end

c_header = ['index', fields', 'sim_result'];
overview_fn = strcat(folder, sepa, 'sim_overview.csv');
overview_h_fn = strcat(folder, sepa, 'sim_overview_h.txt');

comma_header = [c_header;repmat({','},1,numel(c_header))]; %insert commas
comma_header = comma_header(:)';
text_header = cell2mat(comma_header); %header in text with commas

%write header to file
file = fopen(overview_h_fn,'w'); 
fprintf(file,'%s',text_header(1:end-1));
fclose(file);

csvwrite(overview_fn, [index', combs, sim_results]);