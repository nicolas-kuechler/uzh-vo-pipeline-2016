%% Quantitative simulation:
%
% This is the task scheduler. It makes use of the parallel pool when more
% than one core is available.


%-------------------------------------------------------------------------
% STEP 0 - PREQUISITES:
% Install the jsonlab toolbox.
% Addons -> search -> sign on -> install
%-------------------------------------------------------------------------

clc;
clear all;
close all;

%-------------------------------------------------------------------------
% STEP 1
% Setup the machine environment.
folder = 'sim';
prefix = 'task';
sepa = '\';
%-------------------------------------------------------------------------

% create the folder
if ~exist(folder, 'dir')
  mkdir(folder);
end

%-------------------------------------------------------------------------
% STEP 2
% Setup the simulation by adding simulation parameters to the ranges
% struct. All combinations are going to be simulated in parallel.
%

% example struct:

% ranges = struct(...
%     'nonmaximum_supression_radius', 6:2:16, ...
%     'candidate_cap', 300:100:1000,...
%     'surpress_existing_matches', 0:1:1 ,... %1 for true, 0 for false
%     'add_candidate_each_frame', 50:10:150);

 ranges = struct(...
     'nonmaximum_supression_radius', 10:2:16, ...
     'add_candidate_each_frame', 50:50:200 , ...
     'triangulate_max_repr_error', [0.5, 1, 2, 5, 10, 20], ...
     'critical_kp', [0, 20, 50, 100], ...
     'tracker_max_bidirectional_error', [2.1, 5, 9999999]);

% non-representative sim pipeline test config
% ranges = struct(...
%     'nonmaximum_supression_radius', 3);
%-------------------------------------------------------------------------

% Extract the vectors and field names from the struct to calculate
% permutations.
fields = fieldnames(ranges);
vectors = {};
for i=1:numel(fields)
   vectors = [vectors, ranges.(fields{i})];
end

% get all combinations
% credit to stackoverflow question 21895335
n = numel(vectors); 
% pre-define to generate comma-separated list
combs = cell(1,n); 
% the reverse order in these two
[combs{end:-1:1}] = ndgrid(vectors{end:-1:1}); 
% comma-separated lists is needed to produce the rows of the result
% matrix in lexicographical order 
combs = cat(n+1, combs{:}); %concat the n n-dim arrays along dimension n+1
combs = reshape(combs,[],n); %reshape to obtain desired matrix

% result structure
% 0 = not completed
% 1 = completed
% 2 = error
sim_results = zeros(size(combs,1),1);

% sim index for all combinations
index = 1:size(combs,1);

disp(['There are ' num2str(size(combs,1)) ' tasks scheduled.']);
% parallel for loop all combinations
parfor i=index
    
    % assemble struct for that iteration
    params = struct();
    for f=1:numel(fields)
       params.(fields{f}) =   combs(i,f);
    end
    
    %-------------------------------------------------------------------------
    % STEP 3
    % setup env params (file names etc)
    env_params = struct(...
    'max_frames', 500, ...
    'csv_durations', true, ...
    'csv_errors', true, ...
    'csv_file_identifier', strcat(folder, sepa, prefix , int2str(i)));
    sim_results(i) = 1;
    try
        %IMPORTANT: params are merged in main with 'defaults'
        simMain(params,env_params);
        sim_results(i) = 1; %  no exception caught: success
        disp(['Task: ' num2str(i) ' complete. ']);
    catch err 
        sim_results(i) = 2; % exception caught: failure
        %dump the exception for further analysis
        file = fopen(strcat(env_params.csv_file_identifier,'_exception.txt'),'w');
        fprintf(file, '%s', err.getReport('extended', 'hyperlinks','off'));
        fclose(file);
        disp(['Task: ' num2str(i) ' exception! ']);
    end
end

% Write the header seperatly because of the way matlab handles csv files.
c_header = ['index', fields', 'sim_result'];
overview_fn = strcat(folder, sepa, 'sim_overview.csv');
overview_h_fn = strcat(folder, sepa, 'sim_overview_h.txt');

comma_header = [c_header;repmat({','},1,numel(c_header))]; %insert commas
comma_header = comma_header(:)';
text_header = cell2mat(comma_header); %header in text with commas

file = fopen(overview_h_fn,'w'); 
fprintf(file,'%s',text_header(1:end-1));
fclose(file);

% Dump the simulation results to csv
csvwrite(overview_fn, [index', combs, sim_results]);

disp(['Simulation complete ... Check the output dir: ', folder ]);