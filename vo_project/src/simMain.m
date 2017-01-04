function[] = simMain(override_sim_params, override_env_params)
    %
    % Make sure the workspace is cleaned up when run individually (debugging).
    %
    clc;
    if exist('override_env_params') ~= 1 &&  exist('override_sim_params')  ~= 1
       disp('Clearing whole workspace') 
       clear all;
    end

    close all;
    addpath(genpath('./'));
     % Fixate rng to seed to allow a more adequate comparison between scores.
    rng(1);
    
    ds = 0; % 0: KITTI, 1: Malaga, 2: parking
    kitti_path = '../data/kitti';
    malaga_path = '../data/malaga-urban-dataset-extract-07';
    parking_path = '../data/parking';


    if ds == 0
        % need to set kitti_path to folder containing "00" and "poses"
        assert(exist('kitti_path', 'var') ~= 0);
        ground_truth = load([kitti_path '/poses/00.txt']);
        % ground_truth = ground_truth(:, [end-8 end]);
        last_frame = 4540;
        K = [7.188560000000e+02 0 6.071928000000e+02 %initial guess for position estimation
            0 7.188560000000e+02 1.852157000000e+02
            0 0 1];
    elseif ds == 1
        % Path containing the many files of Malaga 7.
        assert(exist('malaga_path', 'var') ~= 0);
        images = dir([malaga_path ...x = [0;0;0;0;0;0;1];
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
        left_images = images(3:2:end);
        last_frame = length(left_images);fields = fieldnames(override_params);
        for i=1:numel(fields)
          params.(fields{i}) =   override_params.(fields{i})
        end
        K = [621.18428 0 404.0076
            0 621.18428 309.05989
            0 0 1];
    elseif ds == 2
        % Path containing images, depths and all...
        assert(exist('parking_path', 'var') ~= 0);
        last_frame = 598;
        K = load([parking_path '/K.txt']);

        ground_truth = load([parking_path '/poses.txt']);
        ground_truth = ground_truth(:, [end-8 end]);
    else
        assert(false);
    end

    %% Bootstrap
    % need to set bootstrap_frames
    bootstrap_frames = [000001 000003];           

    if ds == 0
        img0 = imread([kitti_path '/00/image_0/' ...
            sprintf('%06d.png',bootstrap_frames(1))]);
        img1 = imread([kitti_path '/00/image_0/' ...
            sprintf('%06d.png',bootstrap_frames(2))]);
    elseif ds == 1
        img0 = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(bootstrap_frames(1)).name]));
        img1 = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(bootstrap_frames(2)).name]));
    elseif ds == 2
        img0 = rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
        img1 = rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
    else
        assert(false);
    end

    % The simulation runs without bundle adjustment.
    % These are the default parameters
    params = struct('harris_patch_size', 9, ...
                    'harris_kappa', 0.08, ...
                    'num_keypoints', 800, ... 
                    'nonmaximum_supression_radius', 10, ...
                    'descriptor_radius', 13,... 
                    'match_lambda', 8, ...
                    'triangulation_angle_threshold', 3,...
                    'surpress_existing_matches', true ,... 
                    'candidate_cap', 500,...
                    'add_candidate_each_frame', 100 ,...
                    'triangulate_max_repr_error', 20, ... 
                    'runBA', false, ...
                    'alignment', true, ...
                    'critical_kp', 0, ...
                    'tracker_max_bidirectional_error', inf, ...
                    'tracker_blocksize', 13, ...
                    'ransac_num_iterations', 500, ...
                    'ransac_pixel_tolerance', 10);

    % This configuration adjusts the simulation length and output.
    env_params = struct(...
        'max_frames', 500, ...
        'console_log_durations', false, ...
        'console_log_framenumber', false, ...
        'csv_durations', true, ...
        'csv_errors', true, ...
        'csv_file_identifier', 'sim/output');


    % Override params when this script is run as function from another
    % environment.

    if exist('override_sim_params') == 1
        fields = fieldnames(override_sim_params);
        for i=1:numel(fields)
          params.(fields{i}) =   override_sim_params.(fields{i});
        end
    end

    if exist('override_env_params') == 1
        fields = fieldnames(override_env_params);
        for i=1:numel(fields)
          env_params.(fields{i}) =   override_env_params.(fields{i});
        end
    end


    [R, T, repr_error, pt_cloud, keypoints_l, keypoints_r] = initializePointCloudMono(img0,img1,K, params);

    % state 
    tau1 = zeros(6, 1);
    tau2 = HomogMatrix2twist([R', -R' * T; 0, 0, 0, 1]);
    n = 2; m = size(pt_cloud, 2); k1 = m; k2 = m;
    prev_state = struct('pt_cloud', pt_cloud, ... %initial guess for position estimation
                        'matched_kp', keypoints_r, ...
                        'candidates', [], ...
                        'candidates_start', [], ...
                        'candidates_start_pose', [], ...
                        'hidden_state', [tau1', tau2', pt_cloud(:)'], ...
                        'observations', [2, m, k1, keypoints_l(:)', 1:m, ...
                                               k2, keypoints_r(:)', 1:m]);

    locations = [zeros(3,1), -R' * T];
    orientations = [reshape(eye(3), 9, 1), R(:)];

    %% Continuous operation
    
    % adjust the simulation frame number
    if env_params.max_frames < last_frame
        range = (bootstrap_frames(2)+1):env_params.max_frames;
    else
        range = (bootstrap_frames(2)+1):last_frame;
    end

    % preallocate duration matrix
    if (env_params.csv_durations)
        all_durations = zeros(length(range),1);
    end

    % preallocate error matrix
    if(env_params.csv_errors)
        all_errors = zeros(length(range),4);
    end

    % bootstrap the prev image
    prev_img = img1;
    
    % initial guess for position estimation
    x = [0;0;0;0;0;0;1];
    
    % main simulation iteration
    for i = range

        if (env_params.console_log_framenumber)
            fprintf('\n\nProcessing frame %d\n=====================\n', i);
        end

        if ds == 0
            next_image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
        elseif ds == 1
            next_image = rgb2gray(imread([malaga_path ...
                '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
                left_images(i).name]));
        elseif ds == 2
            next_image = im2uint8(rgb2gray(imread([parking_path ...
                sprintf('/images/img_%05d.png',i)])));
        else
            assert(false);
        end

        if (env_params.console_log_durations || env_params.csv_durations)
            tic;
        end

        % process next image and find next location and orientation
        [R, T, next_state] = processFrame(next_image, prev_img, prev_state, K, params);

        if (env_params.console_log_durations || env_params.csv_durations)
            frame_processing_time = toc;
        end

        % log all durations recorded to console
        if (env_params.console_log_durations)
            disp(['ProcessFrame took: ' num2str(frame_processing_time) ' seconds']);
        end

        % gather data for timings csv
        if (env_params.csv_durations)
            all_durations(i) = frame_processing_time;
        end

        if(env_params.csv_errors)
           % collect orientations and locations
            orientations = [orientations, R(:)];
            locations = [locations, -R' * T];

            % align trajectories
            [~, ~, x, loc_error, ori_error] = alignEstimateToGroundTruth(...
                x, ground_truth(1:i-1,  :), locations, next_state.pt_cloud, orientations);

             all_errors(i,:) = [loc_error, ori_error, loc_error / i, ori_error / i];
        end

        prev_img = next_image;
        prev_state = next_state;
    end

    if(env_params.csv_durations)
        csvwrite(strcat(env_params.csv_file_identifier,'_durations.csv'), all_durations);
    end

    if(env_params.csv_errors)
        csvwrite(strcat(env_params.csv_file_identifier,'_errors.csv'), all_errors);
    end

    if(env_params.csv_durations || env_params.csv_errors)
        json = savejson(struct('params', params, 'env_params', env_params));
        json = regexprep(json, '\\/', '/'); % fix unix slash issue
        file = fopen(strcat(env_params.csv_file_identifier,'_config.json'),'w');
        fprintf(file,json);
        fclose(file);
    end
end

