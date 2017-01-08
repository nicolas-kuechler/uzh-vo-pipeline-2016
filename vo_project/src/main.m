%% Workspace Preparation
clc;
clear all;
close all;
addpath(genpath('./'));
rng(1)

%% Configuration Section
dataset_id = 1; % 0: KITTI, 1: Malaga, 2: parking, 3: own dataset
bundle_adjustment = true; 
align_to_ground_truth = true;

kitti_path = '../data/kitti';
malaga_path = '../data/malaga-urban-dataset-extract-07';
parking_path = '../data/parking';
vespa_path = '../data/vespa';

%% Load Dataset

if dataset_id == 0
    % need to set kitti_path to folder containing "00" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/00.txt']);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
elseif dataset_id == 1
    % Path containing the many files of Malaga 7.
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
elseif dataset_id == 2
    % Path containing images, depths and all...
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);
elseif dataset_id == 3
    assert(exist('vespa_path', 'var') ~= 0);
    images = dir([vespa_path ...
        '/images']);
    K = [609.932619643408, 0, 0;
         0, 610.173360091855, 0;
         329.144287081628, 181.258698042663, 1]';
    last_frame = 3247;
else
    assert(false);
end

%% Bootstrap
% need to set bootstrap_frames
bootstrap_frames = [000001 000003]; 
if exist('ground_truth')
    ground_truth(2, :) = [];
else
    align_to_ground_truth = 0;
end

if dataset_id == 0
    img0 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif dataset_id == 1
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif dataset_id == 2
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
elseif dataset_id == 3
    img0 = imread([vespa_path ...
        sprintf('/images_undistorted/vespa_%04d.png', bootstrap_frames(1))]);
    img1 = imread([vespa_path ...
        sprintf('/images_undistorted/vespa_%04d.png',bootstrap_frames(2))]);
else
    assert(false);
end

params = struct('harris_patch_size', 9, ...
                'harris_kappa', 0.08, ...
                'num_keypoints', 800, ... 
                'nonmaximum_supression_radius', 10, ...
                'descriptor_radius', 13,... 
                'match_lambda', 8, ...
                'use_adaptive_angles', true,...
                'fix_triangulation_angle_threshold', 3,... % must be set if use_adaptive_angles is false
                'adaptive_triangulation_angle', [100, 1.5, 200, 2, 300, 4, 500, 10, 30],... % [nKP_tres_1, angle_1, ... , nKP_tres_4, angle_4, angle_over_tres_4]
                'surpress_existing_matches', true ,... 
                'candidate_cap', 500,...
                'add_candidate_each_frame', 150 ,...
                'triangulate_max_repr_error', 5, ... 
                'runBA', false, ...
                'alignment', align_to_ground_truth, ...
                'critical_kp', 100, ...
                'tracker_max_bidirectional_error', 5, ...
                'tracker_blocksize', 13, ...
                'ransac_num_iterations', 500, ...
                'ransac_pixel_tolerance', 5);
            
% Optimize parameters when bundle adjustment is active
if bundle_adjustment 
    params.candidate_cap = 700;
    params.add_candidate_each_frame = 200;
    params.runBA = true;
    params.critical_kp = 20;
    params.tracker_max_bidirectional_error = 4;
    params.ba_frequency = 3;
    params.triangulate_max_repr_error = 4;
    params.ransac_pixel_tolerance = 4;
end

% Bootstrap key frames
[R, T, repr_error, pt_cloud, keypoints_l, keypoints_r] = initializePointCloudMono(img0,img1,K, params);
tau1 = zeros(6, 1);
tau2 = HomogMatrix2twist([R', -R' * T; 0, 0, 0, 1]);

%% Continuous operation

% Initialize frame state
n = 2; m = size(pt_cloud, 2); k1 = m; k2 = m;
prev_state = struct('pt_cloud', pt_cloud, ... % (3xn) - world coordinates of n matched landmarks
                    'matched_kp', keypoints_r, ... % (2xn) - pixel coordinates of n matched features
                    'candidates', [], ... % (2xm) - pixel coordinates of tracked m candidates
                    'candidates_start', [], ... % (2xm) - pixel coordinates at first observation of m candidates
                    'candidates_start_pose', [], ... % (12xm) - camera homography at first observation for m candidates
                    'hidden_state', [tau1', tau2', pt_cloud(:)'], ... % (1xp) used for bundle adjustment
                    'observations', [n, m, k1, keypoints_l(:)', 1:m, ... % (1xq) used for bundle adjustment
                                           k2, keypoints_r(:)', 1:m]);

% Initialize trajetory
locations = [zeros(3,1), -R' * T];
orientations = [reshape(eye(3), 9, 1), R(:)];

% Initialize plotting (plotPipeline) environment 
fig_num = NaN;
num_candidates_history = nan(1,20);
num_matched_kp_history = nan(1,20);

% First alignment guess (based on ground truth data)
alignment_params = [0;0;0;0;0;0;1];

range = (bootstrap_frames(2)+1):last_frame;
prev_img = img1;

for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if dataset_id == 0
        next_image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
    elseif dataset_id == 1
        next_image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif dataset_id == 2
        next_image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    elseif dataset_id == 3
        next_image = im2uint8(imread([vespa_path ...
            sprintf('/images_undistorted/vespa_%04d.png',i)]));     
    else
        assert(false);
    end
    
    if params.runBA && mod(i, params.ba_frequency) == 1
        % refine poses and point cloud
        hidden_state = runBA(prev_state.hidden_state, prev_state.observations, K);
        n = prev_state.observations(1);
        m = prev_state.observations(2);
        
        % update existing trajectory
        [BAlocations, BAorientations, BAland_marks] = parse_hidden_state(hidden_state, n, m);
        locations(:, end - n + 1 : end) = BAlocations;
        BAorientations(:, end - n + 1 : end) = BAorientations;
        
        % extract last two observations 
        counter = 3;
        Os = cell(2,1);
        for j = 1 : n
            k_i = prev_state.observations(counter);   
            if j > n - 2
                Os{j - n + 2} = prev_state.observations(counter : counter + 3 * k_i)'; 
            end
            counter = counter + 1 + 3 * k_i;
        end
        O1 = Os{1};
        O2 = Os{2};
        
        % crop landmarks to contain only points from last two observations
        [O1, O2, new_l2, point_cloud, new_m] = myCropProb(O1', O2', BAland_marks);
        
        % update previous state
        prev_state.pt_cloud =  point_cloud(:, new_l2);
        prev_state.hidden_state = ...
            [prev_state.hidden_state( 6 * n - 11 : 6 * n), point_cloud(:)'];
        prev_state.observations = [2, new_m, O1, O2];
    end
   
    % process next image and find next location and orientation
    [R, T, next_state] = processFrame(next_image, prev_img, prev_state, K, params);
    
    % collect orientations and locations
    orientations = [orientations, R(:)];
    locations = [locations, -R' * T];

    % align trajectories
    if params.alignment
        [aligned_locations, aligned_pt_cloud, alignment_params, loc_error, ori_error] = alignEstimateToGroundTruth(...
            alignment_params, ground_truth(1:i-1,  :), locations, next_state.pt_cloud, orientations);
    else
        aligned_locations = locations;
        aligned_pt_cloud = next_state.pt_cloud;
    end

    % plot results
    num_candidates_history = [num_candidates_history(2:end) size(next_state.candidates,2)];
    num_matched_kp_history = [num_matched_kp_history(2:end) size(next_state.matched_kp,2)];
    fig_num = plotPipeline(aligned_locations, aligned_pt_cloud, next_state, next_image,fig_num, num_candidates_history, num_matched_kp_history, i);
    
    % Makes sure that plots refresh.    
    pause(0.01)
    
    % update state
    prev_img = next_image;
    prev_state = next_state;
end