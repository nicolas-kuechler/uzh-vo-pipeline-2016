clc;
clear all;
close all;

addpath(genpath('./'));
rng(1);

ori_errors = [];
loc_errors = [];

%% Setup
ds = 3; % 0: KITTI, 1: Malaga, 2: parking
kitti_path = '../data/kitti';
malaga_path = '../data/malaga-urban-dataset-extract-07';
parking_path = '../data/parking';
waedi_path = '../data/waedi';

if ds == 0
    % need to set kitti_path to folder containing "00" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/00.txt']);
    % ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
elseif ds == 1
    % Path containing the many files of Malaga 7.
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
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
elseif ds == 3
    
    assert(exist('waedi_path', 'var') ~= 0);
    images = dir([waedi_path ...
        '/images']);
    K = [609.932619643408,0,0;0,610.173360091855,0;329.144287081628,181.258698042663,1]';
    last_frame = 4396;
    
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
elseif ds == 3
    img0 = imread([waedi_path ...
        sprintf('/images_undistorted/waedi_%04d.png', bootstrap_frames(1))]);
    img1 = imread([waedi_path ...
        sprintf('/images_undistorted/waedi_%04d.png',bootstrap_frames(2))]);
else
    assert(false);
end

% Initialize Parameters --> TODO Maybe different tuning for each dataset
% TODO: optimize parameters to reduce reprojection error, pose error
% (compared to ground truth)
params = struct(...
    'harris_patch_size', 9, ...
    'harris_kappa', 0.08, ...
    'num_keypoints', 800, ...
    'nonmaximum_supression_radius', 10, ...
    'descriptor_radius', 13,...  %desc radius 9,...
    'match_lambda', 8, ...
    'triangulation_angle_threshold', 3,...
    'surpress_existing_matches', 1 ,... %1 for true, 0 for false
    'candidate_cap', 700,...
    'add_candidate_each_frame', 100 ,...
    'eWCP_confidence', 99.0, ...
    'eWCP_max_repr_error', 1, ...
    'triangulate_max_repr_error', 200000000);


[R, T, repr_error, pt_cloud, ~, keypoints_r] = initializePointCloudMono(img0,img1,K, params);

% state 
prev_state = struct('pt_cloud', pt_cloud, ...
                    'matched_kp', keypoints_r, ...
                    'cam_transformation', [R, T], ...
                    'candidates', [], ...
                    'candidates_start', [], ...
                    'candidates_start_pose', []);
                
locations = [zeros(3,1), -R' * T / 2, -R' * T];
orientations = [reshape(eye(3), 9, 1), R(:), R(:)];

%% Continuous operation

fig_num = NaN;
%ring buffer for number of candidates history
num_candidates_history = nan(1,20);


range = (bootstrap_frames(2)+1):last_frame;
prev_img = img1;
for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        next_image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        next_image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        next_image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    elseif ds == 3
        next_image = im2uint8(imread([waedi_path ...
            sprintf('/images_undistorted/waedi_%04d.png',i)]));
        
    else
        assert(false);
    end
    tic;
    [R, T, next_state, debug_data, plot_pose ] = processFrame(next_image, prev_img, prev_state, K, params);
    disp(['ProcessFrame took: ' num2str(toc) ' seconds']);
    % collect orientations and locations
    orientations = [orientations, R(:)];
    if plot_pose
        locations = [locations, -R' * T];
    else
        locations = [locations, locations(:, end)];
    end
    % align trajectories
%     [aligned_locations, loc_error, ori_error] = alignEstimateToGroundTruth(...
%         ground_truth(1:i, :), locations, orientations);
%     ori_errors = [ori_errors, loc_error / i];
%     loc_errors = [loc_errors, ori_error / i];

    %%% PLOT
    %plotTrajectory(locations, orientations, next_state.pt_cloud, 100);
    num_candidates_history = [num_candidates_history(2:end) size(next_state.candidates,2)];
    fig_num = plotPipeline(locations,next_state,next_image,fig_num, num_candidates_history);

    
    % Makes sure that plots refresh.    
    pause(0.01)
        
    prev_img = next_image;
    prev_state = next_state;
end
