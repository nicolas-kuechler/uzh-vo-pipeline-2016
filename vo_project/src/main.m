clc;
clear all;
close all;

addpath(genpath('./'));
rng(1);

%% Setup
ds = 0; % 0: KITTI, 1: Malaga, 2: parking
kitti_path = '../data/kitti';
malaga_path = '../data/malaga-urban-dataset-extract-07';
parking_path = '../data/parking';

if ds == 0
    % need to set kitti_path to folder containing "00" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
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

% Initialize Parameters --> TODO Maybe different tuning for each dataset
% TODO: optimize parameters to reduce reprojection error, pose error
% (compared to ground truth)
params = struct(...
    'harris_patch_size', 9, ...
    'harris_kappa', 0.08, ...
    'num_keypoints', 1000, ...
    'nonmaximum_supression_radius', 15, ...
    'descriptor_radius', 9,...
    'match_lambda', 7);

[R1, T1, repr_error, pt_cloud, ~, keypoints_r] = initializePointCloudMono(img0,img1,K, params);

% state 
prev_state = struct('pt_cloud', pt_cloud, ...
                    'matched_kp', keypoints_r, ...
                    'candidate_kp', [], ...
                    'kp_track_start', [], ...
                    'kp_pose_start', []);
                
Ts = T1;

%% Continuous operation

%Global config
debug = true; %enable debug data collection
playback_mode = false; %save frames to trace errors, requres active debug mode
plot_mode = true; 
window_max_size = 20;

if playback_mode
    window = {};
    
    window_params = struct('window_index', 0, ...
                        'window_size', 0, ...
                        'window_max_size', window_max_size);
                    
    clearvars gui %clear this, otherwise the handle is invalid when running in ctrl+enter mode
end

Ts = [0 0 0]';
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
    else
        assert(false);
    end
    
    [next_T, next_state, debug_data ] = processFrame(next_image, prev_img, prev_state, K, params, ...
        'debug', debug);  %debug enabled
    R_pose = next_T(:,1:3)';
    t_pose = -R_pose * next_T(:,4);
    
    %%%PLOT ALTERNATIVE
    
    
    cameraSize = 0.1;
    figure(100);
    plotCamera('Location',t_pose,'Orientation',R_pose,'Size',...
        cameraSize,'Color','r','Label',num2str(i),'Opacity',.5);
    hold on

    pcshow(next_state.pt_cloud','VerticalAxis','y','VerticalAxisDir',...
        'down','MarkerSize',45);
    grid on
    %%%PLOT ALTERNATIVE END
    
    
    % Makes sure that plots refresh.    
    Ts = [Ts, next_T(:,4)];
    
    if debug && plot_mode 
        debugPlot(ground_truth,i,next_image, next_state, ...
            debug_data,next_T, Ts, K);
    end

    if debug && playback_mode 
        window_params.window_index = mod(window_params.window_index, ...
            window_params.window_max_size)+1;
        
        window{window_params.window_index} = struct('debug_data',debug_data, ...
            'image', next_image, ...
            'translations', Ts);        if window_params.window_size < 20
            window_params.window_size = window_params.window_size+1;
        end
        
        if ~exist('gui','var')
            gui = debugGui(window,window_params, struct());
        else
            gui = debugGui(window,window_params, gui);
        end
    end
    
    prev_img = next_image;
    prev_state = next_state;
end
