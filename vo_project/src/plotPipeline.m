function [ fig_num ] = plotPipeline( locations, state, img, fig_num, candidates_history )
%PLOTPIPELINE Summary of this function goes here
%   Detailed explanation goes here


candidates = state.candidates;
matched_kp = state.matched_kp;


%% Initialize figure if first iteration

if fig_num == None
    fig = figure();
    fig_num = fig.Number;
end

%% Plot image with currently available candidates and currently matched keypoints

subplot(2,4,[1,2])

imshow(img)
hold on;
plot(candidates(1,:),candidates(2,:),'rx', 'Linewidth', 2);
plot(matched_kp(1,:),matched_kp(2,:),'gx', 'Linewidth', 2);



%% Plot number of tracked landmarks over the last 20 frames

subplot(2,4,5)





%% Plot full trajectory

subplot(2,4,6)



%% Plot pointcloud and trajectory over the last 20 frames

subplot(2,4,[3,4,7,8])










end

