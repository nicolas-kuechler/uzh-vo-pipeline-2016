clear all
load('locations.mat');
load('ground_truth.mat');
i = 200;


[aligned_locations, x, location_error] = ...
    alignEstimateToGroundTruth([0,0,0,0,0,0,1], ground_truth(1:i,:), location_estimate(1:i,:))

figure(2)
hold on
plot3(locations(1,1:i),locations(2,1:i),locations(3,1:i),'LineWidth',2);
plot3(aligned_locations(1,:),aligned_locations(2,:),aligned_locations(3,:),'LineWidth',2);
plot3(ground_truth(1:i,  4),ground_truth(1:i,  8),ground_truth(1:i,  12),'LineWidth',2);
view([0,-1,0]);
daspect([4 1 5]);
pbaspect([4 1 5]);
title('Trajectory Comparisons');
legend('not aligned','aligned', 'true')