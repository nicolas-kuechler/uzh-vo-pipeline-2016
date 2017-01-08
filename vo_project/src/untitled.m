clear all
load('locations.mat');
load('ground_truth.mat');
i = 150;

cla();

[aligned_locations, x, location_error] = ...
    alignEstimateToGroundTruth([0,0,0,0,0,0,1]', ground_truth(1:i,:), locations(:,1:i));

figure(2)
hold on
plot3(locations(1,1:i),locations(2,1:i),locations(3,1:i), 'g-', 'LineWidth',2);
plot3(aligned_locations(1,:),aligned_locations(2,:),aligned_locations(3,:),'r-','LineWidth',2);
plot3(ground_truth(1:i,  4),ground_truth(1:i,  8),ground_truth(1:i,  12),'b-.','LineWidth',2);

view([0,-1,0]);
daspect([4 1 5]);
pbaspect([4 1 4.1]);

title('Trajectory Comparisons');
l = legend('not aligned','aligned', 'true');
set(l, 'Position', [.705, 0.845, .05, .05])

hold off