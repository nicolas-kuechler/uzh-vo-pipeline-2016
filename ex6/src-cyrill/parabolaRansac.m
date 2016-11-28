function [best_guess_history, max_num_inliers_history] = ...
    parabolaRansac(data, max_noise)
% data is 2xN with the data points given column-wise, 
% best_guess_history is 3xnum_iterations with the polynome coefficients 
%   from polyfit of the BEST GUESS at each iteration columnwise and
% max_num_inliers_history is 1xnum_iterations, with the inlier count of the
%   BEST GUESS at each iteration.

s = 3;
k = 100;

best_guess_history = zeros(3,1);
max_num_inliers_history = zeros(1,1);

rng(2);

for i = 1:k
    
    rand_points = datasample(data,s,2, 'Replace', false);
    poly_guess = polyfit(rand_points(1,:), rand_points(2,:), 2);
    
    dists = abs(data(2,:) - polyval(poly_guess, data(1,:)));
    
    num_inliers = numel(dists(dists <= max_noise));
    
    if i == 1 || num_inliers > max_num_inliers_history(i-1)
        best_guess_history(:,i) = poly_guess;
        max_num_inliers_history(i) = num_inliers;
    else
        best_guess_history(:,i) = best_guess_history(:,i-1);
        max_num_inliers_history(i) = max_num_inliers_history(i-1);
    end
end

best_guess = best_guess_history(:,end);
dists = abs(data(2,:) - polyval(best_guess, data(1,:)));

inliers = data(:,dists <= max_noise);

best_guess_opt = polyfit(inliers(1,:), inliers(2,:), 2);

best_guess_history(:,k+1) = best_guess_opt;
end

