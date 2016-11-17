function [best_guess_history, max_num_inliers_history] = ...
    parabolaRansac(data, max_noise)
% data is 2xN with the data points given column-wise,
% best_guess_history is 3xnum_iterations with the polynome coefficients
%   from polyfit of the BEST GUESS at each iteration columnwise and
% max_num_inliers_history is 1xnum_iterations, with the inlier count of the
%   BEST GUESS at each iteration.

s=3;
k=100;
N = size(data,2);

best_guess_history = zeros(3,k);
max_num_inliers_history = zeros(1,k);
max_num_inliers = 0;

%1st s Random Guess
rng(2);

ind = 1:N;
best_poly = [0,0,0];
for i=1:k
    rand_points = datasample(data,s,2,'Replace',false);
    
    %2nd fit polynom
    p = polyfit(rand_points(1,:),rand_points(2,:),2);
    
    %3rd calc |y_i - m(x_i)|
    m = polyval(p,data(1,:));
    err = abs(data(2,:)-m);
    
    guess_index = ind(err<=max_noise)
    num_inliers = numel(guess_index);

    
    if(num_inliers > max_num_inliers)
        max_num_inliers = num_inliers;
        best_guess_index = guess_index;
        best_poly = p;
    end
    
        max_num_inliers_history(1,i) = max_num_inliers;
        best_guess_history(:,i) = best_poly;
end
    
    p = polyfit(data(1,best_guess_index),data(2,best_guess_index),2);
    m = polyval(p, data(1,:));
    err = abs(data(2,:)-m);
    guess_index = ind(err<=max_noise);
    num_inliers = numel(guess_index);
    max_num_inliers_history(1,k+1) = num_inliers;
    best_guess_history(:,k+1) = p;
end

