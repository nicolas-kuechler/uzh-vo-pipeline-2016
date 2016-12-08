function debugPlot(ground_truth,gt_i, image, next_state, debug_data,next_T, Ts, K)

    pt_cloud = next_state.pt_cloud;
    candidate_kp = next_state.candidate_kp;
    kp_track_start = next_state.kp_track_start;
    figure(2);
    subplot(2,2,1);
    plot(Ts(1,:)./10, Ts(3,:)./10, ground_truth(1:gt_i,1), ground_truth(1:gt_i,2));
    legend('Ts','ground truth')
    
    subplot(2,2,2);
    plot3(Ts(1,:),Ts(2,:), Ts(3,:))
    grid on
    
    subplot(2,2,[3 4]);
    imshow(image);
    hold on;
    % plot new matched keypoints
    plot(debug_data.curr_matched_kp(1,:), debug_data.curr_matched_kp(2,:), 'rx', 'Linewidth', 2);
    
%     % plot old matched keypoints
%     plot(matched_kp(1,:), matched_kp(2,:), 'mv', 'Linewidth', 2);
%     
%     % plot correspondences between old and new matched keypoints
%     quiver(matched_kp(1,:),matched_kp(2,:),...
%         -matched_kp(1,:)+prop_matched_kp(1,:), -matched_kp(2,:)+prop_matched_kp(2,:), 0, 'm');
%     
     % plot reprojected 3D points (should coincide with new_matched_kp)
    next_keypoints_reprojected = reprojectPoints(pt_cloud, next_T, K);
    plot(next_keypoints_reprojected(1,:), next_keypoints_reprojected(2,:), ...
         'bx', 'Linewidth', 2);
    rms_repr_error = sqrt(sum((next_keypoints_reprojected(:) - debug_data.curr_matched_kp(:)).^2) / size(debug_data.curr_matched_kp, 2));
    text(10,10, ['RMS Reprojection error = ' num2str(rms_repr_error)], 'Color', [0,1,0]); 
     
%     % plot new candidate keypoints
%     plot(next_candidate_kp(1,:), next_candidate_kp(2,:), 'bx', 'Linewidth', 2);
%     
%     % plot previous candidate keypoints
    plot(candidate_kp(1,:), candidate_kp(2,:), 'cv', 'Linewidth', 2);
    
%     % plot correspondences between old and new candidate keypoints
%     quiver(candidate_kp(1,:),candidate_kp(2,:),...
%         -candidate_kp(1,:)+prop_candidate_kp(1,:), -candidate_kp(2,:)+prop_candidate_kp(2,:), 0, 'c');
%     
    % plot track start of each candidate keypoints
    plot(kp_track_start(1,:), kp_track_start(2,:), 'go', 'Linewidth', 2);
    
    % plot correspondences between track start and old candidate keypoints
    quiver(kp_track_start(1,:),kp_track_start(2,:),...
        -kp_track_start(1,:)+candidate_kp(1,:), -kp_track_start(2,:)+candidate_kp(2,:), 0, 'c');

%      % plot correspondences between track start and old candidate keypoints
%      quiver(next_kp_track_start(1,:),next_kp_track_start(2,:),...
%          -next_kp_track_start(1,:)+next_candidate_kp(1,:), -next_kp_track_start(2,:)+next_candidate_kp(2,:), 0, 'c');
%     
    legend('current matched keypoints','reprojected 3D points','Location','NorthOutside')
    title(['Red x: current matched keypoints, Magenta v: previous matched keypoints,' ...
           'Magenta o: reprojected point cloud, Blue x: current candidate keypoints,' ...
           'Cyan v: previous candidate keypoints, Cyan o: track starts']);
       
    hold off;
    pause(0.01);
    %waitforbuttonpress;


end