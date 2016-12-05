function [cloud, matched_kp, remain, maxAngle] = ...
    tryTriangulate(candidate_kp, kp_track_start, kp_pose_start, T, K)
threshold = 5;
num_kp = size(candidate_kp, 2);
cloud = [];
matched_kp = [];
remain = true(1, num_kp);
t = [];
candidate_kp_bearings = K \ [candidate_kp; ones(1, num_kp)];

for i = 1 : num_kp
    T_i = reshape(kp_pose_start(:,i), 3, 4);
    u = K \ [kp_track_start(:, i); 1];
    u = T(:,1:3)' * T_i(:, 1:3) * u;

    v = candidate_kp_bearings(:, i);
    
    
    Theta = 180 / pi * acos(dot(u/norm(u), v/norm(v)));
    t = [t, Theta];
    if Theta > threshold
        new_3d_pt = linearTriangulation(kp_track_start(:, i), ...
            candidate_kp(:, i), K * T_i, K * T);
        
        if new_3d_pt(3) > 0
            cloud = [cloud, new_3d_pt];
            matched_kp = [matched_kp, candidate_kp(:, i)];
        end
        remain(i) = false;    
    end
    
end
maxAngle = max(t);
end
