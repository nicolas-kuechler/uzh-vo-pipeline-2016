function [cloud, matched_kp, remain] = ...
    tryTriangulate(candidate_kp, kp_track_start, kp_pose_start, T, K)
threshold = pi/18; % 10 deg
num_kp = size(candidate_kp, 2);
cloud = [];
matched_kp = [];
remain = ones(1, num_kp);

candidate_kp_bearings = (K * [T; zeros(1, 3), 1]) \ [candidate_kp; ones(1, num_kp)];

for i = 1 : num_kp
    T_i = reshape(kp_pose_start(:,i), 3, 4);
    u = (K * [T_i; zeros(1,3), 1]) \ [kp_track_start(:, i); 1];
    v = candidate_kp_bearings(:, i);
    
    cosTheta = dot(u, v) / (norm(u) * norm(v));
    if cosTheta < cos(threshold)
        new_3d_pt = linearTriangulation(kp_track_start(:, i), ...
            candidate_kp(:, i), K * T_i, K * T);
        cloud = [cloud, new_3d_pt];
        matched_kp = [matched_kp, candidate_kp(:, i)];
        remain(i) = 0;
    end
end
