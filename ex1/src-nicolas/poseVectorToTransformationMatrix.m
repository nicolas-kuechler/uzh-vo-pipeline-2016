function [ RT ] = poseVectorToTransformationMatrix( om, t)

theta = norm(om);
k = om / theta;

Kx = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];

R = eye(3) + sin(theta) * Kx + (1-cos(theta))*Kx*Kx;

RT = [R transpose(t)];
end


