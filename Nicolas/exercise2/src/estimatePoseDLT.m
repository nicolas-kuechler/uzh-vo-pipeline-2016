function [ M] = estimatePoseDLT(p, P, K )
%ESTIMATEPOSEDLT Summary of this function goes here
%   Detailed explanation goes here

% TODO Check if necessary

xy = K\p;
%xy = p;

Q = createQ(P,xy(1:2, :));

[U,S,V] = svd(Q);
M = V(:,12);


%[Q,R] = qr(M)

%M=R;
%K = Q;

M = reshape(M,[4,3])';
%Enforce det R = 1
if M(3,4)<0 %to ensure that the rotation matrix R that will be extracted from M is a proper rotation matrix with determinant + 1
    M = -M;
end

%Reconstruct the [R|T] matrix


%Extracting a rotation matrix from R
R = M(:,1:3);
[U,S,V] = svd(R);
Rtilde = U * V;

a = norm(Rtilde, 'fro')/norm(R, 'fro');

M = [Rtilde a*M(:,4)];

%detR = det(Rtilde);
%tRtilde = Rtilde'  ;
%invrTilde = inv(Rtilde);
end

