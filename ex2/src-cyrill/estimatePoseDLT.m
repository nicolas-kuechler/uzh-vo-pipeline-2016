function [ Mtilde ] = estimatePoseDLT( p, P, Kinv )
%ESTIMATEPOSEDLT Summary of this function goes here
%   Detailed explanation goes here

Q = buildQ(p, P, Kinv);

[~,~,V] = svd(Q);

M = V(:,end);

M = reshape(M, [4,3]);

M = M';

if M(3,4) < 0
    M = M * -1;
end

R = M(:,1:3);

t = M(:,4);
[Ur,~,Vr] = svd(R);
Rtilde = Ur * Vr;
alpha = norm(Rtilde, 'fro') / norm(R, 'fro');

Mtilde = [Rtilde alpha*t];

end

