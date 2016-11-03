

W = [1 4 7; 2 5 8; 3 6 9];
C = [1 3 5; 2 4 6];

Q = createQ(W,C)

[U,S,V] = svd(Q);
M = V(:,12);

%Enforce det R = 1
if M(12)<0 %to ensure that the rotation matrix R that will be extracted from M is a proper rotation matrix with determinant + 1
    M = -1*M;
end

%Reconstruct the [R|T] matrix
M = [M(1) M(2) M(3) M(4); M(5) M(6) M(7) M(8); M(9) M(10) M(11) M(12)]

%Extracting a rotation matrix from R
R = M(1:3,1:3);
[U,S,V] = svd(R);
Rtilde = U * V;

a = norm(Rtilde, 'fro')/norm(R, 'fro');

Mtilde = [Rtilde a*M(1:3,4)];
