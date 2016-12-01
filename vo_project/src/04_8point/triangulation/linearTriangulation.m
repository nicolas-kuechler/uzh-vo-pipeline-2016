% LINEARTRIANGULATION  Linear Triangulation
%
% Input:
%  - p1(2,N): coordinates of points in image 1
%  - p2(2,N): coordinates of points in image 2
%  - M1(3,4): projection matrix (includes K and [R T]) corresponding to first image
%  - M2(3,4): projection matrix (includes K and [R T]) corresponding to second image
%
% Output:
%  - P(3,N): coordinates of 3-D points in camera frame

function P = linearTriangulation(p1, p2, M1, M2)
% convert to homogeneous coordinates
p1 = [p1; ones(1, size(p1, 2))];
p2 = [p2; ones(1, size(p2, 2))];

% Sanity checks
[dim,NumPoints] = size(p1);
[dim2,npoints2] = size(p2);
assert(dim==dim2,'Size mismatch of input points');
assert(NumPoints==npoints2,'Size mismatch of input points');
assert(dim==3,'Arguments x1, x2 should be 3xN matrices (homogeneous coords)');

[rows,cols] = size(M1);
assert(rows==3 && cols==4,'Projection matrices should be of size 3x4');
[rows,cols] = size(M2);
assert(rows==3 && cols==4,'Projection matrices should be of size 3x4');

P = zeros(4,NumPoints);

% Linear algorithm
for j=1:NumPoints
    % Built matrix of linear homogeneous system of equations
    A1 = cross2Matrix(p1(:,j))*M1;
    A2 = cross2Matrix(p2(:,j))*M2;
    A = [A1; A2];
    
    % Solve the linear homogeneous system of equations
    [~,~,v] = svd(A,0);
    P(:,j) = v(:,4);
end

P = P./repmat(P(4,:),4,1); % Dehomogeneize (P is expressed in homogeneous coordinates)
P = P(1:3, :);
return


