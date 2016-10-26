function [ Q ] = createQ( W, C )
%CREATEQ Summary of this function goes here
%   Detailed explanation goes here
%       X1 X2 ... Xn
% W =   Y1 Y2 ... Yn   --> World Coordinates
%       Z1 Z2 ... Zn

% C =   x1 x2 ... xn --> Camera Coordinates
%       y1 y2 ... yn
%

% C = [x1 y1 x2 y2 ... xn yn] --> Camera Coordinates

n = length(C(1,:));
h = 1;
Q = zeros(2*n,12);

for i = 1:n
    x = C(1,i);
    y = C(2, i);
    Xw = W(1,i);
    Yw =  W(2,i);
    Zw = W(3,i);
    
    Q(h,:) = [Xw Yw Zw 1 0 0 0 0 -x*Xw -x*Yw -x*Zw -x];
    
    h = h+1;
      
    Q(h,:) = [0 0 0 0 Xw Yw Zw 1 -y*Xw -y*Yw -y*Zw -y];
    h = h+1;
end

end

