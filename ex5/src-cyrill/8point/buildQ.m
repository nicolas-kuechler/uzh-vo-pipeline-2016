function [ Q ] = buildQ( p1,p2 )
%BUILDQ Summary of this function goes here
%   Detailed explanation goes here

    N = size(p1,2);
    Q = zeros(N,9);
    
    for i = 1:N
        
       Qi = kron(p1(:,i),p2(:,i)); 
       Q(i,:) = Qi;
       
    end
end

