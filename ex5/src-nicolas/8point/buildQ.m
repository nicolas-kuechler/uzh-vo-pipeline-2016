function [ Q ] = buildQ( p1, p2 )

N = size(p1,2);
Q = zeros(N,9);

for i=1:N
    Q(i,:) = kron(p1(:,i),p2(:,i))';
end

end

