function [estimate] = EstimatePositions(positions, measurements)

% Assume robot 0 is at origin
N = size(positions, 2);
M = size(measurements, 1)^2 - N;

A = zeros(M, 2*N);
b = zeros(M, 1);

k = 1;

for i = 1:N
    for j = i+1:N
    
        A(k:k+1, 2*i-1:2*i) = -eye(2);
        A(k:k+1, 2*j-1:2*j) = eye(2);
        b(k:k+1) = reshape(measurements(i,j,:),2,1);
        k = k + 2;
        
    end
end

A = [eye(2), zeros(2, 2*N - 2);
    A];
b = [zeros(2,1);
    b];

%estimate = reshape((A'*A)^-1*A'*b, 2, N);
estimate = reshape(A\b, 2, N);