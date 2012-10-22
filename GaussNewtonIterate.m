function [newPoses] = GaussNewtonIterate(poses, errors, cov)

N = size(poses,1)*size(poses,2);
H = zeros(2*N);
b = zeros(2*N,1);

for i = 1:N
    for j = 1:N
        
        Jij = zeros(2, 2*N);
        Jij(:, 2*i - 1:2*i) = -eye(2);
        Jij(:, 2*j - 1:2*j) = eye(2);
        
        Hij = Jij'*cov*Jij;        
        H = H + Hij;
        
        e = reshape(errors(i,j,:), 2, 1);
        bij = e'*cov*Jij;
        b = b + bij';
    end
end

dx = -H\b;

d = reshape(dx, 2, N)

dPoses = Pose2D(d, zeros(N,1));
newPoses = poses + dPoses;