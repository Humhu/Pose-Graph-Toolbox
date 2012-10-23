function [newPoses] = GaussNewtonIterate(poses, ranges, angles, info)

% Calculate errors
N = size(poses,1);
errors = zeros(N,N,2);

for i = 1:N
    for j = 1:N
        
        pi = poses(i);
        pix = pi.position(1);
        piy = pi.position(2);
        pit = pi.orientation;
        pj = poses(j);
        pjx = pj.position(1);
        pjy = pj.position(2);
        
        r = ranges(i,j);
        a = angles(i,j);
        
        ex = pjx - pix - r*cos(double(a + pit));
        ey = pjy - piy - r*sin(double(a + pit));
        
        errors(i,j,1) = ex;
        errors(i,j,2) = ey;
        
    end
end

% Generate Jacobian
H = zeros(3*N);
b = zeros(3*N,1);

for i = 1:N
    for j = 1:N
        
        Jij = zeros(2, 3*N);
        
        pi = poses(i);
        ti = pi.orientation;
        
        r = ranges(i,j);
        a = angles(i,j);
        
        dt = [  r*sin(double(ti + a));
                -r*cos(double(ti + a))];
        Jij(:, 3*i - 2:3*i) = [-eye(2), dt];
        Jij(:, 3*j - 2:3*j) = [eye(2), zeros(2,1)];
        
        Hij = Jij'*info*Jij;
        H = H + Hij;
        
        e = reshape(errors(i,j,:), 2, 1);
        bij = e'*info*Jij;
        b = b + bij';
        
    end
end

dx = -H\b

d = shiftdim(reshape(dx, 1, 3, N), 2);
dPoses = Pose2D(d(:,:,1:2), d(:,:,3));
newPoses = poses + dPoses;