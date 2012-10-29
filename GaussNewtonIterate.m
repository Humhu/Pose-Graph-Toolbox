function [newPoses] = GaussNewtonIterate(poses, measurements, ids, info)

% Calculate errors
N = size(poses,1);
errors = zeros(N,N,2);

ranges = [measurements.range];
angles = [measurements.bearing];

num_rels = size(ids, 2);

for k = 1:num_rels
    
    i = ids(1,k);
    j = ids(2,k);
    
    pi = poses(i);
    pix = pi.position(1);
    piy = pi.position(2);
    pit = pi.orientation;
    pj = poses(j);
    pjx = pj.position(1);
    pjy = pj.position(2);
    
    r = ranges(k);
    a = angles(k);
    
    ex = pjx - pix - r*cos(double(a + pit));
    ey = pjy - piy - r*sin(double(a + pit));
    
    errors(i,j,1) = ex;
    errors(i,j,2) = ey;
    
end

% Generate Jacobian
H = zeros(3*N);
b = zeros(3*N,1);

for k = 1:num_rels
    
    i = ids(1,k);
    j = ids(2,k);
    
    Jij = zeros(2, 3*N);
    
    pi = poses(i);
    ti = pi.orientation;
    
    r = ranges(k);
    a = angles(k);
    
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

% Have to fix one pose or else system unsolvable
H = [H; eye(3), zeros(3,3*(N-1))];
b = [b; zeros(3,1)];

dx = -H\b;

d = shiftdim(reshape(dx, 1, 3, N), 2);
dPoses = Pose2D(d(:,:,1:2), d(:,:,3));
newPoses = poses + dPoses;