function [poses] = InitRobots(N, world_size)

x = world_size(1);
y = world_size(2);

positions = bsxfun(@times, [x;y], rand(2,N));
orientations = 2*pi*rand(N,1);
poses = Pose2D(positions, orientations);

