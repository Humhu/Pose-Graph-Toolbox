% Demo script
N = 100;
world_size = 1;

w1 = World2D([world_size,world_size]);
w1.InitRobots(N);
w1.PlotPoses
cov = zeros(2);

pause;

[ranges, angles] = w1.GetMeasurements(cov);
w1.PlotMeasurements(ranges, angles);

pause;

w2 = World2D([world_size,world_size]);
w2.InitRobots(N);
cov = 1E-1*eye(3);
noise = mvnrnd(zeros(N,3), cov);
noise = reshape(noise, N, 1, 3);
poses = w1.robot_poses + Pose2D(noise(:,:,1:2), noise(:,:,3));

info = (1E-3*eye(2))^-1;
for i = 1:10
   poses = GaussNewtonIterate(poses, ranges, angles, info);
   w2.robot_poses = poses;
   w2.PlotPoses
   w2.PlotMeasurements(ranges, angles);
   pause;
end

